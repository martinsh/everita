AFRAME.registerShader('my-custom', {
      // The schema declares any parameters for the shader.
      schema: {
        srcL: {type:'map', is:'uniform'},
        srcC: {type:'map', is:'uniform'},
        srcR: {type:'map', is:'uniform'},
        ground: {type:'map', is:'uniform'},
        wireframe: {default: true}
      },
                       
      raw: false,
      
      vertexShader:
    `
      varying vec2 vUV;
      varying vec3 p, c;

      void main(void) {
      gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);

      c = cameraPosition;
      p = vec3(modelMatrix * vec4(position,1.0));

      vUV = position.xy;
      }
    `
    ,
      fragmentShader: 
    `
      precision mediump float;
      #define PI			3.141592653589793
      #define SQRT2		1.414213562373095
      #define INVSQRT2	0.707106781186548


      uniform sampler2D srcL, srcC, srcR;
      uniform sampler2D ground;
      varying vec2 vUV;
      varying vec3 p, c;

      struct Material
      {
          float roughness;
          float tailamount;
          float tailtheta;
          float F0;
          vec3 basecolor;
          vec3 specularcolor;
      };

      //uniform float opacity;
      struct RectLight
      {
          vec3 position;
          mat3 basis;
          vec2 size;
          vec3 intensity;
          float attenuation;
      };

      Material mat = Material(
          0.02, // Roughness.
          0.1, // Tail amount.
          PI/2.0/3.0, // Specular cone tail theta angle.
          0.505, // Schlick Fresnel coefficient for zero viewing angle.
          vec3(1.0), // Base color.
          vec3(2.0) // Specular color.
      );

      RectLight lightL = RectLight(
        vec3(-6.0, 1.6, -2.0), // Light position (center).
        mat3( // Light basis.
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ),
        vec2(2.560, 1.440), // Light size.
          vec3(100.0), // Light intensity.
        120.0 // Constant attenuation at 0 distance.
      );

      RectLight lightC = RectLight(
        vec3(0.0, 1.6, -4.0), // Light position (center).
        mat3( // Light basis.
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ),
        vec2(2.560, 1.440), // Light size.
          vec3(100.0), // Light intensity.
        120.0 // Constant attenuation at 0 distance.
      );

      RectLight lightR = RectLight(
        vec3(6.0, 1.6, -2.0), // Light position (center).
        mat3( // Light basis.
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ),
        vec2(2.560, 1.440), // Light size.
          vec3(100.0), // Light intensity.
        120.0 // Constant attenuation at 0 distance.
      );

      float sqr(float x) { return x*x; }

      vec3 RectLight_calcWeight(in vec3 P, in vec3 R, in RectLight light, float theta)
      {
          // Intersect ray and light plane.
          float RoPlN = dot(R, light.basis[2]);
          float d = dot(light.basis[2], light.position - P) / RoPlN;
          if (d < 0.0 || RoPlN > 0.0) {
              // Intersection behind ray, or direction is away from plane.
            return vec3(0.0);
          }
          // PlC: Point on plane.
          vec3 PlC = P + d*R - light.position;
          // uvPl: UV coordinate on plane.
          vec2 PlUV = vec2(dot(PlC, light.basis[0]), dot(PlC, light.basis[1]));
          // r: Radius of cone at distance d.
          float r = d * tan(theta);
          // s: Rect size shifted by radius. This for weigth 1 inside the rect.
          vec2 s = max(light.size - 0.5*r, 0.0);
          // h: Distance from rect on plane.
          float h = length(max(abs(PlUV) - s, 0.0));
          // sr: Steradians from the sphere cap equation: sr = 2pi * (1-cos(a))
          float sr = 2.0*PI * (1.0 - cos(theta));
          // This distribution f(x) has variance v^2 = 1/2, hence
          // v = (1/2)^(1/2) = 2^(-1/2) = INVSQRT2. Using this we can
          // linearly map h = [0,2r] -> x = [0,3v]. Why 3v is to cover
          // approximately 100% of the distribution. 
          return vec3(exp(-sqr((3.0*INVSQRT2/1.5) * (h/r))) / (light.attenuation + sqr(d)*sr),vec2(PlUV.x/light.size.x,PlUV.y/light.size.y)*0.5+0.5);
      }

      vec3 RectLight_shade(in RectLight light, in Material material, in sampler2D src ,in vec3 P, in vec3 N, in vec3 R, float NoR)
      {
          // Schlick Fresnel.
          float Fr = material.F0 + (1.0-material.F0) * pow(1.0 - NoR, 5.0);

          // Approximate specular/glossy.
          float theta = mix(PI*0.003, PI/2.0/3.0, material.roughness);
          vec3 Cs = RectLight_calcWeight(P, R, light, theta);
          // Specular glossy tail. Using other than Gaussian could help. 
          float Cst = RectLight_calcWeight(P, R, light, material.tailtheta).x;

          // Crude hack for diffuse.
          // Average normal and inversed emitter direction to create
          // a vector W that points towards the light.
          vec3 W = normalize(N - light.basis[2]);
          float Cd = RectLight_calcWeight(P, W, light, PI/4.0).x;
          vec3 ref = texture2D(src,Cs.yz,4.0).rgb;
          //vec3 lightcolor = texture2D(src,vec2(0.5,0.5),4.0).rgb;
          vec3 lightcolor = vec3(1.0);
          return light.intensity * lightcolor * mix(
              (Cd * max(dot(N, W), 0.0)) * material.basecolor,
              (mix(Cs.x, Cst, material.tailamount) * NoR) * material.specularcolor * ref,
              Fr);
      }

    mat3 rotationY( in float angle ) {
      angle = angle * PI / 180.0;
	    return mat3(	cos(angle),		0,		sin(angle),
			 				            0,		1.0,			 0,
					          -sin(angle),	0,		cos(angle));
    }

      void main () {
        vec3 sandColor = texture2D(ground,vec2(vUV.x,vUV.y)*0.05,0.0).rgb;
        vec3 lightColor = vec3(0.32,0.57,0.75)*0.6;
        // Normal and reflection vectors.
        vec3 N = normalize(vec3(0.0,1.0,0.0));
        vec3 D = normalize(p-c);
        float NoR = -dot(N, D);
        vec3 R = D + (2.0*NoR)*N;
        NoR = max(NoR, 0.0);

        mat.basecolor = vec3(sandColor*0.5);
        mat.roughness = pow(sandColor.r*1.0,0.5);
        mat.specularcolor = pow(sandColor,vec3(2.0));
        lightL.basis *= rotationY(-40.0);
        lightR.basis *= rotationY(40.0);
        vec3 color = RectLight_shade(lightL, mat, srcL, p, N, R, NoR);
        color += RectLight_shade(lightC, mat, srcC, p, N, R, NoR);
        color += RectLight_shade(lightR, mat, srcR, p, N, R, NoR);

        gl_FragColor = vec4(color, 1.0);
      }
    `
    });