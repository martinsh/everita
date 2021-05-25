AFRAME.registerShader('sphere_riga', {
  schema: {
    timeMsec: {type:'time'},
  },
  
  uniforms: {
      // the texture value (once the texture source is loaded, update)
      //map: { type: 't', value: null },
      // texture parameters
      timeMsec: {type:'time', is:'uniform'},
  },

  init: function(data) {
      
      this.material = new THREE.ShaderMaterial({

          uniforms: this.uniforms,
          vertexShader: this.vertexShader,
          fragmentShader: this.fragmentShader
        
      });
  },
                               



  update: function (data) {
    //console.log(data.something)

      //AFRAME.utils.material.updateMap(this, data);
       this.uniforms.timeMsec.value = data.timeMsec;
       //this.uniforms.interpolate.value = (data.timeMsec%500.0)/500;
       //this.uniforms.something.value = data.something;
      //console.log(data.timeMsec)
      //console.log((data.timeMsec%500.0)/500)
    
      //if ((data.timeMsec%500.0)-this.timeBefore < 0.0){
      //  console.log('tick')
      //  setTimeout(animate,2000);
        //animate();
      //}
      //console.log(this.timeBefore, data.timeMsec, (data.timeMsec%500.0)-this.timeBefore )
      //this.timeBefore = (data.timeMsec%500.0);
     
   },

  
  vertexShader: `


varying vec3 p;


void main() {
  p = vec3(vec4(position,1.0));
  gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );

}

`,
  fragmentShader: `

//varying float noise;
varying vec3 p;
const float PI = 3.14159;
uniform float timeMsec;

float map(float value, float min1, float max1, float min2, float max2)
{
  // Convert the current value to a percentage
  // 0% - min1, 100% - max1
  float perc = (value - min1) / (max1 - min1);

  // Do the same operation backwards with min2 and max2
  return perc * (max2 - min2) + min2;
}

float chladni(vec3 pos, float k,float l,float m,float s,float L)
{

return cos(k*pos.x*PI/L)*(cos(l*pos.y*PI/L)*cos(m*pos.z*PI/L)+s*cos(m*pos.y*PI/L)*cos(l*pos.z*PI/L))+
       cos(l*pos.x*PI/L)*(cos(k*pos.y*PI/L)*cos(m*pos.z*PI/L)+s*cos(m*pos.y*PI/L)*cos(k*pos.z*PI/L))+
       cos(m*pos.x*PI/L)*(cos(k*pos.y*PI/L)*cos(l*pos.z*PI/L)+s*cos(l*pos.y*PI/L )*cos(k*pos.z*PI/L));// where k, l and m are integers, s=∓ 1.
}

float[] k = float[](1016.6630249023438, 1013.0830078125, 1007.1329956054688, 998.3170166015625, 1017.6500244140625, 1019.5679931640625, 1020.1329956054688, 1013.625, 1012.9210205078125, 1009.8499755859375, 1019.1829833984375, 1010.1209716796875, 1011.1420288085938, 1013.5579833984375, 1009.1669921875, 1016.2210083007812, 1025.1080322265625, 1017.4749755859375, 1024.2960205078125, 1031.3599853515625, 1018.9879760742188, 1007.1209716796875, 1007.8829956054688, 1011.60400390625, 1010.2249755859375, 1002.4290161132812, 994.8579711914062, 998.5880126953125, 995.6829833984375, 994.2830200195312, 986.31298828125, 994.2420043945312, 1001.875, 1014.8499755859375, 1012.4920043945312, 1025.116943359375, 1020.8209838867188, 1011.6079711914062, 979.9710083007812, 981.1079711914062, 988.5040283203125, 1000.1380004882812, 1014.4290161132812, 1020.8419799804688, 1007.8419799804688, 998.7039794921875, 1003.3670043945312, 1005.1920166015625, 1012.7999877929688, 1010.5830078125, 1003.5, 991.4500122070312, 999.6129760742188, 997.0750122070312, 987.5880126953125, 990.5289916992188, 999.2459716796875, 1003.4669799804688, 994.6290283203125, 998.8499755859375, 1002.5540161132812, 1004.2750244140625, 1005.1500244140625, 1006.81298828125, 1008.8170166015625, 1014.5919799804688, 1013.9080200195312, 1006.8829956054688, 990.97900390625, 989.5499877929688, 994.6790161132812, 1011.1580200195312, 1022.8829956054688, 1014.85400390625, 1021.3629760742188, 1016.0999755859375, 1016.0960083007812, 1016.0960083007812, 1020.3829956054688, 1027.4630126953125, 1038.175048828125, 1041.4000244140625, 1038.125, 1034.9830322265625, 1030.8170166015625, 1019.6480102539062, 1017.60400390625, 1019.56298828125, 1021.5540161132812, 1006.0999755859375, 997.3629760742188, 993.8460083007812, 1015.0460205078125, 1030.154052734375, 1031.9630126953125, 1025.154052734375, 1026.428955078125, 1013.9039916992188, 1019.8499755859375, 1024.5169677734375, 1014.4580078125, 998.89599609375, 1004.4130249023438, 1003.7999877929688, 999.1500244140625, 1009.0170288085938, 1014.7379760742188, 1018.5460205078125, 1025.0460205078125, 1027.887939453125, 1026.470947265625, 1017.5579833984375, 1005.5579833984375, 1002.6500244140625, 1005.3880004882812, 1009.8170166015625, 1002.5040283203125, 1006.0709838867188, 1011.9580078125, 1008.9420166015625, 1003.8289794921875, 1008.2039794921875, 1013.5540161132812, 1012.6699829101562, 1010.3099975585938, 1012.6699829101562, 1010.8099975585938, 1001.0670166015625, 1004.9829711914062, 1006.8880004882812, 1011.7249755859375, 1010.3629760742188, 1006.9249877929688, 1007.6329956054688, 1010.3629760742188, 1006.9249877929688, 1007.6329956054688, 1010.5040283203125, 1012.2210083007812, 1016.8330078125, 1018.4039916992188, 1021.9130249023438, 1021.7630004882812, 1016.6129760742188, 1022.1129760742188, 1030.2919921875, 1027.0789794921875, 1024.9169921875, 1024.3170166015625, 1024.22900390625, 1020.6829833984375, 1020.8460083007812, 1021.8579711914062, 1016.625, 1009.7000122070312, 1002.875, 998.9630126953125, 1000.4420166015625, 1010.1829833984375, 1009.2999877929688, 1014.7919921875, 1013.8419799804688, 1015.8790283203125, 1020.1129760742188, 1019.6710205078125, 1013.7880249023438, 1014.0919799804688, 1014.02099609375, 1011.5670166015625, 1012.0830078125, 1012.8170166015625, 1015.5830078125, 1015.2999877929688, 1017.8209838867188, 1024.196044921875, 1026.7130126953125, 1023.31298828125, 1014.4920043945312, 1008.9829711914062, 1007.3419799804688, 1000.5, 1001.8670043945312, 1005.3709716796875, 1010.4500122070312, 1011.6790161132812, 1006.1290283203125, 1002.9290161132812, 1007.125, 1008.9829711914062, 1012.0380249023438, 1010.4169921875, 1010.375, 1017.9539794921875, 1017.4119873046875, 1016.2630004882812, 1014.0540161132812, 1012.7379760742188, 1016.3419799804688, 1020.3289794921875, 1019.0040283203125, 1013.5289916992188, 1010.7249755859375, 1010.1959838867188, 1009.8380126953125, 1008.5289916992188, 1015.5919799804688, 1009.3880004882812, 1008.4039916992188, 1014.1829833984375, 1013.5540161132812, 1010.0040283203125, 1009.0330200195312, 1016.0789794921875, 1024.137939453125, 1027.4329833984375, 10242.728515625, 1020.3790283203125, 1019.6129760742188, 1022.3079833984375, 1023.1420288085938, 1019.0040283203125, 1016.6290283203125, 1015.4920043945312, 1014.8790283203125, 1012.18798828125, 1009.2249755859375, 1011.9210205078125, 1015.5330200195312, 1013.7540283203125, 1008.6209716796875, 1007.9420166015625, 1008.6290283203125, 1008.1749877929688, 1004.7899780273438, 1001.9539794921875, 1008.6290283203125, 1008.1749877929688, 1007.7750244140625, 1008.7169799804688, 1016.8079833984375, 1018.0, 1015.8170166015625, 1016.18798828125, 1012.89599609375, 1013.095947265625, 1015.9039916992188, 1012.9710083007812, 1011.125, 1005.9500122070312, 1016.3629760742188, 1015.9749755859375, 1014.97900390625, 1019.4290161132812, 1022.6500244140625, 1013.8579711914062, 1011.9539794921875, 1020.3709716796875, 1021.0999755859375, 1022.4580078125, 1017.3790283203125, 1012.3079833984375, 1008.7579956054688, 1006.8330078125, 1006.0460205078125, 1004.9130249023438, 1010.2579956054688, 1014.9920043945312, 1018.8330078125, 1017.7000122070312, 1014.6420288085938, 1016.3790283203125, 1015.8330078125, 1008.3829956054688, 1012.4249877929688, 1006.3250122070312, 1009.5380249023438, 1014.4580078125, 1013.77099609375, 1017.3460083007812, 1015.7830200195312, 1015.4959716796875, 1015.6079711914062, 1014.4329833984375, 1013.1829833984375, 1018.4959716796875, 1018.3829956054688, 1002.4749755859375, 1008.0750122070312, 1018.0380249023438, 1011.6630249023438, 1002.7249755859375, 1003.8790283203125, 1006.8920288085938, 1011.9459838867188, 1001.0709838867188, 1002.7249755859375, 1003.8790283203125, 1006.8920288085938, 1011.9459838867188, 1011.0709838867188, 1008.7999877929688, 1011.7670288085938, 1008.7999877929688, 1011.7670288085938, 1010.5709838867188, 1012.6209716796875, 1022.5460205078125, 1022.1209716796875, 1013.1669921875, 1014.3289794921875, 1021.2000122070312, 1019.1209716796875, 1019.9375, 1023.4290161132812, 1021.8629760742188, 1029.1209716796875, 1032.3790283203125, 1032.1629638671875, 1028.37109375, 1024.800048828125, 1021.6630249023438, 1020.9959716796875, 1017.9539794921875, 1019.1790161132812, 1004.81298828125, 1012.7039794921875, 1022.7960205078125, 1006.968994140625, 1006.0460205078125, 1012.7039794921875, 1014.7210083007812, 1012.0579833984375, 1014.7210083007812, 1012.0579833984375, 1011.25, 1021.3330078125, 1024.3330078125, 1018.6699829101562, 1023.9580078125, 1029.6500244140625, 1020.4500122070312, 1013.75, 1015.9169921875, 1021.9329833984375, 1023.1959838867188, 1024.4830322265625, 1025.699951171875, 1021.4920043945312, 1015.5419921875, 1014.8209838867188, 1017.2379760742188, 1018.2960205078125, 1019.3829956054688);
float[] l = float[](78.45800018310547, 82.875, 87.91699981689453, 81.29199981689453, 67.08300018310547, 92.5, 94.70800018310547, 89.20800018310547, 82.45800018310547, 95.91699981689453, 86.04199981689453, 87.875, 89.78299713134766, 90.54199981689453, 82.70800018310547, 75.41699981689453, 82.29199981689453, 86.70800018310547, 81.29199981689453, 82.66699981689453, 85.5, 75.83300018310547, 76.29199981689453, 86.70800018310547, 80.45800018310547, 92.20800018310547, 95.125, 92.125, 92.41699981689453, 91.04199981689453, 94.20800018310547, 94.91699981689453, 88.9749984741211, 81.875, 64.625, 71.16699981689453, 65.58300018310547, 63.5, 90.58300018310547, 80.16699981689453, 82.16699981689453, 79.45800018310547, 84.95800018310547, 79.04199981689453, 91.29199981689453, 82.58300018310547, 78.875, 63.45800018310547, 67.625, 83.33000183105469, 79.0, 83.33300018310547, 85.0, 72.91699981689453, 71.66699981689453, 89.54199981689453, 77.625, 86.83300018310547, 80.66699981689453, 83.625, 76.79199981689453, 80.04199981689453, 84.41699981689453, 85.79199981689453, 79.75, 65.66699981689453, 74.0, 80.625, 72.125, 87.20800018310547, 86.25, 79.29199981689453, 79.04199981689453, 64.66699981689453, 56.58300018310547, 61.20800018310547, 72.83300018310547, 80.20800018310547, 67.08300018310547, 79.29199981689453, 51.79199981689453, 48.125, 46.75, 51.5, 53.54199981689453, 40.33300018310547, 44.79199981689453, 49.0, 57.39099884033203, 59.70800018310547, 46.45800018310547, 68.29199981689453, 73.91699981689453, 57.20800018310547, 62.625, 57.5, 53.54199981689453, 41.58300018310547, 56.70800018310547, 60.051998138427734, 61.20800018310547, 52.54199981689453, 50.58300018310547, 68.91699981689453, 79.95800018310547, 76.33300018310547, 72.5, 68.875, 68.875, 66.79199981689453, 74.70800018310547, 75.125, 65.5, 63.375, 64.125, 63.20800018310547, 65.5, 60.83300018310547, 59.16699981689453, 72.41699981689453, 57.41699981689453, 41.20800018310547, 71.25, 85.25, 68.75, 68.125, 56.83300018310547, 61.70800018310547, 59.5, 68.33300018310547, 54.41699981689453, 72.75, 53.625, 58.54199981689453, 66.25, 72.75, 83.70800018310547, 73.91699981689453, 66.29199981689453, 74.625, 70.58300018310547, 71.41699981689453, 64.25, 48.125, 73.04199981689453, 89.95800018310547, 70.45800018310547, 63.625, 70.54199981689453, 68.54199981689453, 56.41699981689453, 40.45800018310547, 46.75, 51.95800018310547, 53.41699981689453, 73.16699981689453, 64.875, 65.83300018310547, 75.625, 89.91699981689453, 97.76200103759766, 93.45800018310547, 87.20800018310547, 82.625, 54.125, 53.5, 88.75, 74.83300018310547, 64.66699981689453, 75.0, 69.79199981689453, 69.45800018310547, 72.54199981689453, 59.625, 61.91699981689453, 65.41699981689453, 63.375, 48.54199981689453, 47.91699981689453, 62.41699981689453, 79.625, 85.58300018310547, 72.20800018310547, 72.45800018310547, 75.75, 86.16699981689453, 70.79199981689453, 70.79199981689453, 70.5, 70.04199981689453, 84.41699981689453, 69.45800018310547, 76.5, 69.45800018310547, 68.5, 54.58300018310547, 63.125, 74.33300018310547, 66.04199981689453, 58.33300018310547, 58.79199981689453, 75.0, 72.83300018310547, 73.70800018310547, 86.70800018310547, 81.95800018310547, 64.08300018310547, 72.41699981689453, 76.375, 74.54199981689453, 79.41699981689453, 86.85299682617188, 74.375, 70.54199981689453, 75.625, 92.125, 76.625, 61.79199981689453, 60.54199981689453, 71.29199981689453, 74.16699981689453, 81.45800018310547, 62.83300018310547, 59.83300018310547, 77.79199981689453, 74.625, 70.375, 64.33300018310547, 70.25, 64.95800018310547, 56.875, 61.875, 68.04199981689453, 79.83300018310547, 88.83300018310547, 74.125, 86.41699981689453, 78.33300018310547, 82.66699981689453, 68.79199981689453, 85.875, 82.25, 88.70800018310547, 75.95800018310547, 84.54199981689453, 81.41699981689453, 75.45800018310547, 82.875, 88.20800018310547, 84.70800018310547, 91.91699981689453, 81.54199981689453, 75.66699981689453, 72.70800018310547, 69.66699981689453, 65.91699981689453, 84.33300018310547, 81.20800018310547, 77.91699981689453, 82.33300018310547, 66.04199981689453, 78.75, 71.79199981689453, 79.625, 69.45800018310547, 72.33300018310547, 72.875, 69.75, 64.83300018310547, 57.58300018310547, 85.66699981689453, 81.66699981689453, 81.5, 71.625, 69.58300018310547, 75.04199981689453, 76.20800018310547, 83.33300018310547, 72.66699981689453, 76.29199981689453, 83.33300018310547, 87.70800018310547, 91.16699981689453, 88.97100067138672, 82.25, 85.95800018310547, 88.41699981689453, 91.54199981689453, 75.75, 62.08300018310547, 81.70800018310547, 73.625, 77.875, 88.5, 90.58300018310547, 84.45800018310547, 85.45800018310547, 94.04199981689453, 89.125, 86.70800018310547, 90.20800018310547, 93.59100341796875, 93.91699981689453, 90.54199981689453, 86.04199981689453, 79.625, 85.91699981689453, 89.45800018310547, 84.5, 84.25, 91.75, 80.0, 80.08300018310547, 81.25, 82.625, 77.54199981689453, 84.5479965209961, 86.125, 87.04199981689453, 86.125, 95.4749984741211, 92.95800018310547, 82.875, 80.29199981689453, 85.5, 88.29199981689453, 81.70800018310547, 85.79199981689453, 92.875, 91.66699981689453, 86.125, 83.875, 82.375, 93.70800018310547, 89.33300018310547, 85.83300018310547, 83.75, 79.91699981689453, 83.75, 80.29199981689453, 46.54800033569336, 61.875, 67.54199981689453, 70.5, 87.04199981689453, 84.5, 79.58300018310547, 89.75, 92.0, 93.25, 91.25, 93.58300018310547);
float[] m = float[](19.16699981689453, 16.45800018310547, 22.91699981689453, 20.625, 25.83300018310547, 17.273000717163086, 19.79199981689453, 27.29199981689453, 17.08300018310547, 13.541999816894531, 18.75, 26.875, 12.083000183105469, 23.75, 30.625, 23.54199981689453, 21.25, 21.04199981689453, 17.29199981689453, 13.541999816894531, 26.25, 18.33300018310547, 12.916999816894531, 14.791999816894531, 16.66699981689453, 23.54199981689453, 24.58300018310547, 13.157999992370605, 15.625, 29.79199981689453, 20.41699981689453, 16.45800018310547, 28.125, 38.54199981689453, 24.58300018310547, 19.58300018310547, 18.33300018310547, 10.833000183105469, 21.54800033569336, 36.45800018310547, 36.66699981689453, 25.625, 22.29199981689453, 16.04199981689453, 27.29199981689453, 35.41699981689453, 30.0, 25.625, 17.5, 21.45800018310547, 13.541999816894531, 18.125, 17.29199981689453, 24.975000381469727, 18.33300018310547, 16.04199981689453, 22.29199981689453, 12.916999816894531, 11.25, 15.208000183105469, 10.208000183105469, 20.0, 18.54199981689453, 27.5, 28.125, 32.91699981689453, 23.54199981689453, 26.25, 17.91699981689453, 25.83300018310547, 20.70800018310547, 14.645999908447266, 21.645999908447266, 18.29199981689453, 7.688000202178955, 14.354000091552734, 17.520999908447266, 12.666999816894531, 10.541999816894531, 11.109000205993652, 29.52199935913086, 10.562999725341797, 14.812999725341797, 20.770999908447266, 25.29199981689453, 34.8129997253418, 16.70800018310547, 13.041999816894531, 12.581999778747559, 24.312999725341797, 21.062999725341797, 20.54199981689453, 21.5, 18.187999725341797, 23.895999908447266, 22.58300018310547, 25.875, 22.812999725341797, 38.979000091552734, 33.5, 27.875, 23.95800018310547, 15.854000091552734, 16.173999786376953, 14.479000091552734, 16.16699981689453, 17.91699981689453, 22.125, 15.458000183105469, 17.312999725341797, 16.875, 22.375, 13.479000091552734, 25.58300018310547, 13.125, 8.312999725341797, 12.645999908447266, 14.312999725341797, 24.729000091552734, 25.20800018310547, 14.916999816894531, 12.229000091552734, 12.25, 22.95800018310547, 19.645999908447266, 25.04199981689453, 17.020999908447266, 18.83300018310547, 24.625, 21.895999908447266, 18.604000091552734, 11.375, 27.604000091552734, 31.95800018310547, 18.60300064086914, 12.104000091552734, 10.208000183105469, 11.395999908447266, 16.145999908447266, 13.208000183105469, 21.812999725341797, 27.41699981689453, 23.33300018310547, 28.79199981689453, 21.979000091552734, 20.854000091552734, 15.979000091552734, 14.0, 18.062999725341797, 23.312999725341797, 10.937999725341797, 10.125, 12.520999908447266, 12.875, 17.312999725341797, 20.08300018310547, 24.562999725341797, 24.29199981689453, 14.916999816894531, 9.833000183105469, 12.562999725341797, 11.895999908447266, 12.895999908447266, 15.979000091552734, 17.437999725341797, 15.166999816894531, 12.312999725341797, 14.833000183105469, 12.166999816894531, 12.0, 13.416999816894531, 12.375, 10.345000267028809, 23.562999725341797, 26.520999908447266, 16.875, 13.166999816894531, 17.04199981689453, 22.645999908447266, 24.062999725341797, 19.729000091552734, 17.145999908447266, 14.041999816894531, 11.104000091552734, 24.20800018310547, 11.062999725341797, 11.291999816894531, 9.562999725341797, 12.229000091552734, 13.062999725341797, 14.020999908447266, 10.958000183105469, 9.395999908447266, 12.687999725341797, 16.16699981689453, 22.125, 15.375, 16.08300018310547, 16.937999725341797, 13.708000183105469, 13.645999908447266, 9.020999908447266, 18.770999908447266, 22.83300018310547, 18.54199981689453, 19.5, 9.729000091552734, 12.062999725341797, 9.083000183105469, 12.416999816894531, 13.458000183105469, 11.437999725341797, 7.813000202178955, 8.46399974822998, 18.29199981689453, 14.541999816894531, 10.187999725341797, 9.666999816894531, 9.020999908447266, 10.916999816894531, 15.208000183105469, 11.75, 15.041999816894531, 12.520999908447266, 13.125, 26.29199981689453, 14.479000091552734, 19.270999908447266, 13.208000183105469, 16.08300018310547, 13.604000091552734, 11.104000091552734, 14.541999816894531, 16.270999908447266, 10.75, 13.916999816894531, 15.395999908447266, 14.041999816894531, 19.770999908447266, 18.520999908447266, 12.375, 13.625, 20.125, 20.45800018310547, 27.812999725341797, 14.770999908447266, 22.520999908447266, 22.312999725341797, 18.33300018310547, 11.770999908447266, 18.562999725341797, 41.520999908447266, 29.062999725341797, 17.5, 11.0, 14.625, 16.04199981689453, 16.04199981689453, 17.520999908447266, 18.83300018310547, 24.70400047302246, 25.729000091552734, 14.583000183105469, 10.083000183105469, 12.812999725341797, 17.604000091552734, 20.395999908447266, 24.25, 29.54199981689453, 18.29199981689453, 24.79199981689453, 18.41699981689453, 13.875, 19.0, 16.062999725341797, 9.541999816894531, 9.375, 9.270999908447266, 23.895999908447266, 16.75, 11.687999725341797, 9.75, 20.875, 25.020999908447266, 16.437999725341797, 18.0, 24.687999725341797, 21.520999908447266, 13.520999908447266, 14.125, 22.270999908447266, 23.91699981689453, 18.375, 12.86400032043457, 9.5, 6.666999816894531, 14.041999816894531, 27.895999908447266, 20.83300018310547, 17.25, 21.229000091552734, 14.458000183105469, 16.29199981689453, 17.729000091552734, 11.479000091552734, 7.291999816894531, 9.854000091552734, 18.95800018310547, 21.812999725341797, 18.54199981689453, 21.812999725341797, 18.54199981689453, 20.979000091552734, 31.875, 24.216999053955078, 26.479000091552734, 36.125, 21.229000091552734, 20.562999725341797, 23.29199981689453, 20.270999908447266, 19.020999908447266, 14.937999725341797, 20.812999725341797, 17.625, 18.66699981689453, 14.312999725341797, 13.312999725341797, 18.25, 17.437999725341797, 23.45800018310547, 39.83300018310547, 35.270999908447266, 35.95800018310547, 441.64599609375, 28.437999725341797, 26.70800018310547, 18.437999725341797, 19.83300018310547, 24.229000091552734, 16.187999725341797, 9.354000091552734, 20.979000091552734, 21.229000091552734, 13.5, 20.437999725341797, 16.312999725341797, 24.604000091552734, 28.83300018310547, 23.354000091552734, 15.562999725341797, 15.187999725341797, 17.604000091552734, 9.25, 24.83300018310547);
float[] S = float[](3.1500000953674316, 4.5289998054504395, 3.5789999961853027, 3.2920000553131104, 1.5880000591278076, 1.9140000343322754, 3.4519999027252197, 3.638000011444092, 4.104000091552734, 3.117000102996826, 1.996000051498413, 4.617000102996826, 3.8540000915527344, 2.7829999923706055, 6.145999908447266, 6.888000011444092, 4.321000099182129, 2.013000011444092, 4.392000198364258, 3.433000087738037, 6.316999912261963, 4.321000099182129, 3.325000047683716, 6.313000202178955, 4.175000190734863, 3.257999897003174, 3.7960000038146973, 0.9959999918937683, 2.799999952316284, 3.0829999446868896, 2.73799991607666, 4.968999862670898, 5.770999908447266, 2.7170000076293945, 0.15000000596046448, -0.3630000054836273, 1.1330000162124634, -0.08299999684095383, 2.7829999923706055, 2.7790000438690186, 4.474999904632568, 3.996000051498413, 3.187999963760376, 3.253999948501587, 1.496000051498413, 1.600000023841858, 4.313000202178955, 8.704000473022461, 6.775000095367432, 4.7129998207092285, 4.732999801635742, 2.638000011444092, 4.071000099182129, 4.87890625, 3.8420000076293945, 0.34200000762939453, 3.5209999084472656, 1.4539999961853027, 0.5379999876022339, 0.9629999995231628, 5.013000011444092, 5.632999897003174, 5.8420000076293945, 5.875, 4.571000099182129, 4.566999912261963, 4.4079999923706055, 4.199999809265137, 4.775000095367432, 4.479000091552734, 5.849999904632568, 4.413000106811523, 4.525000095367432, 0.1080000028014183, 1.062999963760376, 3.5290000438690186, 4.7170000076293945, 7.11299991607666, 6.313000202178955, 3.9079999923706055, 0.8960000276565552, 0.22100000083446503, 0.6579999923706055, 2.312999963760376, 4.632999897003174, 6.25, 7.321000099182129, 7.052000045776367, 2.7780001163482666, 0.8920000195503235, 1.7130000591278076, 4.541999816894531, 5.928999900817871, 9.875, 12.970999717712402, 10.345999717712402, 7.642000198364258, 6.2170000076293945, 6.446000099182129, 7.833000183105469, 6.696000099182129, 2.0920000076293945, 5.138000011444092, 6.620999813079834, 4.913000106811523, 5.563000202178955, 6.520999908447266, 7.913000106811523, 9.633000373840332, 9.779000282287598, 8.104000091552734, 6.117000102996826, 5.86299991607666, 6.992000102996826, 8.520999908447266, 5.696000099182129, 7.313000202178955, 11.567000389099121, 10.925000190734863, 9.713000297546387, 11.262999534606934, 10.925000190734863, 9.413000106811523, 11.262999534606934, 11.866999626159668, 11.399999618530273, 9.170999526977539, 10.067000389099121, 10.645999908447266, 14.116999626159668, 9.267000198364258, 6.691999912261963, 6.857999801635742, 6.666999816894531, 7.232999801635742, 7.921000003814697, 8.199999809265137, 8.404000282287598, 7.8420000076293945, 8.366999626159668, 8.842000007629395, 8.366999626159668, 8.842000007629395, 10.838000297546387, 13.213000297546387, 11.682999610900879, 9.621000289916992, 13.232999801635742, 14.458000183105469, 13.128999710083008, 12.338000297546387, 14.854000091552734, 16.613000869750977, 14.291999816894531, 13.61299991607666, 14.225000381469727, 14.600000381469727, 17.242000579833984, 17.100000381469727, 16.950000762939453, 18.600000381469727, 14.37600040435791, 17.304000854492188, 19.562999725341797, 19.729000091552734, 19.267000198364258, 17.0, 16.16699981689453, 18.570999145507812, 20.799999237060547, 22.51300048828125, 23.850000381469727, 23.92099952697754, 21.149999618530273, 21.70800018310547, 21.642000198364258, 22.966999053955078, 25.42099952697754, 25.933000564575195, 23.850000381469727, 22.67099952697754, 17.27899932861328, 17.367000579833984, 17.941999435424805, 17.683000564575195, 17.854000091552734, 18.304000854492188, 18.57900047302246, 16.253999710083008, 14.779000282287598, 14.5, 14.517000198364258, 16.863000869750977, 15.088000297546387, 15.475000381469727, 16.54599952697754, 18.429000854492188, 18.520999908447266, 19.54599952697754, 21.22100067138672, 22.22100067138672, 22.82900047302246, 18.875, 15.342000007629395, 13.996000289916992, 13.741999626159668, 16.312999725341797, 18.9689998626709, 19.566999435424805, 20.95400047302246, 19.038000106811523, 15.800000190734863, 16.52899932861328, 17.899999618530273, 17.96299934387207, 18.253999710083008, 17.71299934387207, 18.524999618530273, 19.966999053955078, 21.216999053955078, 21.191999435424805, 22.09600067138672, 20.475000381469727, 18.179000854492188, 16.566999435424805, 15.442000389099121, 18.27899932861328, 20.371000289916992, 21.933000564575195, 19.788999557495117, 19.07900047302246, 20.812999725341797, 20.79599952697754, 21.645999908447266, 19.632999420166016, 19.257999420166016, 18.070999145507812, 15.012999534606934, 15.199999809265137, 15.829000473022461, 14.991999626159668, 13.604000091552734, 17.29599952697754, 15.241999626159668, 15.045999526977539, 15.949999809265137, 17.746000289916992, 17.149999618530273, 16.57900047302246, 14.61299991607666, 13.324999809265137, 12.538000106811523, 15.729000091552734, 14.758000373840332, 12.696000099182129, 15.303999900817871, 15.961999893188477, 16.757999420166016, 18.183000564575195, 19.687999725341797, 12.288000106811523, 12.996000289916992, 12.692000389099121, 12.399999618530273, 14.262999534606934, 15.475000381469727, 17.545000076293945, 18.128999710083008, 18.642000198364258, 19.47100067138672, 19.232999801635742, 14.517000198364258, 13.967000007629395, 13.182999610900879, 14.196000099182129, 17.158000946044922, 16.788000106811523, 16.003999710083008, 14.375, 13.399999618530273, 11.366999626159668, 9.963000297546387, 12.128999710083008, 10.387999534606934, 9.149999618530273, 8.017000198364258, 7.416999816894531, 6.074999809265137, 5.4670000076293945, 4.98799991607666, 5.104000091552734, 6.438000202178955, 7.98799991607666, 11.550000190734863, 12.074999809265137, 10.300000190734863, 9.288000106811523, 10.628999710083008, 13.071000099182129, 9.053999900817871, 10.03600025177002, 8.920999526977539, 8.571000099182129, 8.699999809265137, 8.324999809265137, 11.095999717712402, 8.338000297546387, 8.532999992370605, 10.795999526977539, 10.008000373840332, 8.770999908447266, 6.117000102996826, 7.828999996185303, 5.882999897003174, 5.583000183105469, 4.520999908447266, 3.937999963760376, 4.321000099182129, 5.053999900817871, 6.478000164031982, 10.437999725341797, 9.807999610900879, 2.992000102996826, 2.1080000400543213, 5.5920000076293945, 8.746000289916992, 5.242000102996826, 3.321000099182129, 2.744999885559082, 1.7130000591278076, 0.6169999837875366, 1.6959999799728394, -1.746000051498413, 0.125, 2.0829999446868896, 4.5329999923706055, 3.450000047683716, 0.6460000276565552, -2.75, -3.638000011444092, -2.808000087738037, -0.7519999742507935, 0.0, -1.8919999599456787, -0.6790000200271606, 0.257999986410141, 1.7710000276565552, 4.958000183105469, 4.131999969482422, 5.6579999923706055, 2.187999963760376);

void main() {

  float timeSec = (timeMsec/1000.0)*0.5;
  float fracttime = fract(timeSec);
  int time = int(timeSec);

  vec3 color = vec3(0.0);
  float a = 1.0;
  
  float kinterp = mix(k[time%348], k[(time+1)%348], fracttime);
  float linterp = mix(l[time%348], l[(time+1)%348], fracttime);
  float minterp = map(mix(m[time%348], m[(time+1)%348], fracttime),2.44,10.0,0.0,20.0);
  float Sinterp = map(mix(S[time%348], S[(time+1)%348], fracttime),966.783,1024.6,0.0,5.0);
  
  float lines = abs(chladni(p, kinterp , linterp ,minterp,Sinterp,2.0));
  
  if (lines > 0.3)
  {
  discard; 
  }
  
  color = vec3(dot(p, p));

  gl_FragColor = vec4( color*pow(1.0-lines,1.5) , 1.0 );

}

`
});
