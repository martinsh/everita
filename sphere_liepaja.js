AFRAME.registerShader('sphere_liepaja', {
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

float[] k = float[]{4.921, 5.3, 5.3, 4.446, 2.925, 5.046, 5.25, 5.592, 5.113, 5.088, 3.354, 5.508, 5.258, 4.725, 6.729, 6.258, 4.579, 4.533, 5.333, 4.746, 6.038, 5.167, 4.392, 5.808, 5.008, 5.1, 4.133, 2.113, 3.5, 4.4, 4.233, 5.554, 5.458, 3.675, 0.388, 1.471, 2.767, 0.608, 3.821, 4.775, 5.754, 4.196, 4.642, 4.65, 2.408, 2.213, 5.592, 7.242, 5.746, 4.929, 4.438, 4.329, 5.121, 5.15, 4.192, 1.704, 3.933, 3.021, 1.983, 3.354, 5.042, 5.283, 5.363, 5.417, 4.408, 4.133, 4.171, 3.546, 4.867, 5.342, 6.117, 5.742, 4.746, -0.029, 2.883, 4.629, 2.663, 5.867, 5.408, 4.633, 0.854, 0.175, -0.942, 2.642, 4.05, 2.996, 6.408, 5.561, 2.7, -0.358, 0.604, 5.033, 5.688, 5.583, 4.892, 5.067, 7.4, 8.213, 6.217, 6.913, 3.471, 3.854, 7.296, 6.188, 4.283, 6.454, 7.008, 5.688, 5.717, 5.871, 6.196, 7.442, 7.95, 7.046, 6.517, 6.321, 7.042, 7.571, 7.738, 5.642, 7.621, 10.288, 8.863, 7.133, 8.613, 10.846, 9.588, 8.142, 6.842, 8.679, 9.529, 7.833, 7.283, 6.883, 7.775, 8.517, 8.95, 8.504, 8.783, 9.267, 8.317, 8.654, 10.192, 10.925, 10.004, 10.808, 12.867, 10.996, 12.558, 11.638, 12.917, 15.792, 14.475, 14.017, 14.313, 14.479, 15.613, 12.533, 13.671, 13.579, 14.004, 17.138, 19.188, 19.188, 19.621, 19.633, 17.308, 14.417, 15.533, 16.108, 21.971, 21.975, 21.75, 21.238, 19.579, 17.271, 20.108, 22.075, 23.213, 22.363, 19.646, 19.158, 16.804, 16.675, 16.488, 16.454, 16.567, 16.888, 17.038, 15.446, 14.333, 15.054, 14.921, 15.408, 15.413, 15.517, 15.375, 15.333, 19.95, 19.625, 19.821, 20.854, 21.133, 17.896, 15.971, 16.392, 15.792, 17.354, 18.204, 18.313, 17.921, 17.746, 16.413, 17.038, 18.254, 15.854, 17.513, 16.929, 17.258, 17.746, 20.296, 22.45, 22.65, 20.363, 18.254, 17.038, 16.354, 16.917, 18.654, 17.883, 19.817, 19.896, 19.883, 20.454, 19.688, 20.433, 20.183, 18.725, 16.163, 15.129, 15.283, 14.583, 14.45, 17.754, 15.367, 14.508, 15.079, 16.113, 17.017, 16.596, 16.192, 14.404, 16.192, 16.617, 15.354, 13.32, 16.917, 15.929, 16.804, 16.833, 17.3, 12.45, 13.854, 14.788, 11.217, 15.392, 15.954, 16.238, 17.463, 17.654, 19.842, 19.158, 14.025, 11.517, 12.413, 13.75, 13.075, 13.183, 13.183, 17.592, 15.588, 14.788, 13.104, 12.221, 12.704, 10.721, 11.621, 8.35, 8.225, 5.742, 8.575, 5.963, 4.833, 8.392, 8.658, 8.554, 9.763, 13.004, 12.683, 11.492, 11.817, 11.188, 12.117, 11.05, 10.158, 8.554, 6.842, 8.638, 9.1, 12.525, 11.188, 10.629, 11.417, 10.658, 9.279, 5.708, 7.383, 3.983, 4.917, 4.871, 5.271, 6.9, 8.938, 9.45, 11.117, 9.946, 5.633, 6.408, 9.138, 8.6, 8.221, 9.254, 8.292, 5.279, 2.738, 1.596, 1.675, 1.1, -2.163, -1.221, 1.95, 5.746, 4.471, 1.013, -2.118, -3.46, -3.129, -0.671, 1.025, -1.45, 6.042};
float[] l = float[]{26.042, 29.583, 38.75, 27.917, 23.958, 25.313, 22.918, 40.625, 22.917, 14.375, 22.5, 43.75, 17.292, 28.75, 39.583, 32.708, 22.292, 32.083, 23.542, 27.5, 33.125, 26.458, 25.417, 35.208, 18.125, 29.375, 18.125, 20.833, 19.583, 20.0, 11.875, 24.375, 33.333, 17.5, 14.375, 21.042, 21.667, 13.333, 22.708, 31.957, 49.167, 32.917, 34.583, 32.708, 34.583, 32.708, 10.625, 25.417, 42.5, 47.917, 40.0, 33.75, 17.083, 28.958, 49.583, 45.87, 20.625, 18.958, 38.542, 21.667, 10.625, 20.208, 26.25, 15.417, 11.042, 27.708, 20.833, 11.458, 13.33, 12.292, 17.917, 22.292, 39.975, 46.042, 37.708, 26.458, 30.833, 29.375, 17.5, 32.083, 20.979, 17.0, 22.479, 16.333, 8.458, 21.042, 23.021, 11.792, 9.896, 10.065, 27.696, 15.125, 20.063, 30.229, 38.875, 41.875, 24.708, 15.875, 14.375, 22.958, 15.896, 16.813, 9.771, 11.813, 32.854, 25.188, 25.271, 20.146, 17.458, 21.229, 12.604, 15.229, 12.167, 13.271, 15.063, 17.854, 13.354, 15.063, 17.584, 13.354, 7.833, 13.333, 18.0, 12.521, 17.5, 8.208, 14.25, 16.875, 11.438, 16.333, 13.479, 18.604, 9.979, 12.604, 19.458, 21.0, 32.958, 20.979, 28.708, 39.604, 34.146, 20.521, 16.771, 16.542, 23.667, 17.813, 16.646, 12.458, 12.604, 14.125, 13.604, 17.271, 20.208, 18.708, 25.458, 15.958, 16.125, 13.521, 11.886, 14.563, 23.25, 10.271, 14.208, 13.438, 14.0, 15.917, 21.208, 23.5, 19.75, 11.345, 12.146, 11.563, 16.063, 12.021, 10.521, 11.042, 13.167, 9.75, 10.042, 9.708, 8.625, 8.146, 21.792, 30.188, 18.979, 12.354, 22.042, 29.854, 25.646, 19.521, 9.271, 32.354, 21.271, 15.042, 12.563, 7.792, 11.354, 10.083, 9.313, 8.25, 12.125, 22.146, 22.958, 21.25, 24.479, 16.792, 10.438, 10.208, 11.354, 23.854, 38.792, 26.333, 14.833, 10.896, 9.104, 9.125, 14.167, 8.271, 8.25, 8.583, 15.104, 15.125, 12.0, 10.875, 7.271, 6.708, 8.125, 8.125, 11.021, 11.396, 7.771, 8.438, 14.896, 21.396, 24.771, 11.979, 11.292, 13.458, 6.333, 15.104, 17.542, 11.25, 11.646, 17.542, 14.042, 22.521, 19.375, 15.833, 29.271, 28.438, 24.042, 16.25, 28.792, 32.521, 26.292, 11.542, 15.167, 29.333, 20.479, 9.688, 10.313, 19.563, 17.792, 13.771, 11.521, 9.771, 17.938, 20.438, 12.333, 7.542, 11.979, 14.25, 15.625, 22.313, 23.063, 14.438, 26.438, 13.75, 15.688, 20.021, 12.292, 12.333, 10.458, 9.5, 27.417, 18.75, 8.417, 6.396, 24.167, 27.167, 13.083, 15.813, 33.229, 28.0, 17.875, 15.25, 16.104, 15.396, 23.646, 12.523, 9.188, 75.0, 16.896, 24.104, 29.104, 28.583, 36.958, 14.708, 21.646, 13.688, 8.104, 4.833, 9.646, 12.771, 16.0, 13.125, 15.792, 19.563, 22.043, 36.0, 42.548, 28.083, 33.604, 39.5, 32.604, 27.375, 22.604, 25.813, 17.792, 17.208, 11.292, 8.979, 15.542, 16.729, 19.958, 23.542, 22.375, 25.771, 26.083, 25.646, 18.104, 18.146, 15.75, 17.854, 12.667, 11.708, 18.0, 18.313, 17.292, 22.729, 21.833};
float[] m = float[]{87.917, 89.625, 73.458, 64.542, 94.542, 92.5, 93.208, 87.125, 99.083, 86.458, 92.25, 92.833, 79.625, 87.75, 86.917, 85.375, 85.583, 88.375, 89.0, 91.083, 72.542, 79.583, 93.667, 85.917, 89.333, 91.833, 94.583, 91.208, 91.375, 93.25, 97.25, 88.5, 76.625, 83.833, 74.75, 74.958, 67.833, 88.417, 85.75, 87.208, 84.333, 84.542, 77.333, 92.417, 93.708, 84.542, 92.417, 93.708, 84.875, 76.708, 83.958, 88.833, 86.625, 86.042, 89.292, 78.375, 75.958, 93.417, 82.375, 88.208, 84.292, 76.125, 86.875, 88.583, 95.833, 92.667, 90.125, 75.333, 84.208, 86.125, 73.958, 89.958, 85.25, 83.917, 84.083, 64.917, 58.292, 79.042, 91.708, 91.5, 82.375, 82.875, 64.042, 60.417, 62.917, 65.5, 71.125, 66.0, 47.625, 56.13, 56.913, 72.75, 68.292, 84.25, 83.875, 73.75, 74.667, 87.792, 61.417, 64.542, 83.125, 75.708, 85.75, 86.833, 70.833, 72.958, 64.458, 80.667, 76.0, 68.125, 68.333, 67.875, 80.042, 76.5, 72.458, 83.375, 84.667, 69.583, 75.25, 61.375, 77.917, 67.75, 61.0, 66.75, 94.417, 96.583, 86.583, 75.167, 71.167, 75.292, 76.75, 83.417, 72.708, 76.167, 61.042, 68.0, 65.958, 80.583, 83.25, 79.208, 80.042, 85.25, 73.917, 64.833, 93.458, 84.417, 70.917, 85.083, 74.5, 70.083, 69.042, 50.417, 63.33, 57.458, 57.583, 68.75, 68.875, 83.333, 80.0, 92.042, 78.833, 83.833, 86.083, 85.333, 63.375, 56.25, 87.958, 81.625, 77.958, 70.25, 78.167, 80.792, 84.25, 83.75, 88.542, 64.458, 60.583, 60.042, 73.0, 89.125, 91.792, 83.208, 82.875, 88.083, 76.25, 85.125, 96.583, 77.083, 76.958, 74.542, 74.875, 86.375, 80.375, 77.042, 81.333, 85.0, 78.375, 63.833, 61.625, 68.708, 69.042, 73.708, 77.0, 74.75, 69.667, 81.792, 82.833, 77.042, 83.125, 83.958, 76.125, 78.625, 84.375, 78.792, 83.667, 76.125, 78.625, 84.375, 78.792, 83.667, 89.125, 90.25, 76.042, 81.792, 68.833, 65.708, 71.792, 76.542, 70.292, 78.417, 81.625, 75.0, 85.333, 77.333, 62.208, 67.917, 80.167, 88.625, 86.708, 82.0, 71.375, 77.083, 89.333, 85.333, 75.583, 90.458, 79.083, 82.583, 74.958, 85.542, 88.0, 86.25, 85.25, 71.708, 78.458, 80.833, 82.583, 71.458, 72.708, 72.542, 74.583, 89.625, 91.792, 89.0, 71.0, 64.75, 78.125, 86.167, 81.917, 82.0, 81.5, 81.958, 86.0, 70.917, 65.958, 85.167, 85.375, 85.167, 79.292, 79.208, 89.417, 78.958, 84.548, 85.625, 86.292, 90.25, 84.833, 88.917, 74.25, 88.5, 86.583, 83.208, 87.042, 86.792, 79.125, 70.125, 54.5, 64.333, 90.958, 89.542, 84.917, 85.667, 87.875, 89.917, 90.583, 82.625, 92.455, 92.875, 95.375, 80.792, 93.458, 86.333, 76.708, 78.875, 91.667, 95.75, 87.792, 92.0, 85.375, 90.25, 86.833, 91.458, 90.708, 93.417, 88.917, 92.739, 94.75, 80.958, 59.125, 69.542, 79.417, 71.75, 81.583, 90.625, 87.333, 83.208, 86.042, 88.548, 91.542, 88.875, 86.458, 84.375, 82.167, 82.917, 85.75, 59.042, 69.625, 80.667, 82.625, 85.542, 92.25};
float[] S = float[]{1019.563, 1016.183, 1008.575, 1001.025, 1020.625, 1019.533, 1019.729, 1019.729, 1013.917, 1014.013, 1007.975, 1019.917, 1011.304, 1011.971, 1011.442, 1008.308, 1018.013, 1024.167, 1015.838, 1025.129, 1033.629, 1022.496, 1016.413, 1021.85, 1010.158, 1010.438, 1013.025, 1010.213, 998.392, 991.458, 997.475, 993.617, 993.738, 988.542, 997.3, 1001.233, 1017.2, 1015.304, 1027.154, 1022.238, 1009.183, 977.85, 980.913, 988.717, 1001.45, 1014.542, 1019.738, 1005.283, 999.175, 1004.592, 1005.138, 1013.138, 1008.288, 1003.171, 1003.17, 993.883, 1002.4, 994.246, 984.767, 988.929, 1000.813, 1001.979, 993.108, 996.967, 1000.183, 1001.663, 1004.8, 1004.867, 1007.092, 1014.663, 1011.646, 1004.967, 991.554, 990.792, 997.046, 1004.867, 1007.092, 1014.663, 1011.646, 1004.967, 991.554, 990.792, 997.046, 1015.288, 1022.429, 1014.988, 1022.358, 1017.4, 1018.529, 1022.108, 1029.108, 1038.583, 1036.179, 1034.692, 1029.596, 1019.604, 1019.561, 1020.625, 1022.213, 1008.067, 998.421, 995.717, 1016.896, 1031.083, 1031.238, 1025.342, 1028.258, 1015.704, 1022.167, 1025.9, 1013.763, 1001.142, 1007.9, 1006.483, 1003.704, 1012.875, 1017.75, 1020.683, 1026.679, 1028.829, 1019.708, 1005.788, 1004.338, 1006.892, 1009.504, 1002.829, 1007.358, 1010.221, 1004.888, 1003.075, 1008.817, 1013.825, 1012.438, 1012.829, 1014.188, 1014.4, 1012.829, 1010.596, 1001.783, 1006.492, 1007.729, 1012.629, 1010.967, 1008.146, 1009.125, 1012.217, 1012.246, 1019.496, 1022.608, 1023.608, 1021.013, 1016.6, 1022.0, 1030.929, 1028.346, 1025.492, 1026.421, 1022.196, 1021.975, 1022.65, 1016.596, 1009.021, 1002.608, 996.592, 999.725, 1009.154, 1009.742, 1015.496, 1013.054, 1014.713, 1018.521, 1018.867, 1013.688, 1013.288, 1014.096, 1010.867, 1010.996, 1011.904, 1014.646, 1015.196, 1018.721, 1024.771, 1026.425, 1022.417, 1013.267, 1009.663, 1008.338, 1002.063, 1002.029, 1005.263, 1010.904, 1011.313, 1005.388, 1002.988, 1007.767, 1009.054, 1012.429, 1008.775, 1011.383, 1018.842, 1018.346, 1016.567, 1013.763, 1011.654, 1016.65, 1020.304, 1018.529, 1012.9, 1012.483, 1012.413, 1011.183, 1007.938, 1008.304, 1013.904, 1015.938, 1011.246, 1009.663, 1010.308, 1010.125, 1015.483, 1013.583, 1009.624, 1009.358, 1009.358, 1017.154, 1024.254, 1026.983, 1024.0, 1020.188, 1020.482, 1023.179, 1023.25, 1019.013, 1016.496, 1016.817, 1016.654, 1014.45, 1011.25, 1008.838, 1010.804, 1014.396, 1011.179, 1008.588, 1008.225, 1008.225, 1008.225, 1008.246, 1003.063, 1002.458, 1009.129, 1005.246, 1007.946, 1009.971, 1017.071, 1016.867, 1014.733, 1014.95, 1012.183, 1011.804, 1015.617, 1013.962, 1012.221, 1009.104, 1016.617, 1014.771, 1016.854, 1021.254, 1023.313, 1013.913, 1018.221, 1023.863, 1022.538, 1023.067, 1017.163, 1012.767, 1007.063, 1004.533, 1003.3, 1000.2, 1006.533, 1013.996, 1017.993, 1017.229, 1012.421, 1012.979, 1011.154, 1002.529, 1007.067, 1013.133, 1012.038, 1015.288, 1015.775, 1015.025, 1015.421, 1013.171, 1013.738, 1019.042, 1018.154, 1004.046, 1011.104, 1018.629, 1009.263, 1002.808, 1005.204, 1007.208, 1010.433, 1007.642, 1006.825, 1009.5, 1008.925, 1011.533, 1022.521, 1019.167, 1009.533, 1014.092, 1020.863, 1021.125, 1022.55, 1025.075, 1023.504, 1029.742, 1032.621, 1031.175, 1025.921, 1021.621, 1019.404, 1017.292, 1013.45, 1016.846, 1016.367, 1004.658, 1015.283, 1022.408, 1007.658, 1008.788, 1014.85, 1016.088, 1012.038, 1013.796, 1022.488, 1024.367, 1019.313, 1022.121, 1026.867, 1016.786, 1008.154, 1010.258, 1016.529, 1016.654, 1019.213, 1022.054};

void main() {

  float timeSec = (timeMsec/1000.0)*1.0;
  float fracttime = fract(timeSec);
  int time = int(timeSec);

  vec3 color = vec3(0.0);
  float a = 1.0;
  
  float kinterp = mix(k[time%348], k[(time+1)%348], fracttime);
  float linterp = mix(l[time%348], l[(time+1)%348], fracttime);
  float minterp = map(mix(m[time%348]*0.1, m[(time+1)%348]*0.1, fracttime),2.44,10.0,0.0,20.0);
  float Sinterp = map(mix(S[time%348], S[(time+1)%348], fracttime),966.783,1024.6,0.0,5.0);
  
  float lines = abs(chladni(p, kinterp , linterp , minterp, Sinterp, 3.0));
  
  if (lines > 0.3)
  {
  discard; 
  }
  
  color = vec3(dot(p, p));

  gl_FragColor = vec4( color*pow(1.0-lines,1.5) , 1.0 );

}

`
});

