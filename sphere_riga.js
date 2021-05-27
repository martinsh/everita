

function mix(x, y, a) {
    return x*(1.0-a)+y*a;
}

AFRAME.registerShader('sphere_riga', {
  schema: {
    timeMsec: {type:'time'},
    k: {type:'float'},
    l: {type:'float'},
    m: {type:'float'},
    S: {type:'float'}
  },
  
  uniforms: {
      // the texture value (once the texture source is loaded, update)
      //map: { type: 't', value: null },
      // texture parameters
      timeMsec: {type:'time', is:'uniform'},
      k: {type: 'float', is:'uniform'},
      l: {type: 'float', is:'uniform'},
      m: {type: 'float', is:'uniform'},
      S: {type: 'float', is:'uniform'}
  },

  init: function(data) {
      
      this.material = new THREE.ShaderMaterial({

          uniforms: this.uniforms,
          vertexShader: this.vertexShader,
          fragmentShader: this.fragmentShader
        
      });
    
    this.uniforms.k.value = 0.0;
    this.uniforms.l.value = 0.0;
    this.uniforms.m.value = 0.0;
    this.uniforms.S.value = 0.0;
    
    this.mData = [78.458, 82.875, 87.917, 81.292, 67.083, 92.5, 94.708, 89.208, 82.458, 95.917, 86.042, 87.875, 89.783, 90.542, 82.708, 75.417, 82.292, 86.708, 81.292, 82.667, 85.5, 75.833, 76.292, 86.708, 80.458, 92.208, 95.125, 92.125, 92.417, 91.042, 94.208, 94.917, 88.975, 81.875, 64.625, 71.167, 65.583, 63.5, 90.583, 80.167, 82.167, 79.458, 84.958, 79.042, 91.292, 82.583, 78.875, 63.458, 67.625, 83.33, 79.0, 83.333, 85.0, 72.917, 71.667, 89.542, 77.625, 86.833, 80.667, 83.625, 76.792, 80.042, 84.417, 85.792, 79.75, 65.667, 74.0, 80.625, 72.125, 87.208, 86.25, 79.292, 79.042, 64.667, 56.583, 61.208, 72.833, 80.208, 67.083, 79.292, 51.792, 48.125, 46.75, 51.5, 53.542, 40.333, 44.792, 49.0, 57.391, 59.708, 46.458, 68.292, 73.917, 57.208, 62.625, 57.5, 53.542, 41.583, 56.708, 60.052, 61.208, 52.542, 50.583, 68.917, 79.958, 76.333, 72.5, 68.875, 68.875, 66.792, 74.708, 75.125, 65.5, 63.375, 64.125, 63.208, 65.5, 60.833, 59.167, 72.417, 57.417, 41.208, 71.25, 85.25, 68.75, 68.125, 56.833, 61.708, 59.5, 68.333, 54.417, 72.75, 53.625, 58.542, 66.25, 72.75, 83.708, 73.917, 66.292, 74.625, 70.583, 71.417, 64.25, 48.125, 73.042, 89.958, 70.458, 63.625, 70.542, 68.542, 56.417, 40.458, 46.75, 51.958, 53.417, 73.167, 64.875, 65.833, 75.625, 89.917, 97.762, 93.458, 87.208, 82.625, 54.125, 53.5, 88.75, 74.833, 64.667, 75.0, 69.792, 69.458, 72.542, 59.625, 61.917, 65.417, 63.375, 48.542, 47.917, 62.417, 79.625, 85.583, 72.208, 72.458, 75.75, 86.167, 70.792, 70.792, 70.5, 70.042, 84.417, 69.458, 76.5, 69.458, 68.5, 54.583, 63.125, 74.333, 66.042, 58.333, 58.792, 75.0, 72.833, 73.708, 86.708, 81.958, 64.083, 72.417, 76.375, 74.542, 79.417, 86.853, 74.375, 70.542, 75.625, 92.125, 76.625, 61.792, 60.542, 71.292, 74.167, 81.458, 62.833, 59.833, 77.792, 74.625, 70.375, 64.333, 70.25, 64.958, 56.875, 61.875, 68.042, 79.833, 88.833, 74.125, 86.417, 78.333, 82.667, 68.792, 85.875, 82.25, 88.708, 75.958, 84.542, 81.417, 75.458, 82.875, 88.208, 84.708, 91.917, 81.542, 75.667, 72.708, 69.667, 65.917, 84.333, 81.208, 77.917, 82.333, 66.042, 78.75, 71.792, 79.625, 69.458, 72.333, 72.875, 69.75, 64.833, 57.583, 85.667, 81.667, 81.5, 71.625, 69.583, 75.042, 76.208, 83.333, 72.667, 76.292, 83.333, 87.708, 91.167, 88.971, 82.25, 85.958, 88.417, 91.542, 75.75, 62.083, 81.708, 73.625, 77.875, 88.5, 90.583, 84.458, 85.458, 94.042, 89.125, 86.708, 90.208, 93.591, 93.917, 90.542, 86.042, 79.625, 85.917, 89.458, 84.5, 84.25, 91.75, 80.0, 80.083, 81.25, 82.625, 77.542, 84.548, 86.125, 87.042, 86.125, 95.475, 92.958, 82.875, 80.292, 85.5, 88.292, 81.708, 85.792, 92.875, 91.667, 86.125, 83.875, 82.375, 93.708, 89.333, 85.833, 83.75, 79.917, 83.75, 80.292, 46.548, 61.875, 67.542, 70.5, 87.042, 84.5, 79.583, 89.75, 92.0, 93.25, 91.25, 93.583]
    this.kData = [3.15, 4.529, 3.579, 3.292, 1.588, 1.914, 3.452, 3.638, 4.104, 3.117, 1.996, 4.617, 3.854, 2.783, 6.146, 6.888, 4.321, 2.013, 4.392, 3.433, 6.317, 4.321, 3.325, 6.313, 4.175, 3.258, 3.796, 0.996, 2.8, 3.083, 2.738, 4.969, 5.771, 2.717, 0.15, -0.363, 1.133, -0.083, 2.783, 2.779, 4.475, 3.996, 3.188, 3.254, 1.496, 1.6, 4.313, 8.704, 6.775, 4.713, 4.733, 2.638, 4.071, 4.879, 3.842, 0.342, 3.521, 1.454, 0.538, 0.963, 5.013, 5.633, 5.842, 5.875, 4.571, 4.567, 4.408, 4.2, 4.775, 4.479, 5.85, 4.413, 4.525, 0.108, 1.063, 3.529, 4.717, 7.113, 6.313, 3.908, 0.896, 0.221, 0.658, 2.313, 4.633, 6.25, 7.321, 7.052, 2.778, 0.892, 1.713, 4.542, 5.929, 9.875, 12.971, 10.346, 7.642, 6.217, 6.446, 7.833, 6.696, 2.092, 5.138, 6.621, 4.913, 5.563, 6.521, 7.913, 9.633, 9.779, 8.104, 6.117, 5.863, 6.992, 8.521, 5.696, 7.313, 11.567, 10.925, 9.713, 11.263, 10.925, 9.413, 11.263, 11.867, 11.4, 9.171, 10.067, 10.646, 14.117, 9.267, 6.692, 6.858, 6.667, 7.233, 7.921, 8.2, 8.404, 7.842, 8.367, 8.842, 8.367, 8.842, 10.838, 13.213, 11.683, 9.621, 13.233, 14.458, 13.129, 12.338, 14.854, 16.613, 14.292, 13.613, 14.225, 14.6, 17.242, 17.1, 16.95, 18.6, 14.376, 17.304, 19.563, 19.729, 19.267, 17.0, 16.167, 18.571, 20.8, 22.513, 23.85, 23.921, 21.15, 21.708, 21.642, 22.967, 25.421, 25.933, 23.85, 22.671, 17.279, 17.367, 17.942, 17.683, 17.854, 18.304, 18.579, 16.254, 14.779, 14.5, 14.517, 16.863, 15.088, 15.475, 16.546, 18.429, 18.521, 19.546, 21.221, 22.221, 22.829, 18.875, 15.342, 13.996, 13.742, 16.313, 18.969, 19.567, 20.954, 19.038, 15.8, 16.529, 17.9, 17.963, 18.254, 17.713, 18.525, 19.967, 21.217, 21.192, 22.096, 20.475, 18.179, 16.567, 15.442, 18.279, 20.371, 21.933, 19.789, 19.079, 20.813, 20.796, 21.646, 19.633, 19.258, 18.071, 15.013, 15.2, 15.829, 14.992, 13.604, 17.296, 15.242, 15.046, 15.95, 17.746, 17.15, 16.579, 14.613, 13.325, 12.538, 15.729, 14.758, 12.696, 15.304, 15.962, 16.758, 18.183, 19.688, 12.288, 12.996, 12.692, 12.4, 14.263, 15.475, 17.545, 18.129, 18.642, 19.471, 19.233, 14.517, 13.967, 13.183, 14.196, 17.158, 16.788, 16.004, 14.375, 13.4, 11.367, 9.963, 12.129, 10.388, 9.15, 8.017, 7.417, 6.075, 5.467, 4.988, 5.104, 6.438, 7.988, 11.55, 12.075, 10.3, 9.288, 10.629, 13.071, 9.054, 10.036, 8.921, 8.571, 8.7, 8.325, 11.096, 8.338, 8.533, 10.796, 10.008, 8.771, 6.117, 7.829, 5.883, 5.583, 4.521, 3.938, 4.321, 5.054, 6.478, 10.438, 9.808, 2.992, 2.108, 5.592, 8.746, 5.242, 3.321, 2.745, 1.713, 0.617, 1.696, -1.746, 0.125, 2.083, 4.533, 3.45, 0.646, -2.75, -3.638, -2.808, -0.752, 0.0, -1.892, -0.679, 0.258, 1.771, 4.958, 4.132, 5.658, 2.188]
    this.SData = [1016.663, 1013.083, 1007.133, 998.317, 1017.65, 1019.568, 1020.133, 1013.625, 1012.921, 1009.85, 1019.183, 1010.121, 1011.142, 1013.558, 1009.167, 1016.221, 1025.108, 1017.475, 1024.296, 1031.36, 1018.988, 1007.121, 1007.883, 1011.604, 1010.225, 1002.429, 994.858, 998.588, 995.683, 994.283, 986.313, 994.242, 1001.875, 1014.85, 1012.492, 1025.117, 1020.821, 1011.608, 979.971, 981.108, 988.504, 1000.138, 1014.429, 1020.842, 1007.842, 998.704, 1003.367, 1005.192, 1012.8, 1010.583, 1003.5, 991.45, 999.613, 997.075, 987.588, 990.529, 999.246, 1003.467, 994.629, 998.85, 1002.554, 1004.275, 1005.15, 1006.813, 1008.817, 1014.592, 1013.908, 1006.883, 990.979, 989.55, 994.679, 1011.158, 1022.883, 1014.854, 1021.363, 1016.1, 1016.096, 1016.096, 1020.383, 1027.463, 1038.175, 1041.4, 1038.125, 1034.983, 1030.817, 1019.648, 1017.604, 1019.563, 1021.554, 1006.1, 997.363, 993.846, 1015.046, 1030.154, 1031.963, 1025.154, 1026.429, 1013.904, 1019.85, 1024.517, 1014.458, 998.896, 1004.413, 1003.8, 999.15, 1009.017, 1014.738, 1018.546, 1025.046, 1027.888, 1026.471, 1017.558, 1005.558, 1002.65, 1005.388, 1009.817, 1002.504, 1006.071, 1011.958, 1008.942, 1003.829, 1008.204, 1013.554, 1012.67, 1010.31, 1012.67, 1010.81, 1001.067, 1004.983, 1006.888, 1011.725, 1010.363, 1006.925, 1007.633, 1010.363, 1006.925, 1007.633, 1010.504, 1012.221, 1016.833, 1018.404, 1021.913, 1021.763, 1016.613, 1022.113, 1030.292, 1027.079, 1024.917, 1024.317, 1024.229, 1020.683, 1020.846, 1021.858, 1016.625, 1009.7, 1002.875, 998.963, 1000.442, 1010.183, 1009.3, 1014.792, 1013.842, 1015.879, 1020.113, 1019.671, 1013.788, 1014.092, 1014.021, 1011.567, 1012.083, 1012.817, 1015.583, 1015.3, 1017.821, 1024.196, 1026.713, 1023.313, 1014.492, 1008.983, 1007.342, 1000.5, 1001.867, 1005.371, 1010.45, 1011.679, 1006.129, 1002.929, 1007.125, 1008.983, 1012.038, 1010.417, 1010.375, 1017.954, 1017.412, 1016.263, 1014.054, 1012.738, 1016.342, 1020.329, 1019.004, 1013.529, 1010.725, 1010.196, 1009.838, 1008.529, 1015.592, 1009.388, 1008.404, 1014.183, 1013.554, 1010.004, 1009.033, 1016.079, 1024.138, 1027.433, 10242.729, 1020.379, 1019.613, 1022.308, 1023.142, 1019.004, 1016.629, 1015.492, 1014.879, 1012.188, 1009.225, 1011.921, 1015.533, 1013.754, 1008.621, 1007.942, 1008.629, 1008.175, 1004.79, 1001.954, 1008.629, 1008.175, 1007.775, 1008.717, 1016.808, 1018.0, 1015.817, 1016.188, 1012.896, 1013.096, 1015.904, 1012.971, 1011.125, 1005.95, 1016.363, 1015.975, 1014.979, 1019.429, 1022.65, 1013.858, 1011.954, 1020.371, 1021.1, 1022.458, 1017.379, 1012.308, 1008.758, 1006.833, 1006.046, 1004.913, 1010.258, 1014.992, 1018.833, 1017.7, 1014.642, 1016.379, 1015.833, 1008.383, 1012.425, 1006.325, 1009.538, 1014.458, 1013.771, 1017.346, 1015.783, 1015.496, 1015.608, 1014.433, 1013.183, 1018.496, 1018.383, 1002.475, 1008.075, 1018.038, 1011.663, 1002.725, 1003.879, 1006.892, 1011.946, 1001.071, 1002.725, 1003.879, 1006.892, 1011.946, 1011.071, 1008.8, 1011.767, 1008.8, 1011.767, 1010.571, 1012.621, 1022.546, 1022.121, 1013.167, 1014.329, 1021.2, 1019.121, 1019.938, 1023.429, 1021.863, 1029.121, 1032.379, 1032.163, 1028.371, 1024.8, 1021.663, 1020.996, 1017.954, 1019.179, 1004.813, 1012.704, 1022.796, 1006.969, 1006.046, 1012.704, 1014.721, 1012.058, 1014.721, 1012.058, 1011.25, 1021.333, 1024.333, 1018.67, 1023.958, 1029.65, 1020.45, 1013.75, 1015.917, 1021.933, 1023.196, 1024.483, 1025.7, 1021.492, 1015.542, 1014.821, 1017.238, 1018.296, 1019.383]
    this.lData = [19.167, 16.458, 22.917, 20.625, 25.833, 17.273, 19.792, 27.292, 17.083, 13.542, 18.75, 26.875, 12.083, 23.75, 30.625, 23.542, 21.25, 21.042, 17.292, 13.542, 26.25, 18.333, 12.917, 14.792, 16.667, 23.542, 24.583, 13.158, 15.625, 29.792, 20.417, 16.458, 28.125, 38.542, 24.583, 19.583, 18.333, 10.833, 21.548, 36.458, 36.667, 25.625, 22.292, 16.042, 27.292, 35.417, 30.0, 25.625, 17.5, 21.458, 13.542, 18.125, 17.292, 24.975, 18.333, 16.042, 22.292, 12.917, 11.25, 15.208, 10.208, 20.0, 18.542, 27.5, 28.125, 32.917, 23.542, 26.25, 17.917, 25.833, 20.708, 14.646, 21.646, 18.292, 7.688, 14.354, 17.521, 12.667, 10.542, 11.109, 29.522, 10.563, 14.813, 20.771, 25.292, 34.813, 16.708, 13.042, 12.582, 24.313, 21.063, 20.542, 21.5, 18.188, 23.896, 22.583, 25.875, 22.813, 38.979, 33.5, 27.875, 23.958, 15.854, 16.174, 14.479, 16.167, 17.917, 22.125, 15.458, 17.313, 16.875, 22.375, 13.479, 25.583, 13.125, 8.313, 12.646, 14.313, 24.729, 25.208, 14.917, 12.229, 12.25, 22.958, 19.646, 25.042, 17.021, 18.833, 24.625, 21.896, 18.604, 11.375, 27.604, 31.958, 18.603, 12.104, 10.208, 11.396, 16.146, 13.208, 21.813, 27.417, 23.333, 28.792, 21.979, 20.854, 15.979, 14.0, 18.063, 23.313, 10.938, 10.125, 12.521, 12.875, 17.313, 20.083, 24.563, 24.292, 14.917, 9.833, 12.563, 11.896, 12.896, 15.979, 17.438, 15.167, 12.313, 14.833, 12.167, 12.0, 13.417, 12.375, 10.345, 23.563, 26.521, 16.875, 13.167, 17.042, 22.646, 24.063, 19.729, 17.146, 14.042, 11.104, 24.208, 11.063, 11.292, 9.563, 12.229, 13.063, 14.021, 10.958, 9.396, 12.688, 16.167, 22.125, 15.375, 16.083, 16.938, 13.708, 13.646, 9.021, 18.771, 22.833, 18.542, 19.5, 9.729, 12.063, 9.083, 12.417, 13.458, 11.438, 7.813, 8.464, 18.292, 14.542, 10.188, 9.667, 9.021, 10.917, 15.208, 11.75, 15.042, 12.521, 13.125, 26.292, 14.479, 19.271, 13.208, 16.083, 13.604, 11.104, 14.542, 16.271, 10.75, 13.917, 15.396, 14.042, 19.771, 18.521, 12.375, 13.625, 20.125, 20.458, 27.813, 14.771, 22.521, 22.313, 18.333, 11.771, 18.563, 41.521, 29.063, 17.5, 11.0, 14.625, 16.042, 16.042, 17.521, 18.833, 24.704, 25.729, 14.583, 10.083, 12.813, 17.604, 20.396, 24.25, 29.542, 18.292, 24.792, 18.417, 13.875, 19.0, 16.063, 9.542, 9.375, 9.271, 23.896, 16.75, 11.688, 9.75, 20.875, 25.021, 16.438, 18.0, 24.688, 21.521, 13.521, 14.125, 22.271, 23.917, 18.375, 12.864, 9.5, 6.667, 14.042, 27.896, 20.833, 17.25, 21.229, 14.458, 16.292, 17.729, 11.479, 7.292, 9.854, 18.958, 21.813, 18.542, 21.813, 18.542, 20.979, 31.875, 24.217, 26.479, 36.125, 21.229, 20.563, 23.292, 20.271, 19.021, 14.938, 20.813, 17.625, 18.667, 14.313, 13.313, 18.25, 17.438, 23.458, 39.833, 35.271, 35.958, 441.646, 28.438, 26.708, 18.438, 19.833, 24.229, 16.188, 9.354, 20.979, 21.229, 13.5, 20.438, 16.313, 24.604, 28.833, 23.354, 15.563, 15.188, 17.604, 9.25, 24.833]
  },
                               



  update: function (data) {
    //console.log(data.something)

      //AFRAME.utils.material.updateMap(this, data);
    
       var timeSec = (data.timeMsec/1000.0)*1.0;
       var fracttime = timeSec%1;
       var time = Math.trunc(timeSec);
    
       this.uniforms.timeMsec.value = data.timeMsec;
       this.uniforms.k.value = mix(this.kData[time%350],this.kData[(time+1)%350],fracttime);
       this.uniforms.l.value = mix(this.lData[time%350],this.lData[(time+1)%350],fracttime);
       this.uniforms.m.value = mix(this.mData[time%350],this.mData[(time+1)%350],fracttime);
       this.uniforms.S.value = mix(this.SData[time%350],this.SData[(time+1)%350],fracttime);

     
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
uniform float timeMsec, k, l, m, S;

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


void main() {



  vec3 color = vec3(0.0);
  float a = 1.0;
  
  float kinterp = k;
  float linterp = l;
  float minterp = map(m *0.15,2.44,10.0,0.0,20.0);
  float Sinterp = map(S,966.783,1024.6,0.0,5.0);
  
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

