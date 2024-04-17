attribute vec3 aVertexPosition;
attribute vec3 aNormalPosition;
attribute mat3 aPrecomputeLT;

uniform mat4 uModelMatrix;
uniform mat4 uViewMatrix;
uniform mat4 uProjectionMatrix;
uniform mat3 uPrecomputeL[3];

varying vec3 vNormal;
varying vec3 vColor;

void main(void) {
  vNormal = (uModelMatrix * vec4(aNormalPosition, 0.0)).xyz;

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      vColor[i] += dot(uPrecomputeL[i][j], aPrecomputeLT[j]);
    }
  }
  gl_Position = uProjectionMatrix * uViewMatrix * uModelMatrix *
                vec4(aVertexPosition, 1.0);
}