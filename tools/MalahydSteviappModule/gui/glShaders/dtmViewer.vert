#version 150

in vec3 in_location;
in vec3 in_normal;

uniform mat4 matrixViewProjection;

uniform float sceneScale;

out vec3 normal;

void main(void)
{
    gl_Position = matrixViewProjection * vec4(in_location, 1/sceneScale);
    normal = in_normal;
}
