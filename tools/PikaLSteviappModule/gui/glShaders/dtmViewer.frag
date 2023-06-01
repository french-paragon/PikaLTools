#version 150

in vec3 normal;

uniform vec3 lightDirection;

void main(void)
{
    vec4 color = vec4(0.9, 0.9, 0.9, 1.0);

    if (!gl_FrontFacing) {
        color = vec4(0.55, 0.6, 0.65, 1.0);
    }

    float light = max(dot(lightDirection, normal), 0.1f);
    gl_FragColor = light*color;
}
