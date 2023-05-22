#version 150

in float sequence_pos;

uniform float segmentStart;
uniform float segmentEnd;

void main(void)
{
    gl_FragColor = vec4(0.9, 0.0, 0.0, 1.0);

    if (sequence_pos <= segmentEnd && sequence_pos >= segmentStart) {
        gl_FragColor = vec4(0.0, 0.0, 0.9, 1.0);
    }
}
