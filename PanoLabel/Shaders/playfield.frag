
uniform sampler2D texture;
varying highp vec2 t;


void main(void)
{
    vec4 color = texture2D(texture, t);
    //color.a = 0.5;

    gl_FragColor = color; //vec4(1.0, 0.0, 0.0, 0.5);
}

