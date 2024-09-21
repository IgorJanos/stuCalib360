
uniform mat4 MVP;

attribute highp vec4 vertex;
attribute highp vec2 tex;

varying highp vec2 t;


void main(void)
{
    gl_Position = MVP * vertex;
    t = tex;
}
