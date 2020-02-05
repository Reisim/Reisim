#version 330

layout(location = 0) in vec3 vertex;
layout(location = 1) in vec2 texCoord;
layout(location = 2) in vec3 gColor;
out vec2 texc;
out vec3 pColor;
out vec3 position;

uniform mat4 modelToWorld;
uniform mat4 worldToView;
uniform int isText;
uniform vec3 letterPos;

void main(void)
{
	vec3 pos = vertex;
	if( isText == 100 ){
		pos += letterPos;
	}
	gl_Position = worldToView * modelToWorld * vec4(pos,1.0);
	texc = texCoord;
	pColor = gColor;
	position = vertex.xyz;
}

