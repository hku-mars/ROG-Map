#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 ourColor;

uniform mat4 view;
uniform mat4 projection;
uniform vec2 range;

void main()
{
	gl_Position = projection * view * vec4(aPos, 1.0f);
	if(gl_Position.z < range.x || gl_Position.z > range.y)
	{
		gl_Position = vec4(0.0f,0.0f,0.0f,0.0f);
	}
	ourColor = aColor;
}