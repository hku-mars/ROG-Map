#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec3 ourColor;

uniform mat4 view;
uniform mat4 projection;
uniform vec2 range;

uniform mat3 rot;
uniform vec3 pos;

// uniform vec3 ()
uniform vec2 fov;
uniform vec2 res;
// uniform float downsample_res;
// uniform float polar_res;

void main()
{
	float PI = 3.14159265358;
	// float yaw_fov = 360.0;
	// float vertical_fov = 50.0;
	// float downsample_res = 0.1;
	// float polar_res = 0.2;
	// float cover_dis = 0.55 * 1.7321 * downsample_res;
	float cover_dis = 0.55 * 1.7321 * res.x;

	//log(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y) + 1/sqrt(1 + pow(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y),2)))
	//atan2(aPos.z,sqrt(aPos.x*aPos.x + aPos.y*aPos.y))
	// if(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y) > 0 )
	// {
		// sqrt(aPos.x*aPos.x + aPos.y*aPos.y + aPos.z*aPos.z)
		// (atan2(aPos.y,aPos.x)+PI)/(2*PI)
			// gl_Position = vec4(atan(aPos.y/aPos.x),atan(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y)), 0.0f, 1.0f);
			// gl_Position = vec4(atan(aPos.y/aPos.x),atan(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y)), 0.0f, 1.0f);
			// gl_Position = vec4((aPos.y*aPos.x),(aPos.z*(aPos.x*aPos.x + aPos.y*aPos.y)), 0.0f, 1.0f);

	// }
	// gl_Position = projection * view * vec4(aPos, 1.0f);
	
	//normal speed
	// gl_Position = vec4(aPos,1.0f);

	//normal speed
	// gl_Position = vec4(aPos*aPos,1.0f);

	//normal speed
	// gl_Position = vec4(aPos.x*aPos.x, aPos.y*aPos.y, 0.0f,1.0f);

	// cost 0.5s 1 frame
	// gl_Position = vec4(atan(aPos.y/aPos.x),atan(aPos.z/sqrt(aPos.x*aPos.x + aPos.y*aPos.y)), 0.0f, 1.0f);

	//trans from world to body coordinate
	vec3 bodypos = rot * (aPos - pos);
	// vec4 bodypos4 = view * vec4(aPos, 1.0f);
	// vec3 bodypos;
	// bodypos.x = bodypos4.x;
	// bodypos.y = bodypos4.y;
	// bodypos.z = bodypos4.z;

	//opengl polar coordinate, large point cloud slow
	float y_x;// 
	float z_yx;//
	float depth = sqrt(bodypos.x*bodypos.x + bodypos.y*bodypos.y + (bodypos.z)*(bodypos.z));///range.y
	// depth = -depth* (zFar + zNear) / (zFar - zNear); 

	if(depth > range.y || depth < range.x)
	{
		gl_Position = vec4(0.0f,0.0f,0.0f,0.0f);
		ourColor = aColor;
	}else{
		y_x = -(atan(bodypos.y,bodypos.x))/(PI)* (360.0/fov.x);// 
		z_yx = (atan((bodypos.z),sqrt(bodypos.x*bodypos.x + bodypos.y*bodypos.y))/PI)* (360.0/fov.y) ;//
		// depth = (depth + 1.0) * 2.0;
		gl_Position = vec4(y_x, z_yx, depth/range.y, 1.0f);
		// int half_cover_angle = ceil((asin(cover_dis / 1.0) / (PI * polar_res / 180.0)));//dot(aPos,aPos)
		gl_PointSize = 2*ceil((asin(cover_dis / (depth)) / (PI * res.y / 180.0)));
		// gl_PointSize = 1;		
		// if(abs(y_x) < 0.05 && abs(z_yx) < 0.05)
		// {
		// 	ourColor = vec3(depth,0.0f,(depth));
		// }else{
			// ourColor = vec3(depth,(depth),(depth));
		// }
		
	}

	// //opengl polar coordinate, large point cloud slow
	// float y_x;// 
	// float z_yx;//
	// float depth = sqrt(aPos.x*aPos.x + aPos.y*aPos.y + (aPos.z -1)*(aPos.z -1));///range.y
	// // depth = -depth* (zFar + zNear) / (zFar - zNear); 

	// if(depth > range.y || depth < range.x)
	// {
	// 	gl_Position = vec4(0.0f,0.0f,0.0f,0.0f);
	// 	ourColor = aColor;
	// }else{
	// 	y_x = -(atan(aPos.y,aPos.x))/(PI)* (360.0/fov.x);// 
	// 	z_yx = (atan((aPos.z-1),sqrt(aPos.x*aPos.x + aPos.y*aPos.y))/PI)* (360.0/fov.y) ;//
	// 	// depth = (depth + 1.0) * 2.0;
	// 	gl_Position = vec4(y_x, z_yx, depth/range.y, 1.0f);
	// 	// int half_cover_angle = ceil((asin(cover_dis / 1.0) / (PI * polar_res / 180.0)));//dot(aPos,aPos)
	// 	gl_PointSize = 2*ceil((asin(cover_dis / (depth)) / (PI * res.y / 180.0)));
	// 	// gl_PointSize = 1;		
	// 	// if(abs(y_x) < 0.05 && abs(z_yx) < 0.05)
	// 	// {
	// 	// 	ourColor = vec3(depth,0.0f,(depth));
	// 	// }else{
	// 		ourColor = vec3(depth,(depth),(depth));
	// 	// }
		
	// }


	//opengl three image
	// gl_Position = projection *view * vec4(aPos, 1.0f);// 

	// gl_Position = vec4(0.0f,0.0f,-10.0f,1.0f);
	// gl_PointSize = 5;
	// if(gl_Position.z < range.x || gl_Position.z > range.y)
	// {
	// 	gl_Position = vec4(0.0f,0.0f,0.0f,0.0f);
	// }
	ourColor = aColor;
}