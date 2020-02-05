#version 330

uniform sampler2D texture;
uniform int useTex;
uniform vec3 offsetPos;
uniform vec4 vColor;

in vec2 texc;
in vec3 pColor;
in vec3 position;

out highp vec4 fColor;

void main(){
	if( useTex == 1 ){
		fColor = texture2D(texture, texc);
	}
	else if( useTex == 2 ){
		if( position.x < offsetPos.x ){
			fColor = vec4(1.0,0.0,0.0,1.0);
		}
		else{
			fColor = vColor;
		}
	}
	else if( useTex == 3 ){
		if( position.x > offsetPos.y && position.y > offsetPos.z ){
			fColor = vec4(1.0,1.0,0.0,1.0);
		}
		else{
			fColor = vColor;
		}
	}
	else if( useTex == 4 ){
		if( position.x > offsetPos.y && position.y < -offsetPos.z ){
			fColor = vec4(1.0,1.0,0.0,1.0);
		}
		else{
			fColor = vColor;
		}
	}
	else if( useTex == 5 ){
		if( position.x < offsetPos.x ){
			fColor = vec4(1.0,0.0,0.0,1.0);
		}
		else if( position.x > offsetPos.y && position.y > offsetPos.z ){
			fColor = vec4(1.0,1.0,0.0,1.0);
		}
		else{
			fColor = vColor;
		}
	}
	else if( useTex == 6 ){
		if( position.x < offsetPos.x ){
			fColor = vec4(1.0,0.0,0.0,1.0);
		}
		else if( position.x > offsetPos.y && position.y < -offsetPos.z ){
			fColor = vec4(1.0,1.0,0.0,1.0);
		}
		else{
			fColor = vColor;
		}
	}
	else if( useTex == 12 ){
		if( position.x < offsetPos.x ){
			fColor = vec4(1.0,0.0,0.0,1.0);
		}
		else{
			fColor = texture2D(texture, texc);
		}
	}
	else if( useTex == 13 ){
		if( position.x > offsetPos.y && position.y > offsetPos.z ){
			fColor = vec4(1.0,1.0,0.0,1.0);
		}
		else{
			fColor = texture2D(texture, texc);
		}
	}
	else if( useTex == 14 ){
		if( position.x > offsetPos.y && position.y < -offsetPos.z ){
			fColor = vec4(1.0,1.0,0.0,1.0);
		}
		else{
			fColor = texture2D(texture, texc);
		}
	}
	else if( useTex == 15 ){
		if( position.x < offsetPos.x ){
			fColor = vec4(1.0,0.0,0.0,1.0);
		}
		else if( position.x > offsetPos.y && position.y > offsetPos.z ){
			fColor = vec4(1.0,1.0,0.0,1.0);
		}
		else{
			fColor = texture2D(texture, texc);
		}
	}
	else if( useTex == 16 ){
		if( position.x < offsetPos.x ){
			fColor = vec4(1.0,0.0,0.0,1.0);
		}
		else if( position.x > offsetPos.y && position.y < -offsetPos.z ){
			fColor = vec4(1.0,1.0,0.0,1.0);
		}
		else{
			fColor = texture2D(texture, texc);
		}
	}
	else if( useTex == 10 ){
		fColor = vColor;
	}
	else if( useTex == 100 ){
		fColor = vec4( 1, 1, 1, texture2D(texture, texc).r ) * vColor;
	}
	else if( useTex == 101 ){
		if( 0.05 < texc.x && texc.x < 0.15 ){
			fColor = vec4( pColor, 1.0 );
		}
		else{
			fColor = vec4(0.97,0.97,0.97,1.0);
		}
	}
	else if( useTex == 102 ){
		if( 0.15 < texc.x && texc.x < 0.25 ){
			fColor = vec4( pColor, 1.0 );
		}
		else{
			fColor = vec4(0.97,0.97,0.97,1.0);
		}
	}
	else if( useTex == 103 ){
		if( 0.25 < texc.x && texc.x < 0.35 ){
			fColor = vec4( pColor, 1.0 );
		}
		else{
			fColor = vec4(0.97,0.97,0.97,1.0);
		}
	}
	else if( useTex == 104 ){
		if( (0.25 < texc.x && texc.x < 0.35) || (0.55 < texc.x && texc.x < 0.65)){
			fColor = vec4( pColor, 1.0 );
		}
		else{
			fColor = vec4(0.97,0.97,0.97,1.0);
		}
	}
	else if( useTex == 105 ){
		if( (0.25 < texc.x && texc.x < 0.35) || (0.45 < texc.x && texc.x < 0.55)){
			fColor = vec4( pColor, 1.0 );
		}
		else{
			fColor = vec4(0.97,0.97,0.97,1.0);
		}
	}
	else if( useTex == 106 ){
		if( (0.25 < texc.x && texc.x < 0.35) || (0.45 < texc.x && texc.x < 0.65) ){
			fColor = vec4( pColor, 1.0 );
		}
		else{
			fColor = vec4(0.97,0.97,0.97,1.0);
		}
	}
	else if( useTex == 107 ){
		if( (0.25 < texc.x && texc.x < 0.35) || (0.35 < texc.x && texc.x < 0.45) ){
			fColor = vec4( pColor, 1.0 );
		}
		else{
			fColor = vec4(0.97,0.97,0.97,1.0);
		}
	}
	else if( useTex == 108 ){
		if( (0.25 < texc.x && texc.x < 0.35) || (0.35 < texc.x && texc.x < 0.55) ){
			fColor = vec4( pColor, 1.0 );
		}
		else{
			fColor = vec4(0.97,0.97,0.97,1.0);
		}
	}
	else if( useTex == 108 ){
		if( (0.25 < texc.x && texc.x < 0.35) || (0.35 < texc.x && texc.x < 0.55) ){
			fColor = vec4( pColor, 1.0 );
		}
		else{
			fColor = vec4(0.97,0.97,0.97,1.0);
		}
	}
	else if( useTex == 109 ){
		fColor = vec4(0.97,0.97,0.97,1.0);
	}
	else{
		fColor = vec4( pColor, 1.0 );
	}
}

