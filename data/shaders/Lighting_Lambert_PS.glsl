#version 130

uniform		sampler2D	gTexture;
uniform		sampler2D	gShadowTex;

uniform		mat4		gShadowProj;
uniform		vec3		gLightDir;
uniform		vec3		gLightColour;
uniform		vec3		gAmbientColour;
uniform		vec4		gMaterialData;
uniform		vec4		gFogColor;
uniform		vec4		gFogData;

in		vec3	ViewPos;
in		vec3	Normal;
in		vec4	VertColor;
in		vec2	TexCoord;

out 	vec4 	outColor;

float CalcDiffuse(vec3 world_space_normal, vec3 light_direction)	// Roughness value [0 ~ 1], 0 is same as Lambertian
{
	vec3 n = world_space_normal;
	vec3 l = light_direction;
	float l_dot_n = dot( l, n);
	float diffuse = clamp(l_dot_n, 0, 1);
	return diffuse;
}

float CalcSpecular(
	const vec3 world_normal,
	const vec3 light_direction,
	const vec3 view_direction)
{

	vec3 n = world_normal;
	vec3 v = view_direction;
	vec3 l = light_direction;
	vec3 h = normalize(view_direction + light_direction);
	
	float n_dot_h = dot(n, h);
	float spec = clamp(n_dot_h, 0, 1);
	spec = pow(spec, 10);
	return 0.0;
}

vec3 CalculateBRDF( vec3 normal, vec3 light_dir, 
						vec3 light_colour, vec3 view_dir, float roughness,
						vec3 albedo )
{
	float diffuse_coef = CalcDiffuse(normal, light_dir);
	float spec_coef = CalcSpecular(normal, light_dir, view_dir);

	return diffuse_coef * light_colour * albedo
			+ spec_coef * light_colour;
}

vec2 RotateDirections(vec2 Dir, float theta)
{
	float cos_theta = cos(theta);
	float sin_theta = sin(theta);
    return vec2(Dir.x*cos_theta - Dir.y*sin_theta,
                  Dir.x*sin_theta + Dir.y*cos_theta);
}

float CalculateShadow(vec3 view_pos, vec3 normal)
{
	float bias = 0.04f;
	vec4 position = vec4(view_pos + bias * normal, 1.f);

	vec3 shadow_coord = (gShadowProj * position).xyz;
	float depth = ( shadow_coord.z * 0.5 + 0.5 );// + 0.01f;
	depth = min(depth, 1);

	shadow_coord.xy = shadow_coord.xy * 0.5f + 0.5f;

	float sample_depth = ( texture( gShadowTex, shadow_coord.xy).r );
	sample_depth = (depth <= sample_depth) ? 1.f : 0.f;


	// pcf taps for anti-aliasing
	const int num_samples = 16;
	vec2 poissonDisk[num_samples] = vec2[]( vec2( -0.6474742f, 0.6028621f ),
									vec2( 0.0939157f, 0.6783564f ),
									vec2( -0.3371512f, 0.04865054f ),
									vec2( -0.4010732f, 0.914994f ),
									vec2( -0.2793565f, 0.4456959f ),
									vec2( -0.6683437f, -0.1396244f ),
									vec2( -0.6369296f, -0.6966243f ),
									vec2( -0.2684143f, -0.3756073f ),
									vec2( 0.1146429f, -0.8692533f ),
									vec2( 0.0697926f, 0.01110036f ),
									vec2( 0.4677842f, 0.5375957f ),
									vec2( 0.377133f, -0.3518053f ),
									vec2( 0.6722369f, 0.03702459f ),
									vec2( 0.6890426f, -0.5889201f ),
									vec2( -0.8208677f, 0.2444565f ),
									vec2( 0.8431721f, 0.3903837f ));
	float shadow_map_size = 2048;
	float r = 2;

	const vec4 noise_params = vec4(1, 1, 0.1, 0.2);
	vec2 seed = view_pos.xy * noise_params.xy + noise_params.zw;
	float rand_theta = fract(sin(dot(seed, vec2(12.9898,78.233))) * 43.5453);
	rand_theta = 2 * 3.14159f * rand_theta - 3.14159f;

	for ( int i = 0; i < num_samples; ++i )
	{
		vec2 dir = poissonDisk[i];
		dir = RotateDirections(dir, rand_theta);
		vec2 tap_coord = shadow_coord.xy +  r * dir / shadow_map_size;
		float tap = texture( gShadowTex, tap_coord ).r;
		sample_depth += ( depth <= tap ) ? 1.f : 0.f;
	}

	float shadow_coef = sample_depth / (num_samples + 1);
	return shadow_coef;
}

vec3 CalcAmbient(vec3 normal, vec3 albedo)
{
	vec3 ambient = gAmbientColour;
	//ambient = vec3(0.6, 0.6, 0.6);
	ambient *= albedo;
	return ambient;
}

float CalcFog(vec3 pos)
{
	float fog_cutoff = gFogData.x;
	float fade_distsq = gFogData.y;
	float dist = length(pos);
	dist = max(0, dist - fog_cutoff);
	float fog = 1 - exp(-fade_distsq * dist * dist);
	return fog;
}

void main()
{
	float roughness = gMaterialData.x;
	bool enable_albedo_tex = gMaterialData.y != 0;
	vec3 view_dir = -normalize(ViewPos);
	vec3 norm = normalize(Normal);

	vec3 albedo = VertColor.rgb;
	if (enable_albedo_tex)
	{
		vec4 tex_col = texture(gTexture, TexCoord);
		albedo.rgb = albedo.rgb * tex_col.rgb;
	}
	
	float shadow_coef = CalculateShadow(ViewPos, norm);
	vec3 light_colour = gLightColour;
	//light_colour = vec3(0.5, 0.5, 0.5);
	light_colour *= shadow_coef;
	vec3 light_result = CalculateBRDF(norm, gLightDir, 
										light_colour, view_dir, roughness,
										albedo);

	vec3 ambient = CalcAmbient(norm, albedo.rgb);
	light_result += ambient;

	float fog = CalcFog(ViewPos);
	fog *= gFogColor.w;
	light_result = mix(light_result, gFogColor.xyz, fog);

	outColor = vec4(light_result, VertColor[3]);
}