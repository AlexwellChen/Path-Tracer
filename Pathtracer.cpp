#include "Pathtracer.h"
#include <memory>
#include <iostream>
#include <map>
#include <algorithm>
#include "material.h"
#include "embree.h"
#include "sampling.h"

using namespace std;
using namespace glm;

namespace pathtracer
{
	///////////////////////////////////////////////////////////////////////////////
	// Global variables
	///////////////////////////////////////////////////////////////////////////////
	Settings settings;
	Environment environment;
	Image rendered_image;
	PointLight point_light;
	int refraction_flag = 0; // 0 out of obj, 1 in the obj

	///////////////////////////////////////////////////////////////////////////
	// Restart rendering of image
	///////////////////////////////////////////////////////////////////////////
	void restart()
	{
		// No need to clear image,
		rendered_image.number_of_samples = 0;
	}

	///////////////////////////////////////////////////////////////////////////
	// On window resize, window size is passed in, actual size of pathtraced
	// image may be smaller (if we're subsampling for speed)
	///////////////////////////////////////////////////////////////////////////
	void resize(int w, int h)
	{
		rendered_image.width = w / settings.subsampling;
		rendered_image.height = h / settings.subsampling;
		rendered_image.data.resize(rendered_image.width * rendered_image.height);
		restart();
	}

	///////////////////////////////////////////////////////////////////////////
	// Return the radiance from a certain direction wi from the environment
	// map.
	///////////////////////////////////////////////////////////////////////////
	vec3 Lenvironment(const vec3& wi)
	{
		const float theta = acos(std::max(-1.0f, std::min(1.0f, wi.y)));
		float phi = atan(wi.z, wi.x);
		if (phi < 0.0f)
			phi = phi + 2.0f * M_PI;
		vec2 lookup = vec2(phi / (2.0 * M_PI), theta / M_PI);
		return environment.multiplier * environment.map.sample(lookup.x, lookup.y);
	}

	//vec3 reflect_Li(Intersection hit, Ray currentRay, vec3 pathThroughput, vec3 L) {
	//	Ray shadowRay = Ray();
	//	Diffuse diffuse(hit.material->m_color);
	//	BlinnPhong dielectric(hit.material->m_shininess, hit.material->m_fresnel, &diffuse);
	//	BlinnPhongMetal metal(hit.material->m_color, hit.material->m_shininess,
	//		hit.material->m_fresnel);
	//	LinearBlend metal_blend(hit.material->m_metalness, &metal, &dielectric);
	//	LinearBlend reflectivity_blend(hit.material->m_reflectivity, &metal_blend, &diffuse);
	//	BRDF& mat = reflectivity_blend;


	//	shadowRay = Ray();
	//	shadowRay.d = point_light.position - (hit.position + hit.shading_normal * EPSILON);
	//	shadowRay.o = hit.position + hit.shading_normal * EPSILON;
	//	if (!occluded(shadowRay)) {
	//		const float distance_to_light = length(point_light.position - hit.position);
	//		const float falloff_factor = 1.0f / (distance_to_light * distance_to_light);
	//		vec3 Li = point_light.intensity_multiplier * point_light.color * falloff_factor;
	//		vec3 wi = normalize(point_light.position - hit.position);
	//		L += mat.f(wi, hit.wo, hit.shading_normal) * Li * std::max(0.0f, dot(wi, hit.shading_normal));
	//	}
	//	//emitted light
	//	L += pathThroughput * hit.material->m_color * hit.material->m_emission;

	//	float p;
	//	vec3 wi;
	//	vec3 brdf = mat.sample_wi(wi, hit.wo, hit.shading_normal, p);
	//	float cosineterm = abs(dot(wi, hit.shading_normal));
	//	if (p < EPSILON) return L;
	//	pathThroughput = pathThroughput * (brdf * cosineterm) / p;
	//	if (pathThroughput == vec3(0.0f)) return L;
	//	currentRay = wi;
	//	currentRay.o = EPSILON + hit.shading_normal;
	//	if (!intersect(currentRay)) {
	//		return L + pathThroughput * Lenvironment(wi);
	//	}
	//}

	//vec3 refraction_Li(Intersection hit, Ray currentRay, vec3 pathThroughput, vec3 L) {

	//}

	///////////////////////////////////////////////////////////////////////////
	// Calculate the radiance going from one point (r.hitPosition()) in one
	// direction (-r.d), through path tracing.
	///////////////////////////////////////////////////////////////////////////
	vec3 Li(Ray& primary_ray)
	{
		vec3 L = vec3(0.0f);
		vec3 pathThroughput = vec3(1.0f);
		Ray currentRay = primary_ray;
		Ray shadowRay;
		for (int bonces = 0; bonces < settings.max_bounces; bonces++) {
			Intersection hit = getIntersection(currentRay);
			Diffuse diffuse(hit.material->m_color);
			Glass glassBall(1.8f);
			BlinnPhong dielectric(hit.material->m_shininess, hit.material->m_fresnel, &diffuse);
			BlinnPhongMetal metal(hit.material->m_color, hit.material->m_shininess,
				hit.material->m_fresnel);
			
			LinearBlend metal_blend(hit.material->m_metalness, &metal, &dielectric);
			LinearBlend reflectivity_blend(hit.material->m_reflectivity, &metal_blend, &diffuse);;
			LinearBlend refraction_blend(hit.material->m_transparency, &reflectivity_blend, &glassBall);
			BRDF& mat = refraction_blend;


			shadowRay = Ray();
			shadowRay.d = point_light.position - (hit.position + hit.shading_normal * EPSILON);
			shadowRay.o = hit.position + hit.shading_normal * EPSILON;
			if (!occluded(shadowRay)) {
				const float distance_to_light = length(point_light.position - hit.position);
				const float falloff_factor = 1.0f / (distance_to_light * distance_to_light);
				vec3 Li = point_light.intensity_multiplier * point_light.color * falloff_factor;
				vec3 wi = normalize(point_light.position - hit.position);
				L += mat.f(wi, hit.wo, hit.shading_normal) * Li * std::max(0.0f, dot(wi, hit.shading_normal));
			}
			//emitted light
			L += pathThroughput * hit.material->m_color * hit.material->m_emission;

			float p = 1.0f;
			vec3 wi;
			vec3 brdf = mat.sample_wi(wi, hit.wo, hit.shading_normal, p);
			
			float cosineterm = abs(dot(wi, hit.shading_normal));
			if (p < EPSILON) return L;
			pathThroughput = pathThroughput * (brdf * cosineterm) / p;
			if (pathThroughput == vec3(0.0f)) return L;
			currentRay = Ray(hit.position, wi);
			//currentRay.o += EPSILON * -new_normal;
			/*currentRay.d = wi;
			currentRay.o = hit.position;*/
			if (!intersect(currentRay)) {
				return L + pathThroughput * Lenvironment(currentRay.d);
			}

		}
		return vec3(0.0f);
	}

	//vec3 trace(Ray& currentRay, Ray& shadowRay, vec3& pathThroughput, vec3& L, int bonces) {
	//	if (bonces > settings.max_bounces) {
	//		//return vec3(0.0f);
	//		return L + vec3(1.0f, 1.0f, 0.0f);
	//	}

	//	vec3 reflectAnimation = vec3(0.0f);
	//	vec3 refractAnimation = vec3(0.0f);
	//	if (!intersect(currentRay)) {
	//		//return L + pathThroughput * Lenvironment(currentRay.d);
	//		return L + vec3(1.0f, 1.0f, 0.0f);
	//	}
	//	Intersection hit = getIntersection(currentRay);
	//	Diffuse diffuse(hit.material->m_color);
	//	BlinnPhong dielectric(hit.material->m_shininess, hit.material->m_fresnel, &diffuse);
	//	BlinnPhongMetal metal(hit.material->m_color, hit.material->m_shininess,
	//		hit.material->m_fresnel);
	//	LinearBlend metal_blend(hit.material->m_metalness, &metal, &dielectric);
	//	LinearBlend reflectivity_blend(hit.material->m_reflectivity, &metal_blend, &diffuse);
	//	BRDF& mat = reflectivity_blend;

	//	// direct illumination
	//	shadowRay = Ray();
	//	shadowRay.d = point_light.position - (hit.position + hit.shading_normal * EPSILON);
	//	shadowRay.o = hit.position + hit.shading_normal * EPSILON;
	//	if (!occluded(shadowRay)) {
	//		const float distance_to_light = length(point_light.position - hit.position);
	//		const float falloff_factor = 1.0f / (distance_to_light * distance_to_light);
	//		vec3 Li = point_light.intensity_multiplier * point_light.color * falloff_factor;
	//		vec3 wi = normalize(point_light.position - hit.position);
	//		L += mat.f(wi, hit.wo, hit.shading_normal) * Li * std::max(0.0f, dot(wi, hit.shading_normal));
	//	}

	//	// emitted illumination
	//	L += pathThroughput * hit.material->m_color * hit.material->m_emission;

	//	float p;
	//	vec3 wi;
	//	vec3 brdf = mat.sample_wi(wi, hit.wo, hit.shading_normal, p);
	//	float cosineterm = abs(dot(wi, hit.shading_normal));
	//	if (p < EPSILON) {
	//		return L + vec3(1.0f, 1.0f, 0.0f) * (1 - hit.material->m_transparency);
	//		//return L;
	//	}
	//	pathThroughput = pathThroughput * (brdf * cosineterm) / p;
	//	if (pathThroughput == vec3(0.0f)) return L;

	//	// switch ray
	//	Ray reflectRay = Ray(hit.position, wi);
	//	reflectRay.o += EPSILON * hit.shading_normal;

	//	// refraction illumination
	//	float f = hit.material->m_fresnel;
	//	if (hit.material->m_transparency > 0) {
	//		if (refraction_flag == 0) {
	//			// out of the object
	//			Ray refraction_ray = Ray(currentRay.d, hit.position);
	//			refraction_flag = 1;
	//			refractAnimation = trace(refraction_ray, shadowRay, pathThroughput * (1-f), L, bonces + 1) * hit.material->m_transparency +
	//									 hit.material->m_color * (1 - hit.material->m_transparency) * pathThroughput * (1-f);
	//			reflectAnimation = trace(reflectRay, shadowRay, pathThroughput * f, L, bonces + 1);
	//			//return L + refractAnimation + reflectAnimation;
	//			return L + vec3(1.0f, 1.0f, 0.0f) * (1 - hit.material->m_transparency);
	//		} else{
	//			// in the object
	//			Ray refraction_ray = Ray(currentRay.d, hit.position);
	//			refraction_flag = 0;
	//			if (intersect(refraction_ray)) {
	//				Intersection out_point = getIntersection(refraction_ray);
	//				//return L + trace(refraction_ray, shadowRay, pathThroughput, L, bonces + 1);
	//				return L + vec3(1.0f, 1.0f, 0.0f) * (1 - hit.material->m_transparency);
	//			}
	//			else {
	//				//return L + Lenvironment(refraction_ray.d) * (1 - hit.material->m_transparency);
	//				return L + vec3(1.0f, 1.0f, 0.0f) * (1 - hit.material->m_transparency);
	//			}
	//		}
	//	}
	//	else{
	//		// switch ray
	//		currentRay = reflectRay;
	//		if (!intersect(currentRay)) {
	//			return L + vec3(1.0f, 1.0f, 0.0f) * (1 - hit.material->m_transparency);
	//			//return L + pathThroughput * Lenvironment(currentRay.d);
	//		}
	//		// go deeper
	//		//return L + trace(currentRay, shadowRay, pathThroughput, L, bonces + 1);
	//		return L + vec3(1.0f, 1.0f, 0.0f) * (1 - hit.material->m_transparency);
	//	}
	//}

	//vec3 Li(Ray& primary_ray) {
	//	vec3 L = vec3(0.0f);
	//	vec3 pathThroughput = vec3(1.0f);
	//	Ray currentRay = primary_ray;
	//	Ray shadowRay;
	//	int bonces = 0;

	//	return trace(currentRay, shadowRay, pathThroughput, L, bonces);
	//}

	///////////////////////////////////////////////////////////////////////////
	// Used to homogenize points transformed with projection matrices
	///////////////////////////////////////////////////////////////////////////
	inline static glm::vec3 homogenize(const glm::vec4& p)
	{
		return glm::vec3(p * (1.f / p.w));
	}

	///////////////////////////////////////////////////////////////////////////
	// Trace one path per pixel and accumulate the result in an image
	///////////////////////////////////////////////////////////////////////////
	void tracePaths(const glm::mat4& V, const glm::mat4& P)
	{
		// Stop here if we have as many samples as we want
		if ((int(rendered_image.number_of_samples) > settings.max_paths_per_pixel)
			&& (settings.max_paths_per_pixel != 0))
		{
			return;
		}
		vec3 camera_pos = vec3(glm::inverse(V) * vec4(0.0f, 0.0f, 0.0f, 1.0f));
		// Trace one path per pixel (the omp parallel stuf magically distributes the
		// pathtracing on all cores of your CPU).
		int num_rays = 0;
		vector<vec4> local_image(rendered_image.width * rendered_image.height, vec4(0.0f));

#pragma omp parallel for
		for (int y = 0; y < rendered_image.height; y++)
		{
			for (int x = 0; x < rendered_image.width; x++)
			{
				vec3 color = vec3(0);
				Ray primaryRay;
				primaryRay.o = camera_pos;
				// Create a ray that starts in the camera position and points toward
				// the current pixel on a virtual screen.
				vec2 screenCoord = vec2(float(x) / float(rendered_image.width),
					float(y) / float(rendered_image.height));
				float dx = 1.0f / float(rendered_image.width);
				float dy = 1.0f / float(rendered_image.height);
				// Calculate direction
				vec4 viewCoord = vec4(screenCoord.x * 2.0f - 1.0f, screenCoord.y * 2.0f - 1.0f, 1.0f, 1.0f);
				//for (int i = 0; i < 5; i++) {
				//	vec3 p = homogenize(inverse(P * V) * viewCoord) + vec3(randf() * 0.1f - 1.0f);
				//	primaryRay.d = normalize(p - camera_pos);
				//	// Intersect ray with scene
				//	if(intersect(primaryRay))
				//	{
				//		// If it hit something, evaluate the radiance from that point
				//		color += Li(primaryRay);
				//	}
				//	else
				//	{
				//		// Otherwise evaluate environment
				//		color += Lenvironment(primaryRay.d);
				//	}
				//}
				//color = color / 5.0f;

				for (int i = -1; i < 1; i++)
					for (int j = -1; j < 1; j++) {
						viewCoord.x += (i + randf()) * dx;
						viewCoord.y += (j + randf()) * dy;
						vec3 p = homogenize(inverse(P * V) * viewCoord);
						primaryRay.d = normalize(p - camera_pos);
						// Intersect ray with scene
						if (intersect(primaryRay))
						{
							// If it hit something, evaluate the radiance from that point
							color += Li(primaryRay);
						}
						else
						{
							// Otherwise evaluate environment
							color += Lenvironment(primaryRay.d);
						}
					}
				color = color / 4.0f;

				//vec3 p = homogenize(inverse(P * V) * viewCoord);
				//primaryRay.d = normalize(p - camera_pos);
				//// Intersect ray with scene
				//if (intersect(primaryRay))
				//{
				//	// If it hit something, evaluate the radiance from that point
				//	color += Li(primaryRay);
				//}
				//else
				//{
				//	// Otherwise evaluate environment
				//	color += Lenvironment(primaryRay.d);
				//}

				// Accumulate the obtained radiance to the pixels color
				float n = float(rendered_image.number_of_samples);
				rendered_image.data[y * rendered_image.width + x] =
					rendered_image.data[y * rendered_image.width + x] * (n / (n + 1.0f))
					+ (1.0f / (n + 1.0f)) * color;

			}
		}
		rendered_image.number_of_samples += 1;
	}
}; // namespace pathtracer
