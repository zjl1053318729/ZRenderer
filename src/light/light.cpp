#include "light.hpp"
#include "../core/scene.hpp"

namespace ZR
{
	static long long numLights = 0;
	static long long numAreaLights = 0;
	// Light Method Definitions
	Light::Light(int flags, const Transform& LightToWorld, int nSamples)
			: flags(flags),
			  nSamples(std::max(1, nSamples)),
			  LightToWorld(LightToWorld),
			  WorldToLight(Inverse(LightToWorld))
	{
		++numLights;
	}


	bool VisibilityTester::Unoccluded(const Scene& scene) const
	{
		//bool hit = scene.IntersectP(p0.SpawnRayTo(p1));
		SurfaceInteraction fuck;
//		if(scene.Intersect(Ray(p0.position,(p1.position-p0.position).normalized()), &fuck))
//			std::cerr<<fuck.position.x()<<" "<<fuck.position.y()<<" "<<fuck.position.z()<<"\n";
		return !scene.IntersectP(p0.SpawnRayTo(p1));
	}

	AreaLight::AreaLight(const Transform& LightToWorld, int nSamples)
			: Light((int)LightFlags::Area, LightToWorld, nSamples)
	{
		++numAreaLights;
	}
}