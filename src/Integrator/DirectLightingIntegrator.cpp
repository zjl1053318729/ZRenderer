#include "DirectLightingIntegrator.hpp"
#include "../core/scene.hpp"
#include "../sampler/sampler.hpp"
#include "../light/light.hpp"

namespace ZR
{
	// DirectLightingIntegrator Method Definitions
	void DirectLightingIntegrator::Preprocess(const Scene& scene,
			Sampler& sampler)
	{
		if (strategy == LightStrategy::UniformSampleAll)
		{
			// Compute number of samples to use for each light
			for (const auto& light: scene.lights)
				nLightSamples.push_back(sampler.RoundCount(light->nSamples));

			// Request samples for sampling all lights
			for (int i = 0; i < maxDepth; ++i)
			{
				for (size_t j = 0; j < scene.lights.size(); ++j)
				{
					sampler.Request2DArray(nLightSamples[j]);
					sampler.Request2DArray(nLightSamples[j]);
				}
			}
		}
	}

	Spectrum DirectLightingIntegrator::Li(const Ray& ray,
			const Scene& scene, Sampler& sampler, int depth) const
	{
		Spectrum L(0.f);
		// Find closest ray intersection or return background radiance
		SurfaceInteraction isect;
		if (!scene.Intersect(ray, &isect))
		{
			for (const auto& light: scene.lights) L += light->Le(ray);
			return L;
		}

		// Compute scattering functions for surface interaction
		isect.ComputeScatteringFunctions(ray);
		if (!isect.bsdf)
			return Li(isect.SpawnRay(ray.direction), scene, sampler, depth);
		Eigen::Vector3d wo = isect.wo;
		// Compute emitted light if ray hit an area light source
		L += isect.Le(wo);
		if (scene.lights.size() > 0)
		{
			// Compute direct lighting for _DirectLightingIntegrator_ integrator
			if (strategy == LightStrategy::UniformSampleAll)
				L += UniformSampleAllLights(isect, scene, sampler, nLightSamples);
			else
				L += UniformSampleOneLight(isect, scene, sampler);
		}
		if (depth + 1 < maxDepth)
		{
			// Trace rays for specular reflection and refraction
			L += SpecularReflect(ray, isect, scene, sampler, depth);
			//L += SpecularTransmit(ray, isect, scene, sampler, depth);
		}
		return L;
	}
}