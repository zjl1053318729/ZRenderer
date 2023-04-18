#include "WhittedIntegrator.hpp"
#include "../core/interaction.hpp"
#include "../core/scene.hpp"
#include "../light/light.hpp"
#include "../material/reflection.hpp"
#include "../sampler/sampler.hpp"

namespace ZR
{
	Spectrum WhittedIntegrator::Li(const Ray& ray, const Scene& scene,
			Sampler& sampler, int depth) const
	{
		Spectrum L(0.0);
		// Find closest ray intersection or return background radiance
		SurfaceInteraction isect;
		if (!scene.Intersect(ray, &isect))
		{
			for (const auto& light: scene.lights) L += light->Le(ray);
			return L;
		}

		// Compute emitted and reflected light at ray intersection point

		// Initialize common variables for Whitted integrator
		const Eigen::Vector3d& n = isect.shading.n;
		Eigen::Vector3d wo = isect.wo;

		// Compute scattering functions for surface interaction
		isect.ComputeScatteringFunctions(ray);

		if (!isect.bsdf) return Li(isect.SpawnRay(ray.direction), scene, sampler, depth);

		// Compute emitted light if ray hit an area light source
		L += isect.Le(wo);

		// Add contribution of each light source
		for (const auto& light: scene.lights)
		{
			Eigen::Vector3d wi;
			double pdf;
			VisibilityTester visibility;
			Spectrum Li = light->Sample_Li(isect, sampler.Get2D(), &wi, &pdf, &visibility);
			if (Li.IsBlack() || pdf == 0) continue;
			Spectrum f = isect.bsdf->f(wo, wi);
			if (!f.IsBlack() && visibility.Unoccluded(scene))
				L += f * Li * std::fabs(wi.dot(n)) / pdf;
		}
		if (depth + 1 < maxDepth)
		{
			// Trace rays for specular reflection and refraction
			L += SpecularReflect(ray, isect, scene, sampler, depth);
		}
		return L;
	}
}