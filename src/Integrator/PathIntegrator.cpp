#include "PathIntegrator.hpp"
#include "../core/interaction.hpp"
#include "../core/scene.hpp"
#include "../material/reflection.hpp"
#include "../sampler/sampler.hpp"
#include "../light/light.hpp"

namespace ZR
{
	static long long totalPaths = 0;
	static long long zeroRadiancePaths = 0;
	static long long pathLength = 0;

// PathIntegrator Method Definitions
	PathIntegrator::PathIntegrator(int maxDepth,
			std::shared_ptr<const Camera> camera,
			std::shared_ptr<Sampler> sampler,
			const Bounds2i& pixelBounds, double rrThreshold,
			const std::string& lightSampleStrategy, Buffer* framebuffer)
			: SamplerIntegrator(camera, sampler, pixelBounds, framebuffer),
			  maxDepth(maxDepth),
			  rrThreshold(rrThreshold),
			  lightSampleStrategy(lightSampleStrategy)
	{
	}

	void PathIntegrator::Preprocess(const Scene& scene, Sampler& sampler)
	{
		lightDistribution =
				CreateLightSampleDistribution(lightSampleStrategy, scene);
	}

	Spectrum PathIntegrator::Li(const Ray& r, const Scene& scene,
			Sampler& sampler, int depth) const
	{
		Spectrum L(0.f), beta(1.f);
		Ray ray(r);
		bool specularBounce = false;
		int bounces;
		double etaScale = 1;

		for (bounces = 0;; ++bounces)
		{
			// Find next path vertex and accumulate contribution

			// Intersect _ray_ with scene and store intersection in _isect_
			SurfaceInteraction isect;
			bool foundIntersection = scene.Intersect(ray, &isect);

			// Possibly add emitted light at intersection
			if (bounces == 0 || specularBounce)
			{
				// Add emitted light at path vertex or from the environment
				if (foundIntersection)
				{
					L += beta * isect.Le(-ray.direction);
				}
				else
				{
					for (const auto& light: scene.infiniteLights)
						L += beta * light->Le(ray);
				}
			}

			// Terminate path if ray escaped or _maxDepth_ was reached

			if (!foundIntersection || bounces >= maxDepth) break;

			// Compute scattering functions and skip over medium boundaries

			isect.ComputeScatteringFunctions(ray, true);//boom

			if (!isect.bsdf)
			{
				ray = isect.SpawnRay(ray.direction);
				bounces--;
				continue;
			}
			const Distribution1D* distrib = lightDistribution->Lookup(isect.position);
			// Sample illumination from lights to find path contribution.
			// (But skip this for perfectly specular BSDFs.)
			if (isect.bsdf->NumComponents(BxDFType(BSDF_ALL & ~BSDF_SPECULAR)) > 0)
			{

				++totalPaths;
				Spectrum Ld = beta * UniformSampleOneLight(isect, scene, sampler, false, distrib);

				if (Ld.IsBlack()) ++zeroRadiancePaths;
				L += Ld;
			}

			// Sample BSDF to get new path direction
			Eigen::Vector3d wo = -ray.direction, wi;
			double pdf;
			BxDFType flags;
			Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf, BSDF_ALL, &flags);
			if (f.IsBlack() || pdf == 0.f) break;
			beta *= f * std::fabs(wi.dot(isect.shading.n)) / pdf;

			specularBounce = (flags & BSDF_SPECULAR) != 0;
			if ((flags & BSDF_SPECULAR) && (flags & BSDF_TRANSMISSION))
			{
				double eta = isect.bsdf->eta;
				// Update the term that tracks radiance scaling for refraction
				// depending on whether the ray is entering or leaving the
				// medium.
				etaScale *= (wo.dot(isect.normal) > 0) ? (eta * eta) : 1 / (eta * eta);
			}

			ray = isect.SpawnRay(wi);
			// Possibly terminate the path with Russian roulette.
			// Factor out radiance scaling due to refraction in rrBeta.
			Spectrum rrBeta = beta * etaScale;

			if (rrBeta.MaxComponentValue() < rrThreshold && bounces > 3)
			{
				if (sampler.Get1D() < 0.125) break;
				beta /= 0.875;
			}
		}
		return L;
	}
}