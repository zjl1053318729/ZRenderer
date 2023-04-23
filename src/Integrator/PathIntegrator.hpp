#pragma once

#include "Integrator.hpp"
#include "../light/LightDistribution.hpp"

namespace ZR
{
	// PathIntegrator Declarations
	class PathIntegrator : public SamplerIntegrator
	{
	public:
		// PathIntegrator Public Methods
		PathIntegrator(int maxDepth, std::shared_ptr<const Camera> camera,
				std::shared_ptr<Sampler> sampler,
				const Bounds2i& pixelBounds, double rrThreshold = 1,
				const std::string& lightSampleStrategy = "spatial",
				ZR::Buffer* framebuffer = nullptr);

		void Preprocess(const Scene& scene, Sampler& sampler);
		Spectrum Li(const Ray& ray, const Scene& scene,
				Sampler& sampler, int depth) const;

	private:
		// PathIntegrator Private Data
		const int maxDepth;
		const double rrThreshold;
		const std::string lightSampleStrategy;
		std::unique_ptr<LightDistribution> lightDistribution;
	};
}