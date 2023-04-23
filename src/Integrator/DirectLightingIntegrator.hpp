#pragma once

#include "Integrator.hpp"

namespace ZR
{
	// LightStrategy Declarations
	enum class LightStrategy
	{
		UniformSampleAll, UniformSampleOne
	};

// DirectLightingIntegrator Declarations
	class DirectLightingIntegrator : public SamplerIntegrator
	{
	public:
		// DirectLightingIntegrator Public Methods
		DirectLightingIntegrator(LightStrategy strategy, int maxDepth,
				std::shared_ptr<const Camera> camera,
				std::shared_ptr<Sampler> sampler,
				const Bounds2i& pixelBounds,
				Buffer* framebuffer)
				: SamplerIntegrator(camera, sampler, pixelBounds, framebuffer),
				  strategy(strategy),
				  maxDepth(maxDepth)
		{
		}
		Spectrum Li(const Ray& ray, const Scene& scene,
				Sampler& sampler, int depth) const;
		void Preprocess(const Scene& scene, Sampler& sampler);

	private:
		// DirectLightingIntegrator Private Data
		const LightStrategy strategy;
		const int maxDepth;
		std::vector<int> nLightSamples;
	};
}
