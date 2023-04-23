#pragma once

#include "Integrator.hpp"

namespace ZR
{
	// WhittedIntegrator Declarations
	class WhittedIntegrator : public SamplerIntegrator
	{
	public:
		// WhittedIntegrator Public Methods
		WhittedIntegrator(int maxDepth, std::shared_ptr<const Camera> camera,
				std::shared_ptr<Sampler> sampler,
				const Bounds2i& pixelBounds, Buffer* buffer)
				: SamplerIntegrator(camera, sampler, pixelBounds, buffer), maxDepth(maxDepth)
		{
		}
		Spectrum Li(const Ray& ray, const Scene& scene,
				Sampler& sampler, int depth) const;
	private:
		// WhittedIntegrator Private Data
		const int maxDepth;
	};
}