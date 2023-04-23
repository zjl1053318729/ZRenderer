#pragma once

#include "../sampler/sampling.hpp"
#include <memory>

namespace ZR
{
	// LightDistribution defines a general interface for classes that provide
// probability distributions for sampling light sources at a given point in
// space.
	class LightDistribution
	{
	public:
		virtual ~LightDistribution()
		{
		}

		// Given a point |p| in space, this method returns a (hopefully
		// effective) sampling distribution for light sources at that point.
		virtual const Distribution1D* Lookup(const Eigen::Vector3d& p) const = 0;
	};

	std::unique_ptr<LightDistribution> CreateLightSampleDistribution(
			const std::string& name, const Scene& scene);

// The simplest possible implementation of LightDistribution: this returns
// a uniform distribution over all light sources, ignoring the provided
// point. This approach works well for very simple scenes, but is quite
// ineffective for scenes with more than a handful of light sources. (This
// was the sampling method originally used for the PathIntegrator and the
// VolPathIntegrator in the printed book, though without the
// UniformLightDistribution class.)
	class UniformLightDistribution : public LightDistribution
	{
	public:
		UniformLightDistribution(const Scene& scene);
		const Distribution1D* Lookup(const Eigen::Vector3d& p) const;

	private:
		std::unique_ptr<Distribution1D> distrib;
	};
}