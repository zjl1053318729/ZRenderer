#pragma once

#include <memory>
#include "sampler.hpp"
#include "../core/geometry.hpp"

namespace ZR
{
	// HaltonSampler Declarations
	class ClockRandSampler : public GlobalSampler
	{
	public:
		// HaltonSampler Public Methods
		ClockRandSampler(int samplesPerPixel = 16,
				const Bounds2i& sampleBounds = Bounds2i(Eigen::Vector2i(0, 0), Eigen::Vector2i(100, 100)))
				: GlobalSampler(samplesPerPixel)
		{

		}
		std::unique_ptr<Sampler> Clone(int seed);
		int64_t GetIndexForSample(int64_t sampleNum) const;
		double SampleDimension(int64_t index, int dimension) const;
	};
}