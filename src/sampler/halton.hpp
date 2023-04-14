#pragma once

#include "../core/ZRender.hpp"
#include "sampler.hpp"
#include "LowDiscrepancy.hpp"

namespace ZR
{
	// HaltonSampler Local Constants
	static constexpr int kMaxResolution = 128;

	// HaltonSampler Utility Functions
	static void extendedGCD(uint64_t a, uint64_t b, int64_t* x, int64_t* y);
	static uint64_t multiplicativeInverse(int64_t a, int64_t n);

	// HaltonSampler Declarations
	class HaltonSampler : public GlobalSampler
	{
	public:
		// HaltonSampler Public Methods
		HaltonSampler(int nsamp, const ZR::Bounds2i& sampleBounds,
				bool sampleAtCenter = false);
		int64_t GetIndexForSample(int64_t sampleNum) const;
		double SampleDimension(int64_t index, int dimension) const;
		std::unique_ptr<Sampler> Clone(int seed);

	private:
		// HaltonSampler Private Data
		std::vector<uint16_t> radicalInversePermutations;
		Eigen::Vector2i baseScales, baseExponents;
		int sampleStride;
		int multInverse[2];
		mutable Eigen::Vector2i pixelForOffset = Eigen::Vector2i(std::numeric_limits<int>::max(),
				std::numeric_limits<int>::max());
		mutable int64_t offsetForCurrentPixel;
		// Added after book publication: force all image samples to be at the
		// center of the pixel area.
		bool sampleAtPixelCenter;

		// HaltonSampler Private Methods
		const uint16_t* PermutationForDimension(int dim) const;
	};

	HaltonSampler* CreateHaltonSampler(const Bounds2i& sampleBounds);
}