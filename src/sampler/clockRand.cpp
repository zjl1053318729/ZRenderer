

#include "clockRand.hpp"

namespace ZR
{
	std::unique_ptr<Sampler> ClockRandSampler::Clone(int seed)
	{
		return std::unique_ptr<Sampler>(new ClockRandSampler(*this));
	}
	int64_t ClockRandSampler::GetIndexForSample(int64_t sampleNum) const
	{
		return 0;
	}
	double ClockRandSampler::SampleDimension(int64_t index, int dimension) const
	{
		return ZR::random_double();
	}
}