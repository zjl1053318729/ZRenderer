#pragma once

#include "../core/ZRender.hpp"

namespace ZR
{
	class RNG
	{
	public:
		pcg32 rng;

		RNG()
		{
			pcg_extras::seed_seq_from<std::random_device> seed_source;
			rng.seed(seed_source);
		}
		uint32_t UniformUInt32();
		uint32_t UniformUInt32(uint32_t _a, uint32_t _b);
		double UniformFloat64();
		double UniformFloat64(int _a, int _b);
	};


}