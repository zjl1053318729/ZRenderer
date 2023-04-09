#pragma once

#include <pcg_random.hpp>
#include <random>
#include <limits>

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
		inline uint32_t UniformUInt32();
		inline uint32_t UniformUInt32(uint32_t _a, uint32_t _b);
		inline double UniformFloat64();
		inline double UniformFloat64(int _a, int _b);
	};

	inline uint32_t RNG::UniformUInt32() //生成[0, int最大值]的随机数
	{
		static std::uniform_int_distribution<unsigned> uniform_dist(0, std::numeric_limits<uint32_t>::max());
		return uniform_dist(rng);
	}
	inline uint32_t RNG::UniformUInt32(uint32_t _a, uint32_t _b) //生成[0, int最大值]的随机数
	{
		static std::uniform_int_distribution<unsigned> uniform_dist(_a, _b);
		return uniform_dist(rng);
	}
	inline double RNG::UniformFloat64() //生成[0,1]随机数
	{
		std::uniform_real_distribution<> uniform_dist(0, 1);
		return uniform_dist(rng);
	}
	inline double RNG::UniformFloat64(int _a, int _b)
	{
		std::uniform_real_distribution<> uniform_dist(_a, _b);
		return uniform_dist(rng);
	}
}