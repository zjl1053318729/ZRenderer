

#include "halton.hpp"

namespace ZR
{
	static void extendedGCD(uint64_t a, uint64_t b, int64_t* x, int64_t* y)
	{
		if (b == 0)
		{
			*x = 1;
			*y = 0;
			return;
		}
		int64_t d = a / b, xp, yp;
		extendedGCD(b, a % b, &xp, &yp);
		*x = yp;
		*y = xp - (d * yp);
	}
	static uint64_t multiplicativeInverse(int64_t a, int64_t n)
	{
		int64_t x, y;
		extendedGCD(a, n, &x, &y);
		return Mod(x, n);
	}
	HaltonSampler::HaltonSampler(int nsamp, const Bounds2i& sampleBounds, bool sampleAtCenter)
			: GlobalSampler(samplesPerPixel), sampleAtPixelCenter(sampleAtPixelCenter)
	{
		// Generate random digit permutations for Halton sampler
		if (radicalInversePermutations.empty())
		{
			RNG rng;
			radicalInversePermutations = ComputeRadicalInversePermutations(rng);
		}

		// Find radical inverse base scales and exponents that cover sampling area
		Eigen::Vector2i res = sampleBounds.pMax - sampleBounds.pMin;
		for (int i = 0; i < 2; ++i)
		{
			int base = (i == 0) ? 2 : 3;
			int scale = 1, exp = 0;
			while (scale < std::min(res[i], kMaxResolution))
			{
				scale *= base;
				++exp;
			}
			baseScales[i] = scale;
			baseExponents[i] = exp;
		}

		// Compute stride in samples for visiting each pixel area
		sampleStride = baseScales[0] * baseScales[1];

		// Compute multiplicative inverses for _baseScales_
		multInverse[0] = multiplicativeInverse(baseScales[1], baseScales[0]);
		multInverse[1] = multiplicativeInverse(baseScales[0], baseScales[1]);
	}
	int64_t HaltonSampler::GetIndexForSample(int64_t sampleNum) const
	{
		if (currentPixel != pixelForOffset)
		{
			// Compute Halton sample offset for _currentPixel_
			offsetForCurrentPixel = 0;
			if (sampleStride > 1)
			{
				Eigen::Vector2i pm(Mod(currentPixel[0], kMaxResolution),
						Mod(currentPixel[1], kMaxResolution));
				for (int i = 0; i < 2; ++i)
				{
					uint64_t dimOffset =
							(i == 0)
							? InverseRadicalInverse<2>(pm[i], baseExponents[i])
							: InverseRadicalInverse<3>(pm[i], baseExponents[i]);
					offsetForCurrentPixel +=
							dimOffset * (sampleStride / baseScales[i]) * multInverse[i];
				}
				offsetForCurrentPixel %= sampleStride;
			}
			pixelForOffset = currentPixel;
		}
		return offsetForCurrentPixel + sampleNum * sampleStride;
	}
	double HaltonSampler::SampleDimension(int64_t index, int dim) const
	{
		if (sampleAtPixelCenter && (dim == 0 || dim == 1)) return 0.5f;
		if (dim == 0)
			return RadicalInverse(dim, index >> baseExponents[0]);
		else if (dim == 1)
			return RadicalInverse(dim, index / baseScales[1]);
		else
			return ScrambledRadicalInverse(dim, index,
					PermutationForDimension(dim));
	}
	std::unique_ptr<Sampler> HaltonSampler::Clone(int seed)
	{
		return std::unique_ptr<Sampler>(new HaltonSampler(*this));
	}
	const uint16_t* HaltonSampler::PermutationForDimension(int dim) const
	{
		return &radicalInversePermutations[PrimeSums[dim]];
	}
	HaltonSampler* CreateHaltonSampler(const Bounds2i& sampleBounds)
	{
		int nsamp = 16;
		//if (PbrtOptions.quickRender) nsamp = 1;
		bool sampleAtCenter = false;

		return new HaltonSampler(nsamp, sampleBounds, sampleAtCenter);
	}
}