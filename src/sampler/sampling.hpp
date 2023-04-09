#pragma once

#include <Eigen/Eigen>
#include "../core/ZRender.hpp"
#include "RNG.hpp"

namespace ZR
{
	template<typename T>
	void Shuffle(T* samp, int count, int nDimensions, RNG& rng)
	{
		for (int i = 0; i < count; ++i)
		{
			int other = i + rng.UniformUInt32(0, count - i);
			for (int j = 0; j < nDimensions; ++j)
				std::swap(samp[nDimensions * i + j], samp[nDimensions * other + j]);
		}
	}
	Eigen::Vector2d ConcentricSampleDisk(const Eigen::Vector2d& u)
	{
		// Map uniform random numbers to $[-1,1]^2$
		Eigen::Vector2d uOffset = 2.f * u - Eigen::Vector2d(1, 1);

		// Handle degeneracy at the origin
		if (uOffset.x() == 0 && uOffset.y() == 0) return Eigen::Vector2d(0, 0);

		// Apply concentric mapping to point
		double theta, r;
		if (std::fabs(uOffset.x()) > std::fabs(uOffset.y()))
		{
			r = uOffset.x();
			theta = PiOver4 * (uOffset.y() / uOffset.x());
		}
		else
		{
			r = uOffset.y();
			theta = PiOver2 - PiOver4 * (uOffset.x() / uOffset.y());
		}
		return r * Eigen::Vector2d(std::cos(theta), std::sin(theta));
	}
}