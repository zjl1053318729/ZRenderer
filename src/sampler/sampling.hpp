#pragma once

#include <Eigen/Eigen>
#include "RNG.hpp"

namespace ZR
{
	template<typename T>
	inline void Shuffle(T* samp, int count, int nDimensions, RNG& rng)
	{
		for (int i = 0; i < count; ++i)
		{
			int other = i + rng.UniformUInt32(0, count - i);
			for (int j = 0; j < nDimensions; ++j)
				std::swap(samp[nDimensions * i + j], samp[nDimensions * other + j]);
		}
	}
	Eigen::Vector2d ConcentricSampleDisk(const Eigen::Vector2d& u);
	// Sampling Declarations
	void StratifiedSample1D(double* samples, int nsamples, RNG& rng,
			bool jitter = true);

	void StratifiedSample2D(Eigen::Vector2d* samples, int nx, int ny, RNG& rng,
			bool jitter = true);

	void LatinHypercube(double* samples, int nSamples, int nDim, RNG& rng);

	Eigen::Vector2d RejectionSampleDisk(RNG& rng);

	Eigen::Vector3d UniformSampleHemisphere(const Eigen::Vector2d& u);

	double UniformHemispherePdf();

	Eigen::Vector3d UniformSampleSphere(const Eigen::Vector2d& u);

	double UniformSpherePdf();

	Eigen::Vector3d UniformSampleCone(const Eigen::Vector2d& u, double thetamax);

	Eigen::Vector3d UniformSampleCone(const Eigen::Vector2d& u, double thetamax, const Eigen::Vector3d& x,
			const Eigen::Vector3d& y, const Eigen::Vector3d& z);

	double UniformConePdf(double thetamax);

	Eigen::Vector2d UniformSampleDisk(const Eigen::Vector2d& u);

	Eigen::Vector2d ConcentricSampleDisk(const Eigen::Vector2d& u);

	Eigen::Vector2d UniformSampleTriangle(const Eigen::Vector2d& u);

	Eigen::Vector3d CosineSampleHemisphere(const Eigen::Vector2d& u);

	double CosineHemispherePdf(double cosTheta);

	double BalanceHeuristic(int nf, double fPdf, int ng, double gPdf);

	double PowerHeuristic(int nf, double fPdf, int ng, double gPdf);
}