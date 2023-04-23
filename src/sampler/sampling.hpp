#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "RNG.hpp"

namespace ZR
{
	class Distribution1D
	{
		// Distribution1D Public Methods
	public:
		Distribution1D(const double* f, int n);
		int Count() const
		{
			return (int)func.size();
		}
		double SampleContinuous(double u, double* pdf, int* off = nullptr) const;
		int SampleDiscrete(double u, double* pdf = nullptr,
				double* uRemapped = nullptr) const;
		double DiscretePDF(int index) const
		{
			//CHECK(index >= 0 && index < Count());
			return func[index] / (funcInt * Count());
		}

		// Distribution1D Public Data
		std::vector<double> func, cdf;
		double funcInt;
	};

	class Distribution2D
	{
	public:
		// Distribution2D Public Methods
		Distribution2D(const double* data, int nu, int nv);
		Eigen::Vector2d SampleContinuous(const Eigen::Vector2d& u, double* pdf) const;
		double Pdf(const Eigen::Vector2d& p) const;

	private:
		// Distribution2D Private Data
		std::vector<std::unique_ptr<Distribution1D>> pConditionalV;
		std::unique_ptr<Distribution1D> pMarginal;
	};

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