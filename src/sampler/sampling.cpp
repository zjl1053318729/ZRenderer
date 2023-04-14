

#include "sampling.hpp"

namespace ZR
{
	// Sampling Function Definitions
	void StratifiedSample1D(double* samp, int nSamples, RNG& rng, bool jitter)
	{
		double invNSamples = (double)1 / nSamples;
		for (int i = 0; i < nSamples; ++i)
		{
			double delta = jitter ? rng.UniformFloat64() : 0.5f;
			samp[i] = std::min((i + delta) * invNSamples, 1.0 - 1e-6);
		}
	}

	void StratifiedSample2D(Eigen::Vector2d* samp, int nx, int ny, RNG& rng, bool jitter)
	{
		double dx = (double)1 / nx, dy = (double)1 / ny;
		for (int y = 0; y < ny; ++y)
			for (int x = 0; x < nx; ++x)
			{
				double jx = jitter ? rng.UniformFloat64() : 0.5f;
				double jy = jitter ? rng.UniformFloat64() : 0.5f;
				samp->x() = std::min((x + jx) * dx, 1.0 - 1e-6);
				samp->y() = std::min((y + jy) * dy, 1.0 - 1e-6);
				++samp;
			}
	}

	void LatinHypercube(double* samples, int nSamples, int nDim, RNG& rng)
	{
		// Generate LHS samples along diagonal
		double invNSamples = (double)1 / nSamples;
		for (int i = 0; i < nSamples; ++i)
			for (int j = 0; j < nDim; ++j)
			{
				double sj = (i + (rng.UniformFloat64())) * invNSamples;
				samples[nDim * i + j] = std::min(sj, 1.0 - 1e-6);
			}

		// Permute LHS samples in each dimension
		for (int i = 0; i < nDim; ++i)
		{
			for (int j = 0; j < nSamples; ++j)
			{
				int other = j + rng.UniformUInt32(0, nSamples - j);
				std::swap(samples[nDim * j + i], samples[nDim * other + i]);
			}
		}
	}

	Eigen::Vector2d RejectionSampleDisk(RNG& rng)
	{
		Eigen::Vector2d p;
		do
		{
			p.x() = 1 - 2 * rng.UniformFloat64();
			p.y() = 1 - 2 * rng.UniformFloat64();
		} while (p.x() * p.x() + p.y() * p.y() > 1);
		return p;
	}

	Eigen::Vector3d UniformSampleHemisphere(const Eigen::Vector2d& u)
	{
		double z = u[0];
		double r = std::sqrt(std::max((double)0, (double)1. - z * z));
		double phi = 2 * Pi * u[1];
		return Eigen::Vector3d(r * std::cos(phi), r * std::sin(phi), z);
	}

	double UniformHemispherePdf()
	{
		return Inv2Pi;
	}

	Eigen::Vector3d UniformSampleSphere(const Eigen::Vector2d& u)
	{
		double z = 1 - 2 * u[0];
		double r = std::sqrt(std::max((double)0, (double)1 - z * z));
		double phi = 2 * Pi * u[1];
		return Eigen::Vector3d(r * std::cos(phi), r * std::sin(phi), z);
	}

	double UniformSpherePdf()
	{
		return Inv4Pi;
	}

	Eigen::Vector2d UniformSampleDisk(const Eigen::Vector2d& u)
	{
		double r = std::sqrt(u[0]);
		double theta = 2 * Pi * u[1];
		return Eigen::Vector2d(r * std::cos(theta), r * std::sin(theta));
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

	double UniformConePdf(double cosThetaMax)
	{
		return 1 / (2 * Pi * (1 - cosThetaMax));
	}

	Eigen::Vector3d UniformSampleCone(const Eigen::Vector2d& u, double cosThetaMax)
	{
		double cosTheta = ((double)1 - u[0]) + u[0] * cosThetaMax;
		double sinTheta = std::sqrt((double)1 - cosTheta * cosTheta);
		double phi = u[1] * 2 * Pi;
		return Eigen::Vector3d(std::cos(phi) * sinTheta, std::sin(phi) * sinTheta,
				cosTheta);
	}

	Eigen::Vector3d UniformSampleCone(const Eigen::Vector2d& u, double cosThetaMax,
			const Eigen::Vector3d& x, const Eigen::Vector3d& y,
			const Eigen::Vector3d& z)
	{
		double cosTheta = Lerp(u[0], cosThetaMax, 1.f);
		double sinTheta = std::sqrt((double)1. - cosTheta * cosTheta);
		double phi = u[1] * 2 * Pi;
		return std::cos(phi) * sinTheta * x + std::sin(phi) * sinTheta * y +
			   cosTheta * z;
	}

	Eigen::Vector2d UniformSampleTriangle(const Eigen::Vector2d& u)
	{
		double su0 = std::sqrt(u[0]);
		return Eigen::Vector2d(1 - su0, u[1] * su0);
	}
	Eigen::Vector3d CosineSampleHemisphere(const Eigen::Vector2d& u)
	{
		Eigen::Vector2d d = ConcentricSampleDisk(u);
		double z = std::sqrt(std::max((double)0, 1 - d.x() * d.x() - d.y() * d.y()));
		return Eigen::Vector3d(d.x(), d.y(), z);
	}
	double CosineHemispherePdf(double cosTheta)
	{
		return cosTheta * InvPi;
	}

	double BalanceHeuristic(int nf, double fPdf, int ng, double gPdf)
	{
		return (nf * fPdf) / (nf * fPdf + ng * gPdf);
	}

	double PowerHeuristic(int nf, double fPdf, int ng, double gPdf)
	{
		double f = nf * fPdf, g = ng * gPdf;
		return (f * f) / (f * f + g * g);
	}
}