#pragma once

#include "../core/interaction.hpp"
#include "../core/spectrum.hpp"

namespace ZR
{
	class VisibilityTester
	{
	public:
		VisibilityTester()
		{
		}
		// VisibilityTester Public Methods
		VisibilityTester(const Interaction& p0, const Interaction& p1)
				: p0(p0), p1(p1)
		{
		}
		const Interaction& P0() const
		{
			return p0;
		}
		const Interaction& P1() const
		{
			return p1;
		}
		bool Unoccluded(const Scene& scene) const;
		Spectrum Tr(const Scene& scene, Sampler& sampler) const
		{
			return Spectrum(0.f);
		}

	private:
		Interaction p0, p1;
	};

	// LightFlags Declarations
	enum class LightFlags : int
	{
		DeltaPosition = 1,
		DeltaDirection = 2,
		Area = 4,
		Infinite = 8
	};

	inline bool IsDeltaLight(int flags)
	{
		return flags & (int)LightFlags::DeltaPosition ||
			   flags & (int)LightFlags::DeltaDirection;
	}

// Light Declarations
	class Light
	{
	public:
		// Light Interface
		virtual ~Light()
		{
		}
		Light(int flags, const Transform& LightToWorld, int nSamples = 1);
		virtual Spectrum Sample_Li(const Interaction& ref, const Eigen::Vector2d& u,
				Eigen::Vector3d* wi, double* pdf,
				VisibilityTester* vis) const = 0;
		virtual Spectrum Power() const = 0;
		virtual void Preprocess(const Scene& scene)
		{
		}
		virtual Spectrum Le(const Ray& r) const
		{
			return Spectrum(0.f);
		}
		virtual double Pdf_Li(const Interaction& ref, const Eigen::Vector3d& wi) const = 0;
		virtual Spectrum Sample_Le(const Eigen::Vector2d& u1, const Eigen::Vector2d& u2, double time,
				Ray* ray, Eigen::Vector3d* nLight, double* pdfPos,
				double* pdfDir) const = 0;
		virtual void Pdf_Le(const Ray& ray, const Eigen::Vector3d& nLight, double* pdfPos,
				double* pdfDir) const = 0;
		virtual void generatePhoton(Eigen::Vector3d& ori, Eigen::Vector3d& dir, double& powScale) = 0;

		// Light Public Data
		const int flags;
		const int nSamples;
	protected:
		// Light Protected Data
		const Transform LightToWorld, WorldToLight;
	};

	class AreaLight : public Light
	{
	public:
		// AreaLight Interface
		AreaLight(const Transform& LightToWorld, int nSamples);
		virtual Spectrum L(const Interaction& intr, const Eigen::Vector3d& w) const = 0;
	};
}