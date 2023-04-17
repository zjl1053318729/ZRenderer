#pragma once

#include <Eigen/Eigen>
#include "../core/ZRender.hpp"
#include "../core/interaction.hpp"
#include "../core/spectrum.hpp"
#include "Fresnel.hpp"

namespace ZR
{
	class SurfaceInteraction;


	double CosTheta(const Eigen::Vector3d& w);
	double Cos2Theta(const Eigen::Vector3d& w);
	double AbsCosTheta(const Eigen::Vector3d& w);
	double Sin2Theta(const Eigen::Vector3d& w);
	double SinTheta(const Eigen::Vector3d& w);
	double TanTheta(const Eigen::Vector3d& w);
	double Tan2Theta(const Eigen::Vector3d& w);

	double CosPhi(const Eigen::Vector3d& w);
	double SinPhi(const Eigen::Vector3d& w);

	double Cos2Phi(const Eigen::Vector3d& w);
	double Sin2Phi(const Eigen::Vector3d& w);

	double CosDPhi(const Eigen::Vector3d& wa, const Eigen::Vector3d& wb);

	Eigen::Vector3d Reflect(const Eigen::Vector3d& wo, const Eigen::Vector3d& n);

	bool Refract(const Eigen::Vector3d& wi, const Eigen::Vector3d& n, double eta,
			Eigen::Vector3d* wt);

	bool SameHemisphere(const Eigen::Vector3d& w, const Eigen::Vector3d& wp);

	enum BxDFType
	{
		BSDF_REFLECTION = 1 << 0,
		BSDF_TRANSMISSION = 1 << 1,
		BSDF_DIFFUSE = 1 << 2,
		BSDF_GLOSSY = 1 << 3,
		BSDF_SPECULAR = 1 << 4,
		BSDF_ALL = BSDF_DIFFUSE | BSDF_GLOSSY | BSDF_SPECULAR | BSDF_REFLECTION |
				   BSDF_TRANSMISSION,
	};

	// BxDF Declarations
	class BxDF
	{
	public:
		// BxDF Interface
		virtual ~BxDF()
		{
		}
		BxDF(BxDFType type) : type(type)
		{
		}
		bool MatchesFlags(BxDFType t) const
		{
			return (type & t) == type;
		}
		virtual Spectrum f(const Eigen::Vector3d& wo, const Eigen::Vector3d& wi) const = 0;
		virtual Spectrum Sample_f(const Eigen::Vector3d& wo, Eigen::Vector3d* wi,
				const Eigen::Vector2d& sample, double* pdf, BxDFType* sampledType = nullptr) const;
		virtual Spectrum rho(const Eigen::Vector3d& wo, int nSamples, const Eigen::Vector2d* samples) const;
		virtual Spectrum rho(int nSamples, const Eigen::Vector2d* samples1, const Eigen::Vector2d* samples2) const;
		virtual double Pdf(const Eigen::Vector3d& wo, const Eigen::Vector3d& wi) const;

		// BxDF Public Data
		const BxDFType type;
	};

	// BSDF Declarations
	class BSDF
	{
	public:
		// BSDF Public Methods
		BSDF(const SurfaceInteraction& si, double eta = 1)
				: eta(eta),
				  ns(si.shading.n),
				  ng(si.normal),
				  ss(si.shading.dpdu.normalized()),
				  ts(ns.cross(ss))
		{
		}
		void Add(BxDF* b)
		{
			bxdfs[nBxDFs++] = b;
		}
		int NumComponents(BxDFType flags = BSDF_ALL) const;
		Eigen::Vector3d WorldToLocal(const Eigen::Vector3d& v) const;
		Eigen::Vector3d LocalToWorld(const Eigen::Vector3d& v) const;
		Spectrum f(const Eigen::Vector3d& woW, const Eigen::Vector3d& wiW,
				BxDFType flags = BSDF_ALL) const;
		Spectrum rho(int nSamples, const Eigen::Vector2d* samples1, const Eigen::Vector2d* samples2,
				BxDFType flags = BSDF_ALL) const;
		Spectrum rho(const Eigen::Vector3d& wo, int nSamples, const Eigen::Vector2d* samples,
				BxDFType flags = BSDF_ALL) const;
		Spectrum Sample_f(const Eigen::Vector3d& wo, Eigen::Vector3d* wi, const Eigen::Vector2d& u,
				double* pdf, BxDFType type = BSDF_ALL,
				BxDFType* sampledType = nullptr) const;
		double Pdf(const Eigen::Vector3d& wo, const Eigen::Vector3d& wi,
				BxDFType flags = BSDF_ALL) const;

		// BSDF Public Data
		const double eta;
		// BSDF Private Methods
		~BSDF();
	private:
		// BSDF Private Data
		const Eigen::Vector3d ns, ng;
		const Eigen::Vector3d ss, ts;
		int nBxDFs = 0;
		static constexpr int MaxBxDFs = 8;
		BxDF* bxdfs[MaxBxDFs];
	};


	class LambertianReflection : public BxDF
	{
	public:
		// LambertianReflection Public Methods
		LambertianReflection(const Spectrum& R)
				: BxDF(BxDFType(BSDF_REFLECTION | BSDF_DIFFUSE)), R(R)
		{
		}
		Spectrum f(const Eigen::Vector3d& wo, const Eigen::Vector3d& wi) const;
		Spectrum rho(const Eigen::Vector3d&, int, const Eigen::Vector2d*) const;
		Spectrum rho(int, const Eigen::Vector2d*, const Eigen::Vector2d*) const;
	private:
		// LambertianReflection Private Data
		const Spectrum R;
	};

	class SpecularReflection : public BxDF
	{
	public:
		// SpecularReflection Public Methods
		SpecularReflection(const Spectrum& R, Fresnel* fresnel)
				: BxDF(BxDFType(BSDF_REFLECTION | BSDF_SPECULAR)),
				  R(R),
				  fresnel(fresnel)
		{
		}
		~SpecularReflection()
		{
			fresnel->~Fresnel();
		}
		virtual Spectrum f(const Eigen::Vector3d& wo, const Eigen::Vector3d& wi) const
		{
			return Spectrum(0.f);
		}
		virtual Spectrum Sample_f(const Eigen::Vector3d& wo, Eigen::Vector3d* wi, const Eigen::Vector2d& sample,
				double* pdf, BxDFType* sampledType) const;
		double Pdf(const Eigen::Vector3d& wo, const Eigen::Vector3d& wi) const
		{
			return 0;
		}

	private:
		// SpecularReflection Private Data
		const Spectrum R;
		const Fresnel* fresnel;
	};

}