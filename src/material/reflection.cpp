

#include "../core/spectrum.hpp"
#include "reflection.hpp"
#include "../sampler/sampling.hpp"

namespace ZR
{
	double CosTheta(const Eigen::Vector3d& w)
	{
		return w.z();
	}
	double Cos2Theta(const Eigen::Vector3d& w)
	{
		return w.z() * w.z();
	}
	double AbsCosTheta(const Eigen::Vector3d& w)
	{
		return std::fabs(w.z());
	}
	double Sin2Theta(const Eigen::Vector3d& w)
	{
		return std::max((double)0, (double)1 - Cos2Theta(w));
	}
	double SinTheta(const Eigen::Vector3d& w)
	{
		return std::sqrt(Sin2Theta(w));
	}
	double TanTheta(const Eigen::Vector3d& w)
	{
		return SinTheta(w) / CosTheta(w);
	}
	double Tan2Theta(const Eigen::Vector3d& w)
	{
		return Sin2Theta(w) / Cos2Theta(w);
	}

	double CosPhi(const Eigen::Vector3d& w)
	{
		double sinTheta = SinTheta(w);
		return (sinTheta == 0) ? 1 : Clamp(w.x() / sinTheta, -1, 1);
	}
	double SinPhi(const Eigen::Vector3d& w)
	{
		double sinTheta = SinTheta(w);
		return (sinTheta == 0) ? 0 : Clamp(w.y() / sinTheta, -1, 1);
	}

	double Cos2Phi(const Eigen::Vector3d& w)
	{
		return CosPhi(w) * CosPhi(w);
	}
	double Sin2Phi(const Eigen::Vector3d& w)
	{
		return SinPhi(w) * SinPhi(w);
	}

	double CosDPhi(const Eigen::Vector3d& wa, const Eigen::Vector3d& wb)
	{
		double waxy = wa.x() * wa.x() + wa.y() * wa.y();
		double wbxy = wb.x() * wb.x() + wb.y() * wb.y();
		if (waxy == 0 || wbxy == 0)
			return 1;
		return Clamp((wa.x() * wb.x() + wa.y() * wb.y()) / std::sqrt(waxy * wbxy), -1, 1);
	}

	Eigen::Vector3d Reflect(const Eigen::Vector3d& wo, const Eigen::Vector3d& n)
	{
		return -wo + 2 * wo.dot(n) * n;
	}

	bool Refract(const Eigen::Vector3d& wi, const Eigen::Vector3d& n, double eta,
			Eigen::Vector3d* wt)
	{
		// Compute $\cos \theta_\roman{t}$ using Snell's law
		double cosThetaI = n.dot(wi);
		double sin2ThetaI = std::max(double(0), double(1 - cosThetaI * cosThetaI));
		double sin2ThetaT = eta * eta * sin2ThetaI;

		// Handle total internal reflection for transmission
		if (sin2ThetaT >= 1) return false;
		double cosThetaT = std::sqrt(1 - sin2ThetaT);
		*wt = eta * -wi + (eta * cosThetaI - cosThetaT) * Eigen::Vector3d(n);
		return true;
	}

	bool SameHemisphere(const Eigen::Vector3d& w, const Eigen::Vector3d& wp)
	{
		return w.z() * wp.z() > 0;
	}
	Spectrum BxDF::rho(const Eigen::Vector3d& wo, int nSamples, const Eigen::Vector2d* samples) const
	{
		Spectrum r(0.0);
		for (int i = 0; i < nSamples; ++i)
		{
			// Estimate one term of $\rho_\roman{hd}$
			Eigen::Vector3d wi;
			double pdf = 0;
			Spectrum f = Sample_f(wo, &wi, samples[i], &pdf);
			if (pdf > 0) r += f * AbsCosTheta(wi) / pdf;
		}
		return r / nSamples;
	}
	Spectrum BxDF::rho(int nSamples, const Eigen::Vector2d* samples1, const Eigen::Vector2d* samples2) const
	{
		Spectrum r(0.0);
		for (int i = 0; i < nSamples; ++i)
		{
			// Estimate one term of $\rho_\roman{hh}$
			Eigen::Vector3d wo, wi;
			wo = UniformSampleHemisphere(samples1[i]);
			double pdfo = UniformHemispherePdf(), pdfi = 0;
			Spectrum f = Sample_f(wo, &wi, samples2[i], &pdfi);
			if (pdfi > 0)
				r += f * AbsCosTheta(wi) * AbsCosTheta(wo) / (pdfo * pdfi);
		}
		return r / (Pi * nSamples);
	}
	Spectrum BxDF::Sample_f(const Eigen::Vector3d& wo, Eigen::Vector3d* wi, const Eigen::Vector2d& sample, double* pdf,
			BxDFType* sampledType) const
	{
		// Cosine-sample the hemisphere, flipping the direction if necessary
		*wi = CosineSampleHemisphere(sample);
		if (wo.z() < 0) wi->z() *= -1;
		*pdf = Pdf(wo, *wi);
		return f(wo, *wi);
	}
	double BxDF::Pdf(const Eigen::Vector3d& wo, const Eigen::Vector3d& wi) const
	{
		return SameHemisphere(wo, wi) ? AbsCosTheta(wi) * InvPi : 0;
	}
	Eigen::Vector3d BSDF::WorldToLocal(const Eigen::Vector3d& v) const
	{
		return Eigen::Vector3d(v.dot(ss), v.dot(ts), v.dot(ns));
	}
	Eigen::Vector3d BSDF::LocalToWorld(const Eigen::Vector3d& v) const
	{
		return Eigen::Vector3d(ss.x() * v.x() + ts.x() * v.y() + ns.x() * v.z(),
				ss.y() * v.x() + ts.y() * v.y() + ns.y() * v.z(),
				ss.z() * v.x() + ts.z() * v.y() + ns.z() * v.z());
	}
	int BSDF::NumComponents(BxDFType flags) const
	{
		int num = 0;
		for (int i = 0; i < nBxDFs; ++i)
			if (bxdfs[i]->MatchesFlags(flags)) ++num;
		return num;
	}
	Spectrum BSDF::f(const Eigen::Vector3d& woW, const Eigen::Vector3d& wiW,
			BxDFType flags) const
	{
		Eigen::Vector3d wi = WorldToLocal(wiW), wo = WorldToLocal(woW);
		if (wo.z() == 0) return 0.;
		bool reflect = wiW.dot(ng) * woW.dot(ng) > 0;
		Spectrum f(0.f);
		for (int i = 0; i < nBxDFs; ++i)
			if (bxdfs[i]->MatchesFlags(flags) &&
				((reflect && (bxdfs[i]->type & BSDF_REFLECTION)) ||
				 (!reflect && (bxdfs[i]->type & BSDF_TRANSMISSION))))
				f += bxdfs[i]->f(wo, wi);
		return f;
	}
	Spectrum BSDF::rho(int nSamples, const Eigen::Vector2d* samples1, const Eigen::Vector2d* samples2,
			BxDFType flags) const
	{
		Spectrum ret(0.0);
		for (int i = 0; i < nBxDFs; ++i)
			if (bxdfs[i]->MatchesFlags(flags))
				ret += bxdfs[i]->rho(nSamples, samples1, samples2);
		return ret;
	}
	Spectrum
	BSDF::rho(const Eigen::Vector3d& woWorld, int nSamples, const Eigen::Vector2d* samples, BxDFType flags) const
	{
		Eigen::Vector3d wo = WorldToLocal(woWorld);
		Spectrum ret(0.f);
		for (int i = 0; i < nBxDFs; ++i)
			if (bxdfs[i]->MatchesFlags(flags))
				ret += bxdfs[i]->rho(wo, nSamples, samples);
		return ret;
	}
	Spectrum
	BSDF::Sample_f(const Eigen::Vector3d& woWorld, Eigen::Vector3d* wiWorld, const Eigen::Vector2d& u, double* pdf,
			BxDFType type,
			BxDFType* sampledType) const
	{
		// Choose which _BxDF_ to sample
		int matchingComps = NumComponents(type);
		if (matchingComps == 0)
		{
			*pdf = 0;
			if (sampledType) *sampledType = BxDFType(0);
			return Spectrum(0);
		}
		int comp =
				std::min((int)std::floor(u[0] * matchingComps), matchingComps - 1);

		// Get _BxDF_ pointer for chosen component
		BxDF* bxdf = nullptr;
		int count = comp;
		for (int i = 0; i < nBxDFs; ++i)
			if (bxdfs[i]->MatchesFlags(type) && count-- == 0)
			{
				bxdf = bxdfs[i];
				break;
			}
		//CHECK(bxdf != nullptr);

		// Remap _BxDF_ sample _u_ to $[0,1)^2$
		Eigen::Vector2d uRemapped(std::min(u[0] * matchingComps - comp, 1.0 - 1e-6),
				u[1]);

		// Sample chosen _BxDF_
		Eigen::Vector3d wi, wo = WorldToLocal(woWorld);
		if (wo.z() == 0) return 0.;
		*pdf = 0;
		if (sampledType) *sampledType = bxdf->type;
		Spectrum f = bxdf->Sample_f(wo, &wi, uRemapped, pdf, sampledType);
		if (*pdf == 0)
		{
			if (sampledType) *sampledType = BxDFType(0);
			return 0;
		}
		*wiWorld = LocalToWorld(wi);

		// Compute overall PDF with all matching _BxDF_s
		if (!(bxdf->type & BSDF_SPECULAR) && matchingComps > 1)
			for (int i = 0; i < nBxDFs; ++i)
				if (bxdfs[i] != bxdf && bxdfs[i]->MatchesFlags(type))
					*pdf += bxdfs[i]->Pdf(wo, wi);
		if (matchingComps > 1) *pdf /= matchingComps;

		// Compute value of BSDF for sampled direction
		if (!(bxdf->type & BSDF_SPECULAR))
		{
			bool reflect = wiWorld->dot(ng) * woWorld.dot(ng) > 0;
			f = 0.;
			for (int i = 0; i < nBxDFs; ++i)
				if (bxdfs[i]->MatchesFlags(type) &&
					((reflect && (bxdfs[i]->type & BSDF_REFLECTION)) ||
					 (!reflect && (bxdfs[i]->type & BSDF_TRANSMISSION))))
					f += bxdfs[i]->f(wo, wi);
		}
		return f;
	}
	double BSDF::Pdf(const Eigen::Vector3d& woWorld, const Eigen::Vector3d& wiWorld, BxDFType flags) const
	{
		if (nBxDFs == 0) return 0.0;
		Eigen::Vector3d wo = WorldToLocal(woWorld), wi = WorldToLocal(wiWorld);
		if (wo.z() == 0) return 0.;
		double pdf = 0.f;
		int matchingComps = 0;
		for (int i = 0; i < nBxDFs; ++i)
			if (bxdfs[i]->MatchesFlags(flags))
			{
				++matchingComps;
				pdf += bxdfs[i]->Pdf(wo, wi);
			}
		double v = matchingComps > 0 ? pdf / matchingComps : 0.0;
		return v;
	}
	BSDF::~BSDF()
	{
		for (int i = 0; i < nBxDFs; i++)
			bxdfs[i]->~BxDF();
	}
	Spectrum LambertianReflection::f(const Eigen::Vector3d& wo, const Eigen::Vector3d& wi) const
	{
		return R * InvPi;
	}
	Spectrum LambertianReflection::rho(const Eigen::Vector3d&, int, const Eigen::Vector2d*) const
	{
		return R;
	}
	Spectrum LambertianReflection::rho(int, const Eigen::Vector2d*, const Eigen::Vector2d*) const
	{
		return R;
	}
}