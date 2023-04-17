#pragma once

#include "../core/spectrum.hpp"

namespace ZR
{
	// Reflection Declarations
	double FrDielectric(double cosThetaI, double etaI, double etaT);
	Spectrum FrConductor(double cosThetaI, const Spectrum& etaI,
			const Spectrum& etaT, const Spectrum& k);

	class Fresnel
	{
	public:
		// Fresnel Interface
		virtual ~Fresnel()
		{
		}
		virtual Spectrum Evaluate(double cosI) const = 0;
	};

	class FresnelConductor : public Fresnel
	{
	public:
		// FresnelConductor Public Methods
		Spectrum Evaluate(double cosThetaI) const;
		FresnelConductor(const Spectrum& etaI, const Spectrum& etaT,
				const Spectrum& k)
				: etaI(etaI), etaT(etaT), k(k)
		{
		}

	private:
		Spectrum etaI, etaT, k;
	};

	class FresnelDielectric : public Fresnel
	{
	public:
		// FresnelDielectric Public Methods
		Spectrum Evaluate(double cosThetaI) const;
		FresnelDielectric(double etaI, double etaT) : etaI(etaI), etaT(etaT)
		{
		}

	private:
		double etaI, etaT;
	};

	class FresnelNoOp : public Fresnel
	{
	public:
		Spectrum Evaluate(double) const
		{
			return Spectrum(1.);
		}
	};
}