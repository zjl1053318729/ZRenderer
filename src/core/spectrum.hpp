#pragma once

#include "ZRender.hpp"

namespace ZR
{
	void XYZToRGB(const double xyz[3], double rgb[3]);
	void RGBToXYZ(const double rgb[3], double xyz[3]);
	enum class SpectrumType
	{
		Reflectance, Illuminant
	};

	// Spectrum Declarations
	template<int nSpectrumSamples>
	class CoefficientSpectrum
	{
	public:
		// CoefficientSpectrum Public Methods
		CoefficientSpectrum(double v = 0.f)
		{
			for (int i = 0; i < nSpectrumSamples; ++i) c[i] = v;
		}

		CoefficientSpectrum& operator+=(const CoefficientSpectrum& s2)
		{
			for (int i = 0; i < nSpectrumSamples; ++i) c[i] += s2.c[i];
			return *this;
		}
		CoefficientSpectrum operator+(const CoefficientSpectrum& s2) const
		{
			CoefficientSpectrum ret = *this;
			for (int i = 0; i < nSpectrumSamples; ++i) ret.c[i] += s2.c[i];
			return ret;
		}
		CoefficientSpectrum operator-(const CoefficientSpectrum& s2) const
		{
			CoefficientSpectrum ret = *this;
			for (int i = 0; i < nSpectrumSamples; ++i) ret.c[i] -= s2.c[i];
			return ret;
		}
		CoefficientSpectrum operator/(const CoefficientSpectrum& s2) const
		{
			CoefficientSpectrum ret = *this;
			for (int i = 0; i < nSpectrumSamples; ++i)
			{
				ret.c[i] /= s2.c[i];
			}
			return ret;
		}
		CoefficientSpectrum operator*(const CoefficientSpectrum& sp) const
		{
			CoefficientSpectrum ret = *this;
			for (int i = 0; i < nSpectrumSamples; ++i) ret.c[i] *= sp.c[i];
			return ret;
		}
		CoefficientSpectrum& operator*=(const CoefficientSpectrum& sp)
		{
			for (int i = 0; i < nSpectrumSamples; ++i) c[i] *= sp.c[i];
			return *this;
		}
		CoefficientSpectrum operator*(double a) const
		{
			CoefficientSpectrum ret = *this;
			for (int i = 0; i < nSpectrumSamples; ++i) ret.c[i] *= a;
			return ret;
		}
		CoefficientSpectrum& operator*=(double a)
		{
			for (int i = 0; i < nSpectrumSamples; ++i) c[i] *= a;
			return *this;
		}
		friend CoefficientSpectrum operator*(double a,
				const CoefficientSpectrum& s)
		{
			return s * a;
		}
		CoefficientSpectrum operator/(double a) const
		{
			CoefficientSpectrum ret = *this;
			for (int i = 0; i < nSpectrumSamples; ++i) ret.c[i] /= a;
			return ret;
		}
		CoefficientSpectrum& operator/=(double a)
		{
			for (int i = 0; i < nSpectrumSamples; ++i) c[i] /= a;
			return *this;
		}
		bool operator==(const CoefficientSpectrum& sp) const
		{
			for (int i = 0; i < nSpectrumSamples; ++i)
				if (c[i] != sp.c[i]) return false;
			return true;
		}
		bool operator!=(const CoefficientSpectrum& sp) const
		{
			return !(*this == sp);
		}
		bool IsBlack() const
		{
			for (int i = 0; i < nSpectrumSamples; ++i)
				if (c[i] != 0.) return false;
			return true;
		}
		friend CoefficientSpectrum Sqrt(const CoefficientSpectrum& s)
		{
			CoefficientSpectrum ret;
			for (int i = 0; i < nSpectrumSamples; ++i) ret.c[i] = std::sqrt(s.c[i]);
			return ret;
		}
		template<int n>
		friend CoefficientSpectrum<n> Pow(const CoefficientSpectrum<n>& s,
				double e);
		CoefficientSpectrum operator-() const
		{
			CoefficientSpectrum ret;
			for (int i = 0; i < nSpectrumSamples; ++i) ret.c[i] = -c[i];
			return ret;
		}
		friend CoefficientSpectrum Exp(const CoefficientSpectrum& s)
		{
			CoefficientSpectrum ret;
			for (int i = 0; i < nSpectrumSamples; ++i) ret.c[i] = std::exp(s.c[i]);
			return ret;
		}
		friend std::ostream& operator<<(std::ostream& os,
				const CoefficientSpectrum& s)
		{
			return os << s.ToString();
		}
		std::string ToString() const
		{
			std::string str = "[ ";
			for (int i = 0; i < nSpectrumSamples; ++i)
			{
				str += StringPrintf("%f", c[i]);
				if (i + 1 < nSpectrumSamples) str += ", ";
			}
			str += " ]";
			return str;
		}
		CoefficientSpectrum Clamp(double low = 0, double high = ZR::Infinity) const
		{
			CoefficientSpectrum ret;
			for (int i = 0; i < nSpectrumSamples; ++i)
				ret.c[i] = ZR::Clamp(c[i], low, high);
			return ret;
		}
		double MaxComponentValue() const
		{
			double m = c[0];
			for (int i = 1; i < nSpectrumSamples; ++i)
				m = std::max(m, c[i]);
			return m;
		}
		bool HasNaNs() const
		{
			for (int i = 0; i < nSpectrumSamples; ++i)
				if (std::isnan(c[i])) return true;
			return false;
		}
		double& operator[](int i)
		{
			return c[i];
		}
		double operator[](int i) const
		{
			return c[i];
		}

		// CoefficientSpectrum Public Data
		static const int nSamples = nSpectrumSamples;

	protected:
		// CoefficientSpectrum Protected Data
		double c[nSpectrumSamples];
	};


	class RGBSpectrum : public CoefficientSpectrum<3>
	{
		using CoefficientSpectrum<3>::c;

	public:
		// RGBSpectrum Public Methods
		RGBSpectrum(double v = 0.f) : CoefficientSpectrum<3>(v)
		{
		}
		RGBSpectrum(const CoefficientSpectrum<3>& v) : CoefficientSpectrum<3>(v)
		{
		}
		RGBSpectrum(const RGBSpectrum& s,
				SpectrumType type = SpectrumType::Reflectance)
		{
			*this = s;
		}
		static RGBSpectrum FromRGB(const double rgb[3],
				SpectrumType type = SpectrumType::Reflectance)
		{
			RGBSpectrum s;
			s.c[0] = rgb[0];
			s.c[1] = rgb[1];
			s.c[2] = rgb[2];
			return s;
		}
		void ToRGB(double* rgb) const
		{
			rgb[0] = c[0];
			rgb[1] = c[1];
			rgb[2] = c[2];
		}
		const RGBSpectrum& ToRGBSpectrum() const
		{
			return *this;
		}
		void ToXYZ(double xyz[3]) const
		{
			RGBToXYZ(c, xyz);
		}
		static RGBSpectrum FromXYZ(const double xyz[3],
				SpectrumType type = SpectrumType::Reflectance)
		{
			RGBSpectrum r;
			XYZToRGB(xyz, r.c);
			return r;
		}
		double y() const
		{
			const double YWeight[3] = { 0.212671f, 0.715160f, 0.072169f };
			return YWeight[0] * c[0] + YWeight[1] * c[1] + YWeight[2] * c[2];
		}
	};
}