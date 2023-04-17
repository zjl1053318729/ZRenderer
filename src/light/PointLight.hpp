#pragma once

#include "light.hpp"

namespace ZR
{
	// PointLight Declarations
	class PointLight : public Light
	{
	public:
		// PointLight Public Methods
		PointLight(const Transform& LightToWorld, const Spectrum& I)
				: Light((int)LightFlags::DeltaPosition, LightToWorld),
				  pLight(LightToWorld(Eigen::Vector3d(0, 0, 0))),
				  I(I)
		{
		}
		Spectrum Sample_Li(const Interaction& ref, const Eigen::Vector2d& u, Eigen::Vector3d* wi,
				double* pdf, VisibilityTester* vis) const;
		Spectrum Power() const
		{
			return 4 * Pi * I;
		}
		double Pdf_Li(const Interaction&, const Eigen::Vector3d&) const
		{
			return 0;
		}
		Spectrum Sample_Le(const Eigen::Vector2d& u1, const Eigen::Vector2d& u2, double time,
				Ray* ray, Eigen::Vector3d* nLight, double* pdfPos,
				double* pdfDir) const;
		void Pdf_Le(const Ray&, const Eigen::Vector3d&, double* pdfPos,
				double* pdfDir) const;

	private:
		// PointLight Private Data
		const Eigen::Vector3d pLight;
		const Spectrum I;
	};
}