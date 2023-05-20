#pragma once

#include "light.hpp"

namespace ZR
{
	// DiffuseAreaLight Declarations
	class DiffuseAreaLight : public AreaLight
	{
	public:
		// DiffuseAreaLight Public Methods
		DiffuseAreaLight(const Transform& LightToWorld, const Spectrum& Le,
				int nSamples, const std::shared_ptr<Shape>& shape,
				bool twoSided = false);
		Spectrum L(const Interaction& intr, const Eigen::Vector3d& w) const
		{
			return (twoSided || intr.normal.dot(w) > 0) ? Lemit : Spectrum(0.f);
		}
		Spectrum Power() const;
		Spectrum Sample_Li(const Interaction& ref, const Eigen::Vector2d& u, Eigen::Vector3d* wo,
				double* pdf, VisibilityTester* vis) const;
		double Pdf_Li(const Interaction&, const Eigen::Vector3d&) const;
		Spectrum Sample_Le(const Eigen::Vector2d& u1, const Eigen::Vector2d& u2, double time,
				Ray* ray, Eigen::Vector3d* nLight, double* pdfPos,
				double* pdfDir) const;
		void Pdf_Le(const Ray&, const Eigen::Vector3d&, double* pdfPos,
				double* pdfDir) const;
		void generatePhoton(Eigen::Vector3d& ori, Eigen::Vector3d& dir, double& powScale);

	protected:
		// DiffuseAreaLight Protected Data
		const Spectrum Lemit;
		std::shared_ptr<Shape> shape;
		// Added after book publication: by default, DiffuseAreaLights still
		// only emit in the hemimsphere around the surface normal.  However,
		// this behavior can now be overridden to give emission on both sides.
		const bool twoSided;
		const double area;
	};
}