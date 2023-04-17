#include "PointLight.hpp"
#include "../sampler/sampling.hpp"

namespace ZR
{
	// PointLight Method Definitions
	Spectrum PointLight::Sample_Li(const Interaction& ref, const Eigen::Vector2d& u,
			Eigen::Vector3d* wi, double* pdf,
			VisibilityTester* vis) const
	{
		*wi = (pLight - ref.position).normalized();
		*pdf = 1.f;
		*vis = VisibilityTester(ref, Interaction(pLight, ref.time));
		Eigen::Vector3d _x = pLight - ref.position;
		return I / _x.dot(_x);
	}

	Spectrum PointLight::Sample_Le(const Eigen::Vector2d& u1, const Eigen::Vector2d& u2, double time,
			Ray* ray, Eigen::Vector3d* nLight, double* pdfPos, double* pdfDir) const
	{
		*ray = Ray(pLight, UniformSampleSphere(u1), Infinity, time);
		*nLight = (Eigen::Vector3d)ray->direction;
		*pdfPos = 1;
		*pdfDir = UniformSpherePdf();
		return I;
	}

	void PointLight::Pdf_Le(const Ray&, const Eigen::Vector3d&, double* pdfPos,
			double* pdfDir) const
	{
		*pdfPos = 0;
		*pdfDir = UniformSpherePdf();
	}
}