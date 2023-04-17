//
// Created by 10533 on 2023/4/16.
//

#include "DiffuseAreaLight.hpp"

namespace ZR
{
	// DiffuseAreaLight Method Definitions
	DiffuseAreaLight::DiffuseAreaLight(const Transform& LightToWorld,
			const Spectrum& Lemit, int nSamples,
			const std::shared_ptr<Shape>& shape,
			bool twoSided)
			: AreaLight(LightToWorld, nSamples),
			  Lemit(Lemit),
			  shape(shape),
			  twoSided(twoSided),
			  area(shape->Area())
	{
		// Warn if light has transformation with non-uniform scale, though not
		// for Triangles, since this doesn't matter for them.
		//if (WorldToLight.HasScale() && dynamic_cast<const Triangle *>(shape.get()) == nullptr);
	}


	Spectrum DiffuseAreaLight::Power() const
	{
		return (twoSided ? 2 : 1) * Lemit * area * Pi;
	}

	Spectrum DiffuseAreaLight::Sample_Li(const Interaction& ref, const Eigen::Vector2d& u,
			Eigen::Vector3d* wi, double* pdf,
			VisibilityTester* vis) const
	{
		Interaction pShape = shape->Sample(ref, u, pdf);
		Eigen::Vector3d _a = pShape.position - ref.position;
		if (*pdf == 0 || fabs(_a.dot(_a)) <= 1e-6)
		{
			*pdf = 0;
			return 0.f;
		}
		*wi = (pShape.position - ref.position).normalized();
		*vis = VisibilityTester(ref, pShape);
		return L(pShape, -*wi);
	}

	double DiffuseAreaLight::Pdf_Li(const Interaction& ref,
			const Eigen::Vector3d& wi) const
	{
		return shape->Pdf(ref, wi);
	}

	Spectrum DiffuseAreaLight::Sample_Le(const Eigen::Vector2d& u1, const Eigen::Vector2d& u2,
			double time, Ray* ray, Eigen::Vector3d* nLight,
			double* pdfPos, double* pdfDir) const
	{
		return Spectrum(0.f);
	}

	void DiffuseAreaLight::Pdf_Le(const Ray& ray, const Eigen::Vector3d& n, double* pdfPos,
			double* pdfDir) const
	{
	}
}