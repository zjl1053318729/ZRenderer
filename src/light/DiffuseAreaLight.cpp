//
// Created by 10533 on 2023/4/16.
//

#include "DiffuseAreaLight.hpp"
#include "../sampler/sampler.hpp"

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
	void DiffuseAreaLight::generatePhoton(Eigen::Vector3d& ori, Eigen::Vector3d& dir, double& powScale)
	{
		double pdf;
		Interaction inte = this->shape->Sample(Eigen::Vector2d::Random().array().abs(), &pdf);
		ori = inte.position;
		double phi,theta;
		theta = ZR::random_double(0, 2 * ZR::Pi);
		phi = ZR::random_double(0, 0.5 * ZR::Pi);

		Eigen::Matrix3d rotMatrix;
		rotMatrix = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,1,0), inte.normal).toRotationMatrix();
		dir =  rotMatrix * Eigen::Vector3d(cos(theta)*sin(phi), cos(phi), sin(theta)*sin(phi));

		powScale = dir.dot(inte.normal);
	}
}