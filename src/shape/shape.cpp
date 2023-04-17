

#include "shape.hpp"
#include "../core/interaction.hpp"

namespace ZR
{
	Bounds3 Shape::WorldBound() const
	{
		return (*ObjectToWorld)(ObjectBound());
	}
	Shape::~Shape()
	{

	}
	bool Shape::IntersectP(const Ray& ray, bool testAlphaTexture) const
	{
		return Intersect(ray, nullptr, nullptr, testAlphaTexture);
	}
	Interaction Shape::Sample(const Interaction& ref, const Eigen::Vector2d& u, double* pdf) const
	{
		Interaction intr = Sample(u, pdf);
		Eigen::Vector3d wi = intr.position - ref.position;
		if (fabs(wi.dot(wi)) <= 1e-6)
			*pdf = 0;
		else
		{
			wi = wi.normalized();
			// Convert from area measure, as returned by the Sample() call
			// above, to solid angle measure.
			Eigen::Vector3d aa = ref.position - intr.position;
			*pdf *= aa.dot(aa) / std::fabs(intr.normal.dot(-wi));
			if (std::isinf(*pdf)) *pdf = 0.0;
		}
		return intr;
	}
	double Shape::Pdf(const Interaction& ref, const Eigen::Vector3d& wi) const
	{
		// Intersect sample ray with area light geometry
		Ray ray = ref.SpawnRay(wi);
		double tHit;
		SurfaceInteraction isectLight;
		// Ignore any alpha textures used for trimming the shape when performing
		// this intersection. Hack for the "San Miguel" scene, where this is used
		// to make an invisible area light.
		if (!Intersect(ray, &tHit, &isectLight, false)) return 0;

		// Convert light sample weight to solid angle measure
		Eigen::Vector3d aa = ref.position - isectLight.position;
		double pdf = aa.dot(aa) / (std::fabs(isectLight.normal.dot(-wi)) * Area());
		if (std::isinf(pdf)) pdf = 0.0;
		return pdf;
	}
}