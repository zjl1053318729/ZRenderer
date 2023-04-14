

#include "sphere.hpp"

namespace ZR
{
	bool Sphere::Intersect(const Ray& r, double* tHit, SurfaceInteraction* isect, bool testAlphaTexture) const
	{
		Eigen::Vector3d pHit;
		// Transform _Ray_ to object space
		Ray ray = (*WorldToObject)(r);

		Eigen::Vector3d oc = ray.origin - Eigen::Vector3d(0.0f, 0.0f, 0.0f);
		double a = ray.direction.dot(ray.direction);
		double b = 2.0 * oc.dot(ray.direction);
		double c = oc.dot(oc) - radius * radius;
		double discriminant = b * b - 4 * a * c;

		return (discriminant > 0);
	}

	bool Sphere::IntersectP(const Ray& r, bool testAlphaTexture) const
	{
		return false;
	}
	Bounds3 Sphere::ObjectBound() const
	{
		return Bounds3(Eigen::Vector3d(-radius, -radius, -radius),
				Eigen::Vector3d(radius, radius, radius));
	}
	double Sphere::Area() const
	{
		return 4.0 * Pi * radius * radius;
	}
}