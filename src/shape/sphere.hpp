#pragma once

#include "shape.hpp"

namespace ZR
{
	// Sphere Declarations
	class Sphere : public Shape
	{
	public:
		// Sphere Public Methods
		Sphere(const Transform *ObjectToWorld, const Transform *WorldToObject,
				bool reverseOrientation, double radius)
				: Shape(ObjectToWorld, WorldToObject, reverseOrientation),
				  radius(radius) {}
		Bounds3 ObjectBound() const;
		bool Intersect(const Ray &ray, double *tHit, SurfaceInteraction *isect,
				bool testAlphaTexture) const;
		bool IntersectP(const Ray &ray, bool testAlphaTexture) const;
		double Area() const;
	private:
		// Sphere Private Data
		const double radius;
	};

	bool Sphere::Intersect(const Ray &r, double *tHit, SurfaceInteraction *isect, bool testAlphaTexture) const
	{
		Eigen::Vector3d pHit;
		// Transform _Ray_ to object space
		Ray ray = (*WorldToObject)(r);

		Eigen::Vector3d oc = ray.origin - Eigen::Vector3d(0.0f,0.0f,0.0f);
		double a = ray.direction.dot(ray.direction);
		double b = 2.0 * oc.dot(ray.direction);
		double c = oc.dot(oc) - radius*radius;
		double discriminant = b*b - 4 * a*c;

		return (discriminant > 0);
	}

	bool Sphere::IntersectP(const Ray &r, bool testAlphaTexture) const
	{
		return false;
	}
}