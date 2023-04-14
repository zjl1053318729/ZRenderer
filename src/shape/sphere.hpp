#pragma once

#include "shape.hpp"

namespace ZR
{
	// Sphere Declarations
	class Sphere : public Shape
	{
	public:
		// Sphere Public Methods
		Sphere(const Transform* ObjectToWorld, const Transform* WorldToObject,
				bool reverseOrientation, double radius)
				: Shape(ObjectToWorld, WorldToObject, reverseOrientation),
				  radius(radius)
		{
		}
		Bounds3 ObjectBound() const override;
		bool Intersect(const Ray& ray, double* tHit, SurfaceInteraction* isect,
				bool testAlphaTexture) const;
		bool IntersectP(const Ray& ray, bool testAlphaTexture) const;
		double Area() const;
	private:
		// Sphere Private Data
		const double radius;
	};


}