#pragma once

#include <Eigen/Eigen>
#include "../core/geometry.hpp"
#include "../core/intersection.hpp"

namespace ZR
{
	class Shape
	{
	public:
		// Shape Interface
		Shape(const Transform *ObjectToWorld, const Transform *WorldToObject, bool reverseOrientation);
		virtual ~Shape();
		virtual Bounds3 ObjectBound() const = 0;
		virtual Bounds3 WorldBound() const;
		virtual bool Intersect(const Ray &ray, float *tHit,
				SurfaceInteraction *isect,
				bool testAlphaTexture = true) const = 0;
		virtual bool IntersectP(const Ray &ray,
				bool testAlphaTexture = true) const {
			return Intersect(ray, nullptr, nullptr, testAlphaTexture);
		}
		virtual float Area() const = 0;

		const Transform *ObjectToWorld, *WorldToObject;
		const bool reverseOrientation;
	};
}