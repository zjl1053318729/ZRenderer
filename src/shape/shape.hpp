#pragma once

#include <Eigen/Eigen>
#include "../core/geometry.hpp"
#include "../core/transform.hpp"

namespace ZR
{
	static long long nShapesCreated = 0;

	class Shape
	{
	public:
		const Transform* ObjectToWorld, * WorldToObject;
		const bool reverseOrientation;

		// Shape Interface
		Shape(const Transform* ObjectToWorld, const Transform* WorldToObject, bool reverseOrientation) :
				ObjectToWorld(ObjectToWorld), WorldToObject(WorldToObject), reverseOrientation(reverseOrientation)
		{
			++nShapesCreated;
		};
		virtual ~Shape();
		virtual Bounds3 ObjectBound() const = 0;
		virtual Bounds3 WorldBound() const;
		virtual bool Intersect(const Ray& ray, double* tHit,
				SurfaceInteraction* isect,
				bool testAlphaTexture = true) const = 0;
		virtual bool IntersectP(const Ray& ray, bool testAlphaTexture = true) const
		{
			return Intersect(ray, nullptr, nullptr, testAlphaTexture);
		}
		virtual double Area() const = 0;
	};

	Bounds3 Shape::WorldBound() const
	{
		return (*ObjectToWorld)(ObjectBound());
	}
	Shape::~Shape()
	{

	}
}