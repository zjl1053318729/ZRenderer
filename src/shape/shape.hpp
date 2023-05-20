#pragma once

#include <Eigen/Eigen>
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
		virtual bool IntersectP(const Ray& ray, bool testAlphaTexture = true) const;
		virtual double Area() const = 0;
		// Sample a point on the surface of the shape and return the PDF with
		// respect to area on the surface.
		virtual Interaction Sample(const Eigen::Vector2d& u, double* pdf) const = 0;
		virtual double Pdf(const Interaction&) const
		{
			return 1 / Area();
		}
		// Sample a point on the shape given a reference point |ref| and
		// return the PDF with respect to solid angle from |ref|.
		virtual Interaction Sample(const Interaction& ref, const Eigen::Vector2d& u, double* pdf) const;
		virtual double Pdf(const Interaction& ref, const Eigen::Vector3d& wi) const;
	};


}