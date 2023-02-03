#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "ZRender.hpp"
#include "geometry.hpp"
#include "intersection.hpp"

namespace ZR
{
	class Primitive {
	public:
		virtual ~Primitive();
		virtual Bounds3 WorldBound() const = 0;
		virtual bool Intersect(const Ray &r, SurfaceInteraction *) const = 0;
		virtual bool IntersectP(const Ray &r) const = 0;
		virtual void ComputeScatteringFunctions() const = 0;
	};

	class GeometricPrimitive : public Primitive {
	public:
		virtual Bounds3 WorldBound() const;
		virtual bool Intersect(const Ray &r, SurfaceInteraction *isect) const;
		virtual bool IntersectP(const Ray &r) const;
		GeometricPrimitive(const std::shared_ptr<Shape> &shape);
		void ComputeScatteringFunctions() const {}

	private:
		std::shared_ptr<Shape> shape;
	};

	class Aggregate : public Primitive {
	public:
		// Aggregate Public Methods
		void ComputeScatteringFunctions() const {}

	};

}