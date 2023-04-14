#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "geometry.hpp"
#include "../material/material.hpp"


namespace ZR
{
	static long long primitiveMemory = 0;

	class Primitive
	{
	public:
		virtual ~Primitive();
		virtual Bounds3 WorldBound() const = 0;
		virtual bool Intersect(const Ray& r, SurfaceInteraction*) const = 0;
		virtual bool IntersectP(const Ray& r) const = 0;
		virtual void ComputeScatteringFunctions(
				SurfaceInteraction* isect,
				TransportMode mode,
				bool allowMultipleLobes) const = 0;
	};

	class GeometricPrimitive : public Primitive
	{
	public:
		virtual Bounds3 WorldBound() const;
		virtual bool Intersect(const Ray& r, SurfaceInteraction* isect) const;
		virtual bool IntersectP(const Ray& r) const override;
		GeometricPrimitive(const std::shared_ptr<Shape>& shape,
				const std::shared_ptr<Material>& material);
		virtual void ComputeScatteringFunctions(SurfaceInteraction* isect,
				TransportMode mode,
				bool allowMultipleLobes) const;

	private:
		std::shared_ptr<Shape> shape;
		std::shared_ptr<Material> material;
	};

	class Aggregate : public Primitive
	{
	public:
		// Aggregate Public Methods
		virtual void ComputeScatteringFunctions(SurfaceInteraction* isect,
				TransportMode mode,
				bool allowMultipleLobes) const
		{
		}

	};

}