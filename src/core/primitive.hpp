#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "geometry.hpp"
#include "../material/material.hpp"


namespace ZR
{
	class Primitive
	{
	public:
		virtual ~Primitive();
		virtual Bounds3 WorldBound() const = 0;
		virtual bool Intersect(const Ray& r, SurfaceInteraction*) const = 0;
		virtual bool IntersectP(const Ray& r) const = 0;
		virtual const AreaLight* GetAreaLight() const = 0;
		virtual const Material* GetMaterial() const = 0;
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
				const std::shared_ptr<Material>& material,
				const std::shared_ptr<AreaLight>& areaLight);
		virtual void ComputeScatteringFunctions(SurfaceInteraction* isect,
				TransportMode mode,
				bool allowMultipleLobes) const;
		virtual const AreaLight* GetAreaLight() const;
		virtual const Material* GetMaterial() const;

	private:
		std::shared_ptr<Shape> shape;
		std::shared_ptr<Material> material;
		std::shared_ptr<AreaLight> areaLight;
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
		virtual const AreaLight* GetAreaLight() const;
		virtual const Material* GetMaterial() const;
	};

}