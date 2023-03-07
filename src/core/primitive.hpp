#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "../shape/shape.hpp"
#include "ZRender.hpp"
#include "geometry.hpp"
#include "interaction.hpp"


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
		virtual void ComputeScatteringFunctions() const = 0;
	};

	Primitive::~Primitive()
	{

	}

	class GeometricPrimitive : public Primitive
	{
	public:
		Bounds3 WorldBound() const override;
		bool Intersect(const Ray& r, SurfaceInteraction* isect) const override;
		bool IntersectP(const Ray& r) const override;
		GeometricPrimitive(const std::shared_ptr<Shape>& shape);
		void ComputeScatteringFunctions() const
		{
		}

	private:
		std::shared_ptr<Shape> shape;
	};

	GeometricPrimitive::GeometricPrimitive(const std::shared_ptr<Shape>& shape) : shape(shape)
	{
		primitiveMemory += sizeof(*this);
	}
	Bounds3 GeometricPrimitive::WorldBound() const
	{
		return shape->WorldBound();
	}
	bool GeometricPrimitive::Intersect(const Ray& r, SurfaceInteraction* isect) const
	{
		double tHit;
		if (!shape->Intersect(r, &tHit, isect)) return false;
		r.tMax = tHit;
		// Initialize _SurfaceInteraction::mediumInterface_ after _Shape_
		// intersection
		return true;
	}
	bool GeometricPrimitive::IntersectP(const Ray& r) const
	{
		return shape->IntersectP(r);
	}

	class Aggregate : public Primitive
	{
	public:
		// Aggregate Public Methods
		void ComputeScatteringFunctions() const
		{
		}

	};

}