

#include "primitive.hpp"
#include "../shape/shape.hpp"
#include "interaction.hpp"

namespace ZR
{
	Primitive::~Primitive()
	{

	}
	GeometricPrimitive::GeometricPrimitive(const std::shared_ptr<Shape>& shape,
			const std::shared_ptr<Material>& material) : shape(shape), material(material)
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
		isect->primitive = this;
		return true;
	}
	bool GeometricPrimitive::IntersectP(const Ray& r) const
	{
		return shape->IntersectP(r);
	}
	void GeometricPrimitive::ComputeScatteringFunctions(SurfaceInteraction* isect,
			TransportMode mode,
			bool allowMultipleLobes) const
	{
		if (material)
			material->ComputeScatteringFunctions(isect, mode,
					allowMultipleLobes);
	}
}