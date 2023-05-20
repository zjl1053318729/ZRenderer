

#include "primitive.hpp"
#include "../shape/shape.hpp"
#include "interaction.hpp"

namespace ZR
{
	static long long primitiveMemory = 0;

	Primitive::~Primitive()
	{

	}
	GeometricPrimitive::GeometricPrimitive(const std::shared_ptr<Shape>& shape,
			const std::shared_ptr<Material>& material, const std::shared_ptr<AreaLight>& areaLight) :
			shape(shape), material(material), areaLight(areaLight)
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
		isect->is_specular = material->is_specular;
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
	const AreaLight* GeometricPrimitive::GetAreaLight() const
	{
		return areaLight.get();
	}
	const Material* GeometricPrimitive::GetMaterial() const
	{
		return material.get();
	}
	const AreaLight* Aggregate::GetAreaLight() const
	{
		return nullptr;
	}
	const Material* Aggregate::GetMaterial() const
	{
		return nullptr;
	}
}