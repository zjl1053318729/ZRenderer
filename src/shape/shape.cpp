

#include "shape.hpp"

namespace ZR
{
	Bounds3 Shape::WorldBound() const
	{
		return (*ObjectToWorld)(ObjectBound());
	}
	Shape::~Shape()
	{

	}
	bool Shape::IntersectP(const Ray& ray, bool testAlphaTexture) const
	{
		return Intersect(ray, nullptr, nullptr, testAlphaTexture);
	}
}