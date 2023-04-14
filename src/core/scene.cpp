

#include "scene.hpp"

namespace ZR
{
	const Bounds3& Scene::WorldBound() const
	{
		return worldBound;
	}
	bool Scene::Intersect(const Ray& ray, SurfaceInteraction* isect) const
	{
		++nIntersectionTests;
		return aggregate->Intersect(ray, isect);
	}
	bool Scene::IntersectP(const Ray& ray) const
	{
		++nShadowTests;
		return aggregate->IntersectP(ray);
	}
}