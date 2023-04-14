#pragma once

#include "geometry.hpp"
#include "primitive.hpp"

namespace ZR
{
	static long long nIntersectionTests = 0;
	static long long nShadowTests = 0;

	class Scene
	{
	public:
		// Scene Public Methods
		Scene(Aggregate* aggregate)
				: aggregate(aggregate)
		{
			// Scene Constructor Implementation
			worldBound = aggregate->WorldBound();
		}
		const Bounds3& WorldBound() const;
		bool Intersect(const Ray& ray, SurfaceInteraction* isect) const;
		bool IntersectP(const Ray& ray) const;

	private:
		// Scene Private Data
		std::shared_ptr<Primitive> aggregate;
		Bounds3 worldBound;
	};
}