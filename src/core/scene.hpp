#pragma once

#include "geometry.hpp"
#include "primitive.hpp"
#include "../light/light.hpp"

namespace ZR
{
	static long long nIntersectionTests = 0;
	static long long nShadowTests = 0;

	class Scene
	{
	public:
		// Scene Public Methods
		Scene(std::shared_ptr<Primitive> aggregate, const std::vector<std::shared_ptr<Light>>& lights)
				: aggregate(aggregate), lights(lights)
		{
			// Scene Constructor Implementation
			worldBound = aggregate->WorldBound();
			for (const auto& light: lights)
			{
				light->Preprocess(*this);
				if (light->flags & (int)LightFlags::Infinite)
					infiniteLights.push_back(light);
			}
		}
		const Bounds3& WorldBound() const;
		bool Intersect(const Ray& ray, SurfaceInteraction* isect) const;
		bool IntersectP(const Ray& ray) const;

		std::vector<std::shared_ptr<Light>> lights;
		std::vector<std::shared_ptr<Light>> infiniteLights;
	private:
		// Scene Private Data
		std::shared_ptr<Primitive> aggregate;
		Bounds3 worldBound;
	};
}