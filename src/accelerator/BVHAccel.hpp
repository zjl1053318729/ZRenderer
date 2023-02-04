#pragma once

#include "../core/primitive.hpp"

namespace ZR
{
	struct BVHBuildNode;

// BVHAccel Forward Declarations
	struct BVHPrimitiveInfo;
	struct LinearBVHNode;

	class BVHAccel : public Aggregate
	{
	public:
		// BVHAccel Public Types
		enum class SplitMethod
		{
			SAH, HLBVH, Middle, EqualCounts
		};

		// BVHAccel Public Methods
		BVHAccel(std::vector<std::shared_ptr<Primitive>> p,
				int maxPrimsInNode = 1,
				SplitMethod splitMethod = SplitMethod::SAH);
		Bounds3 WorldBound() const;
		~BVHAccel();
		bool Intersect(const Ray& ray, SurfaceInteraction* isect) const;
		bool IntersectP(const Ray& ray) const;

	private:
		// BVHAccel Private Methods
		BVHBuildNode* recursiveBuild(std::vector<BVHPrimitiveInfo>& primitiveInfo,
				int start, int end, int* totalNodes,
				std::vector<std::shared_ptr<Primitive>>& orderedPrims);
		int flattenBVHTree(BVHBuildNode* node, int* offset);

		// BVHAccel Private Data
		const int maxPrimsInNode;
		const SplitMethod splitMethod;
		std::vector<std::shared_ptr<Primitive>> primitives;
		LinearBVHNode* nodes = nullptr;
	};
}