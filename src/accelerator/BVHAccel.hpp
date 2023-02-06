#pragma once

#include "../core/primitive.hpp"

namespace ZR
{
	static long long treeBytes = 0;

	struct BVHBuildNode;

// BVHAccel Forward Declarations
	struct BVHPrimitiveInfo
	{
		BVHPrimitiveInfo() {}
		BVHPrimitiveInfo(size_t primitiveNumber, const Bounds3 &bounds)
				: primitiveNumber(primitiveNumber),
				  bounds(bounds),
				  centroid(.5f * bounds.pMin + .5f * bounds.pMax) {}
		size_t primitiveNumber;
		Bounds3 bounds;
		Eigen::Vector3d centroid;
	};
	struct LinearBVHNode
	{
		Bounds3 bounds;
		union {
			int primitivesOffset;   // leaf
			int secondChildOffset;  // interior
		};
		uint16_t nPrimitives;  // 0 -> interior node
		uint8_t axis;          // interior node: xyz
		uint8_t pad[1];        // ensure 32 byte total size
	};

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
				SplitMethod splitMethod = SplitMethod::SAH) :
				maxPrimsInNode(std::min(255, maxPrimsInNode)),
				splitMethod(splitMethod),
				primitives(std::move(p))
		{
			if (primitives.empty()) return;
			// Build BVH from _primitives_

			// Initialize _primitiveInfo_ array for primitives
			std::vector<BVHPrimitiveInfo> primitiveInfo(primitives.size());
			for (size_t i = 0; i < primitives.size(); ++i)
				primitiveInfo[i] = { i, primitives[i]->WorldBound() };

			// Build BVH tree for primitives using _primitiveInfo_
			int totalNodes = 0;
			std::vector<std::shared_ptr<Primitive>> orderedPrims;
			orderedPrims.reserve(primitives.size());
			BVHBuildNode* root;

			root = recursiveBuild(primitiveInfo, 0, primitives.size(),
					&totalNodes, orderedPrims);
			primitives.swap(orderedPrims);
			primitiveInfo.resize(0);

			// Compute representation of depth-first traversal of BVH tree
			treeBytes += totalNodes * sizeof(LinearBVHNode) + sizeof(*this) +
						 primitives.size() * sizeof(primitives[0]);
			nodes = new LinearBVHNode[totalNodes];
			int offset = 0;
			flattenBVHTree(root, &offset);
		};
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

	BVHBuildNode*
	BVHAccel::recursiveBuild(std::vector<BVHPrimitiveInfo>& primitiveInfo, int start, int end, int* totalNodes,
			std::vector<std::shared_ptr<Primitive>>& orderedPrims)
	{
		BVHBuildNode *node = new BVHBuildNode;
		(*totalNodes)++;
		// Compute bounds of all primitives in BVH node
		Bounds3f bounds;
		for (int i = start; i < end; ++i)
			bounds = Union(bounds, primitiveInfo[i].bounds);
		int nPrimitives = end - start;
		if (nPrimitives == 1) {
			// Create leaf _BVHBuildNode_
			int firstPrimOffset = orderedPrims.size();
			for (int i = start; i < end; ++i) {
				int primNum = primitiveInfo[i].primitiveNumber;
				orderedPrims.push_back(primitives[primNum]);
			}
			node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
			return node;
		}
		else {
			// Compute bound of primitive centroids, choose split dimension _dim_
			Bounds3f centroidBounds;
			for (int i = start; i < end; ++i)
				centroidBounds = Union(centroidBounds, primitiveInfo[i].centroid);
			int dim = centroidBounds.MaximumExtent();

			// Partition primitives into two sets and build children
			int mid = (start + end) / 2;
			if (centroidBounds.pMax[dim] == centroidBounds.pMin[dim]) {
				// Create leaf _BVHBuildNode_
				int firstPrimOffset = orderedPrims.size();
				for (int i = start; i < end; ++i) {
					int primNum = primitiveInfo[i].primitiveNumber;
					orderedPrims.push_back(primitives[primNum]);
				}
				node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
				return node;
			}
			else {
				// Partition primitives based on _splitMethod_
				switch (splitMethod) {
				case SplitMethod::Middle: {
					// Partition primitives through node's midpoint
					float pmid =
							(centroidBounds.pMin[dim] + centroidBounds.pMax[dim]) / 2;
					BVHPrimitiveInfo *midPtr = std::partition(
							&primitiveInfo[start], &primitiveInfo[end - 1] + 1,
							[dim, pmid](const BVHPrimitiveInfo &pi) {
								return pi.centroid[dim] < pmid;
							});
					mid = midPtr - &primitiveInfo[0];
					// For lots of prims with large overlapping bounding boxes, this
					// may fail to partition; in that case don't break and fall
					// through
					// to EqualCounts.
					if (mid != start && mid != end) break;
				}
				case SplitMethod::EqualCounts: {
					// Partition primitives into equally-sized subsets
					mid = (start + end) / 2;
					std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
							&primitiveInfo[end - 1] + 1,
							[dim](const BVHPrimitiveInfo &a,
									const BVHPrimitiveInfo &b) {
								return a.centroid[dim] < b.centroid[dim];
							});
					break;
				}
				case SplitMethod::SAH:
				default: {
					// Partition primitives using approximate SAH
					if (nPrimitives <= 2) {
						// Partition primitives into equally-sized subsets
						mid = (start + end) / 2;
						std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
								&primitiveInfo[end - 1] + 1,
								[dim](const BVHPrimitiveInfo &a,
										const BVHPrimitiveInfo &b) {
									return a.centroid[dim] <
										   b.centroid[dim];
								});
					}
					else {
						// Allocate _BucketInfo_ for SAH partition buckets
						constexpr int nBuckets = 12;
						BucketInfo buckets[nBuckets];

						// Initialize _BucketInfo_ for SAH partition buckets
						for (int i = start; i < end; ++i) {
							int b = nBuckets *
									centroidBounds.Offset(
											primitiveInfo[i].centroid)[dim];
							if (b == nBuckets) b = nBuckets - 1;
							buckets[b].count++;
							buckets[b].bounds =
									Union(buckets[b].bounds, primitiveInfo[i].bounds);
						}

						// Compute costs for splitting after each bucket
						float cost[nBuckets - 1];
						for (int i = 0; i < nBuckets - 1; ++i) {
							Bounds3f b0, b1;
							int count0 = 0, count1 = 0;
							for (int j = 0; j <= i; ++j) {
								b0 = Union(b0, buckets[j].bounds);
								count0 += buckets[j].count;
							}
							for (int j = i + 1; j < nBuckets; ++j) {
								b1 = Union(b1, buckets[j].bounds);
								count1 += buckets[j].count;
							}
							cost[i] = 1 +
									  (count0 * b0.SurfaceArea() +
									   count1 * b1.SurfaceArea()) /
									  bounds.SurfaceArea();
						}

						// Find bucket to split at that minimizes SAH metric
						float minCost = cost[0];
						int minCostSplitBucket = 0;
						for (int i = 1; i < nBuckets - 1; ++i) {
							if (cost[i] < minCost) {
								minCost = cost[i];
								minCostSplitBucket = i;
							}
						}

						// Either create leaf or split primitives at selected SAH
						// bucket
						float leafCost = nPrimitives;
						if (nPrimitives > maxPrimsInNode || minCost < leafCost) {
							BVHPrimitiveInfo *pmid = std::partition(
									&primitiveInfo[start], &primitiveInfo[end - 1] + 1,
									[=](const BVHPrimitiveInfo &pi) {
										int b = nBuckets *
												centroidBounds.Offset(pi.centroid)[dim];
										if (b == nBuckets) b = nBuckets - 1;
										return b <= minCostSplitBucket;
									});
							mid = pmid - &primitiveInfo[0];
						}
						else {
							// Create leaf _BVHBuildNode_
							int firstPrimOffset = orderedPrims.size();
							for (int i = start; i < end; ++i) {
								int primNum = primitiveInfo[i].primitiveNumber;
								orderedPrims.push_back(primitives[primNum]);
							}
							node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
							return node;
						}
					}
					break;
				}

				}
				node->InitInterior(dim,
						recursiveBuild(primitiveInfo, start, mid,
								totalNodes, orderedPrims),
						recursiveBuild(primitiveInfo, mid, end,
								totalNodes, orderedPrims));
			}
		}
		return node;
	}
}