#pragma once

#include "../core/primitive.hpp"

namespace ZR
{
	static long long treeBytes = 0;
	static long long totalPrimitives = 0;
	static long long totalLeafNodes = 0;
	static long long interiorNodes = 0;
	static long long leafNodes = 0;

	struct BVHBuildNode
	{
		// BVHBuildNode Public Methods
		void InitLeaf(int first, int n, const Bounds3& b)
		{
			firstPrimOffset = first;
			nPrimitives = n;
			bounds = b;
			children[0] = children[1] = nullptr;
			++leafNodes;
			++totalLeafNodes;
			totalPrimitives += n;
		}
		void InitInterior(int axis, BVHBuildNode* c0, BVHBuildNode* c1)
		{
			children[0] = c0;
			children[1] = c1;
			bounds = ZR::Union(c0->bounds, c1->bounds);
			splitAxis = axis;
			nPrimitives = 0;
			++interiorNodes;
		}
		Bounds3 bounds;
		BVHBuildNode* children[2];
		int splitAxis, firstPrimOffset, nPrimitives;
	};

// BVHAccel Forward Declarations
	struct BVHPrimitiveInfo
	{
		BVHPrimitiveInfo()
		{
		}
		BVHPrimitiveInfo(size_t primitiveNumber, const Bounds3& bounds)
				: primitiveNumber(primitiveNumber),
				  bounds(bounds),
				  centroid(.5f * bounds.pMin + .5f * bounds.pMax)
		{
		}
		size_t primitiveNumber;
		Bounds3 bounds;
		Eigen::Vector3d centroid;
	};

	struct LinearBVHNode
	{
		Bounds3 bounds;
		union
		{
			int primitivesOffset;   // leaf
			int secondChildOffset;  // interior
		};
		uint16_t nPrimitives;  // 0 -> interior node
		uint8_t axis;          // interior node: xyz
		uint8_t pad[1];        // ensure 32 byte total size
	};
	struct BucketInfo
	{
		int count = 0;
		Bounds3 bounds;
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
		Bounds3 WorldBound() const override;
		~BVHAccel() override;
		bool Intersect(const Ray& ray, SurfaceInteraction* isect) const override;
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
		BVHBuildNode* node = new BVHBuildNode;
		(*totalNodes)++;
		// Compute bounds of all primitives in BVH node
		Bounds3 bounds;
		for (int i = start; i < end; ++i)
			bounds = Union(bounds, primitiveInfo[i].bounds);
		int nPrimitives = end - start;
		if (nPrimitives == 1)
		{
			// Create leaf _BVHBuildNode_
			int firstPrimOffset = orderedPrims.size();
			for (int i = start; i < end; ++i)
			{
				int primNum = primitiveInfo[i].primitiveNumber;
				orderedPrims.push_back(primitives[primNum]);
			}
			node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
			return node;
		}
		else
		{
			// Compute bound of primitive centroids, choose split dimension _dim_
			Bounds3 centroidBounds;
			for (int i = start; i < end; ++i)
				centroidBounds = Union(centroidBounds, primitiveInfo[i].centroid);
			int dim = centroidBounds.MaximumExtent();

			// Partition primitives into two sets and build children
			int mid = (start + end) / 2;
			if (centroidBounds.pMax[dim] == centroidBounds.pMin[dim])
			{
				// Create leaf _BVHBuildNode_
				int firstPrimOffset = orderedPrims.size();
				for (int i = start; i < end; ++i)
				{
					int primNum = primitiveInfo[i].primitiveNumber;
					orderedPrims.push_back(primitives[primNum]);
				}
				node->InitLeaf(firstPrimOffset, nPrimitives, bounds);
				return node;
			}
			else
			{
				// Partition primitives based on _splitMethod_
				switch (splitMethod)
				{
				case SplitMethod::Middle:
				{
					// Partition primitives through node's midpoint
					double pmid =
							(centroidBounds.pMin[dim] + centroidBounds.pMax[dim]) / 2;
					BVHPrimitiveInfo* midPtr = std::partition(
							&primitiveInfo[start], &primitiveInfo[end - 1] + 1,
							[dim, pmid](const BVHPrimitiveInfo& pi)
							{
								return pi.centroid[dim] < pmid;
							});
					mid = midPtr - &primitiveInfo[0];
					// For lots of prims with large overlapping bounding boxes, this
					// may fail to partition; in that case don't break and fall
					// through
					// to EqualCounts.
					if (mid != start && mid != end) break;
				}
				case SplitMethod::EqualCounts:
				{
					// Partition primitives into equally-sized subsets
					mid = (start + end) / 2;
					std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
							&primitiveInfo[end - 1] + 1,
							[dim](const BVHPrimitiveInfo& a,
									const BVHPrimitiveInfo& b)
							{
								return a.centroid[dim] < b.centroid[dim];
							});
					break;
				}
				case SplitMethod::SAH:
				default:
				{
					// Partition primitives using approximate SAH
					if (nPrimitives <= 2)
					{
						// Partition primitives into equally-sized subsets
						mid = (start + end) / 2;
						std::nth_element(&primitiveInfo[start], &primitiveInfo[mid],
								&primitiveInfo[end - 1] + 1,
								[dim](const BVHPrimitiveInfo& a,
										const BVHPrimitiveInfo& b)
								{
									return a.centroid[dim] <
										   b.centroid[dim];
								});
					}
					else
					{
						// Allocate _BucketInfo_ for SAH partition buckets
						constexpr int nBuckets = 12;
						BucketInfo buckets[nBuckets];

						// Initialize _BucketInfo_ for SAH partition buckets
						for (int i = start; i < end; ++i)
						{
							int b = nBuckets *
									centroidBounds.Offset(
											primitiveInfo[i].centroid)[dim];
							if (b == nBuckets) b = nBuckets - 1;
							buckets[b].count++;
							buckets[b].bounds =
									Union(buckets[b].bounds, primitiveInfo[i].bounds);
						}

						// Compute costs for splitting after each bucket
						double cost[nBuckets - 1];
						for (int i = 0; i < nBuckets - 1; ++i)
						{
							Bounds3 b0, b1;
							int count0 = 0, count1 = 0;
							for (int j = 0; j <= i; ++j)
							{
								b0 = Union(b0, buckets[j].bounds);
								count0 += buckets[j].count;
							}
							for (int j = i + 1; j < nBuckets; ++j)
							{
								b1 = Union(b1, buckets[j].bounds);
								count1 += buckets[j].count;
							}
							cost[i] = 1 +
									  (count0 * b0.SurfaceArea() +
									   count1 * b1.SurfaceArea()) /
									  bounds.SurfaceArea();
						}

						// Find bucket to split at that minimizes SAH metric
						double minCost = cost[0];
						int minCostSplitBucket = 0;
						for (int i = 1; i < nBuckets - 1; ++i)
						{
							if (cost[i] < minCost)
							{
								minCost = cost[i];
								minCostSplitBucket = i;
							}
						}

						// Either create leaf or split primitives at selected SAH
						// bucket
						double leafCost = nPrimitives;
						if (nPrimitives > maxPrimsInNode || minCost < leafCost)
						{
							BVHPrimitiveInfo* pmid = std::partition(
									&primitiveInfo[start], &primitiveInfo[end - 1] + 1,
									[=](const BVHPrimitiveInfo& pi)
									{
										int b = nBuckets *
												centroidBounds.Offset(pi.centroid)[dim];
										if (b == nBuckets) b = nBuckets - 1;
										return b <= minCostSplitBucket;
									});
							mid = pmid - &primitiveInfo[0];
						}
						else
						{
							// Create leaf _BVHBuildNode_
							int firstPrimOffset = orderedPrims.size();
							for (int i = start; i < end; ++i)
							{
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
	int BVHAccel::flattenBVHTree(BVHBuildNode* node, int* offset)
	{
		LinearBVHNode* linearNode = &nodes[*offset];
		linearNode->bounds = node->bounds;
		int myOffset = (*offset)++;
		if (node->nPrimitives > 0)
		{
			linearNode->primitivesOffset = node->firstPrimOffset;
			linearNode->nPrimitives = node->nPrimitives;
		}
		else
		{
			// Create interior flattened BVH node
			linearNode->axis = node->splitAxis;
			linearNode->nPrimitives = 0;
			flattenBVHTree(node->children[0], offset);
			linearNode->secondChildOffset =
					flattenBVHTree(node->children[1], offset);
		}
		return myOffset;
	}
	BVHAccel::~BVHAccel()
	{
		free(nodes);
	}
	Bounds3 BVHAccel::WorldBound() const
	{
		return nodes ? nodes[0].bounds : Bounds3();
	}
	bool BVHAccel::Intersect(const Ray& ray, SurfaceInteraction* isect) const
	{
		if (!nodes) return false;
		bool hit = false;

		Eigen::Vector3d invDir(1 / ray.direction.x(), 1 / ray.direction.y(), 1 / ray.direction.z());
		int dirIsNeg[3] = { invDir.x() < 0, invDir.y() < 0, invDir.z() < 0 };
		// Follow ray through BVH nodes to find primitive intersections
		int toVisitOffset = 0, currentNodeIndex = 0;
		int nodesToVisit[64];
		while (true)
		{

			const LinearBVHNode* node = &nodes[currentNodeIndex];
			// Check ray against BVH node
			if (node->bounds.IntersectP(ray, invDir, dirIsNeg))
			{
				if (node->nPrimitives > 0)
				{

					// Intersect ray with primitives in leaf BVH node
					for (int i = 0; i < node->nPrimitives; ++i)
						if (primitives[node->primitivesOffset + i]->Intersect(ray, isect))
							hit = true;
					if (toVisitOffset == 0) break;
					currentNodeIndex = nodesToVisit[--toVisitOffset];
				}
				else
				{
					// Put far BVH node on _nodesToVisit_ stack, advance to near
					// node
					if (dirIsNeg[node->axis])
					{
						nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
						currentNodeIndex = node->secondChildOffset;
					}
					else
					{
						nodesToVisit[toVisitOffset++] = node->secondChildOffset;
						currentNodeIndex = currentNodeIndex + 1;
					}
				}
			}
			else
			{
				if (toVisitOffset == 0) break;
				currentNodeIndex = nodesToVisit[--toVisitOffset];
			}
		}
		return hit;
	}
	bool BVHAccel::IntersectP(const Ray& ray) const
	{
		if (!nodes) return false;
		Eigen::Vector3d invDir(1.f / ray.direction.x(), 1.f / ray.direction.y(), 1.f / ray.direction.z());
		int dirIsNeg[3] = { invDir.x() < 0, invDir.y() < 0, invDir.z() < 0 };
		int nodesToVisit[64];
		int toVisitOffset = 0, currentNodeIndex = 0;
		while (true)
		{
			const LinearBVHNode* node = &nodes[currentNodeIndex];
			if (node->bounds.IntersectP(ray, invDir, dirIsNeg))
			{
				// Process BVH node _node_ for traversal
				if (node->nPrimitives > 0)
				{
					for (int i = 0; i < node->nPrimitives; ++i)
					{
						if (primitives[node->primitivesOffset + i]->IntersectP(
								ray))
						{
							return true;
						}
					}
					if (toVisitOffset == 0) break;
					currentNodeIndex = nodesToVisit[--toVisitOffset];
				}
				else
				{
					if (dirIsNeg[node->axis])
					{
						/// second child first
						nodesToVisit[toVisitOffset++] = currentNodeIndex + 1;
						currentNodeIndex = node->secondChildOffset;
					}
					else
					{
						nodesToVisit[toVisitOffset++] = node->secondChildOffset;
						currentNodeIndex = currentNodeIndex + 1;
					}
				}
			}
			else
			{
				if (toVisitOffset == 0) break;
				currentNodeIndex = nodesToVisit[--toVisitOffset];
			}
		}
		return false;
	}
}