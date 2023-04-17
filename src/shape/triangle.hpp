#pragma once

#include <memory>
#include "../core/interaction.hpp"

namespace ZR
{
	static long long nTris = 0;
	static long long nMeshes = 0;
	static long long nHits = 0;
	static long long nTests = 0;
	static long long triMeshBytes = 0;

	struct TriangleMesh
	{
		// TriangleMesh Public Methods
		TriangleMesh(
				const Transform& ObjectToWorld, int nTriangles, const int* vertexIndices,
				int nVertices, const Eigen::Vector3d* P, const Eigen::Vector3d* S, const Eigen::Vector3d* N,
				const Eigen::Vector2d* UV, const int* fIndices);
		// TriangleMesh Data
		const int nTriangles, nVertices;
		std::vector<int> vertexIndices;
		std::unique_ptr<Eigen::Vector3d[]> p; //point
		std::unique_ptr<Eigen::Vector3d[]> n; //normal
		std::unique_ptr<Eigen::Vector3d[]> s; //
		std::unique_ptr<Eigen::Vector2d[]> uv;
		std::vector<int> faceIndices;
	};


	class Triangle : public Shape
	{
	public:
		// Triangle Public Methods
		Triangle(const Transform* ObjectToWorld, const Transform* WorldToObject,
				bool reverseOrientation, const std::shared_ptr<TriangleMesh>& mesh,
				int triNumber)
				: Shape(ObjectToWorld, WorldToObject, reverseOrientation), mesh(mesh)
		{
			v = &mesh->vertexIndices[3 * triNumber];
			triMeshBytes += sizeof(*this);
			faceIndex = mesh->faceIndices.size() ? mesh->faceIndices[triNumber] : 0;
		}
		Bounds3 ObjectBound() const override;
		Bounds3 WorldBound() const override;
		bool Intersect(const Ray& ray, double* tHit, SurfaceInteraction* isect,
				bool testAlphaTexture = true) const override;
		bool IntersectP(const Ray& ray, bool testAlphaTexture = true) const override;
		double Area() const override;
		Interaction Sample(const Eigen::Vector2d& u, double* pdf) const;
	private:
		// Triangle Private Methods
		void GetUVs(Eigen::Vector2d uv[3]) const;
		// Triangle Private Data
		std::shared_ptr<TriangleMesh> mesh;
		const int* v;
		int faceIndex;
	};


}
