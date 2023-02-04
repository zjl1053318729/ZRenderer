#pragma once

#include <memory>
#include "../core/transform.hpp"
#include "shape.hpp"

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
				const Eigen::Vector2d* UV, const int* fIndices)
				: nTriangles(nTriangles),
				  nVertices(nVertices),
				  vertexIndices(vertexIndices, vertexIndices + 3 * nTriangles)
		{
			++nMeshes;
			nTris += nTriangles;
			triMeshBytes += sizeof(*this) + this->vertexIndices.size() * sizeof(int) +
							nVertices * (sizeof(*P) + (N ? sizeof(*N) : 0) +
										 (S ? sizeof(*S) : 0) + (UV ? sizeof(*UV) : 0) +
										 (fIndices ? sizeof(*fIndices) : 0));

			// Transform mesh vertices to world space
			p.reset(new Eigen::Vector3d[nVertices]);
			for (int i = 0; i < nVertices; ++i) p[i] = ObjectToWorld(P[i]);
			// Copy _UV_, _N_, and _S_ vertex data, if present
			if (UV)
			{
				uv.reset(new Eigen::Vector2d[nVertices]);
				memcpy(uv.get(), UV, nVertices * sizeof(Eigen::Vector3d));
			}
			if (N)
			{
				n.reset(new Eigen::Vector3d[nVertices]);
				for (int i = 0; i < nVertices; ++i) n[i] = ObjectToWorld(N[i]);
			}
			if (S)
			{
				s.reset(new Eigen::Vector3d[nVertices]);
				for (int i = 0; i < nVertices; ++i) s[i] = ObjectToWorld(S[i]);
			}
			if (fIndices)
				faceIndices = std::vector<int>(fIndices, fIndices + nTriangles);
		}
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
		Bounds3 ObjectBound() const;
		Bounds3 WorldBound() const;
		bool Intersect(const Ray& ray, float* tHit, SurfaceInteraction* isect,
				bool testAlphaTexture = true) const;
		bool IntersectP(const Ray& ray, bool testAlphaTexture = true) const;
		double Area() const;
	private:
		// Triangle Private Methods
		void GetUVs(Eigen::Vector2d uv[3]) const
		{
			if (mesh->uv)
			{
				uv[0] = mesh->uv[v[0]];
				uv[1] = mesh->uv[v[1]];
				uv[2] = mesh->uv[v[2]];
			}
			else
			{
				uv[0] = Eigen::Vector2d(0, 0);
				uv[1] = Eigen::Vector2d(1, 0);
				uv[2] = Eigen::Vector2d(1, 1);
			}
		}
		// Triangle Private Data
		std::shared_ptr<TriangleMesh> mesh;
		const int* v;
		int faceIndex;
	};
}
