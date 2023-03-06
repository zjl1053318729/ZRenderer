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
		Bounds3 ObjectBound() const override;
		Bounds3 WorldBound() const override;
		bool Intersect(const Ray& ray, float* tHit, SurfaceInteraction* isect,
				bool testAlphaTexture = true) const override;
		bool IntersectP(const Ray& ray, bool testAlphaTexture = true) const override;
		double Area() const override;
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

	Bounds3 Triangle::ObjectBound() const
	{
		// Get triangle vertices in _p0_, _p1_, and _p2_
		const Eigen::Vector3d& p0 = mesh->p[v[0]];
		const Eigen::Vector3d& p1 = mesh->p[v[1]];
		const Eigen::Vector3d& p2 = mesh->p[v[2]];
		return Union(Bounds3((*WorldToObject)(p0), (*WorldToObject)(p1)),
				(*WorldToObject)(p2));
	}
	Bounds3 Triangle::WorldBound() const
	{
		// Get triangle vertices in _p0_, _p1_, and _p2_
		const Eigen::Vector3d& p0 = mesh->p[v[0]];
		const Eigen::Vector3d& p1 = mesh->p[v[1]];
		const Eigen::Vector3d& p2 = mesh->p[v[2]];
		return Union(Bounds3(p0, p1), p2);
	}
	bool Triangle::Intersect(const Ray& ray, float* tHit, SurfaceInteraction* isect, bool testAlphaTexture) const
	{
		++nTests;
		// Get triangle vertices in _p0_, _p1_, and _p2_
		const Eigen::Vector3d& p0 = mesh->p[v[0]];
		const Eigen::Vector3d& p1 = mesh->p[v[1]];
		const Eigen::Vector3d& p2 = mesh->p[v[2]];

		// Perform ray--triangle intersection test

		// Transform triangle vertices to ray coordinate space

		// Translate vertices based on ray origin
		Eigen::Vector3d p0t = p0 - Eigen::Vector3d(ray.origin);
		Eigen::Vector3d p1t = p1 - Eigen::Vector3d(ray.origin);
		Eigen::Vector3d p2t = p2 - Eigen::Vector3d(ray.origin);

		// Permute components of triangle vertices and ray direction
		int kz = MaxDimension(abs(ray.direction));
		int kx = kz + 1;
		if (kx == 3) kx = 0;
		int ky = kx + 1;
		if (ky == 3) ky = 0;
		Eigen::Vector3d d = Permute(ray.direction, kx, ky, kz);
		p0t = Permute(p0t, kx, ky, kz);
		p1t = Permute(p1t, kx, ky, kz);
		p2t = Permute(p2t, kx, ky, kz);

		// Apply shear transformation to translated vertex positions
		float Sx = -d.x() / d.z();
		float Sy = -d.y() / d.z();
		float Sz = 1.f / d.z();
		p0t.x() += Sx * p0t.z();
		p0t.y() += Sy * p0t.z();
		p1t.x() += Sx * p1t.z();
		p1t.y() += Sy * p1t.z();
		p2t.x() += Sx * p2t.z();
		p2t.y() += Sy * p2t.z();

		// Compute edge function coefficients _e0_, _e1_, and _e2_
		float e0 = p1t.x() * p2t.y() - p1t.y() * p2t.x();
		float e1 = p2t.x() * p0t.y() - p2t.y() * p0t.x();
		float e2 = p0t.x() * p1t.y() - p0t.y() * p1t.x();

		// Fall back to double precision test at triangle edges
		if (e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)
		{
			double p2txp1ty = (double)p2t.x() * (double)p1t.y();
			double p2typ1tx = (double)p2t.y() * (double)p1t.x();
			e0 = (float)(p2typ1tx - p2txp1ty);
			double p0txp2ty = (double)p0t.x() * (double)p2t.y();
			double p0typ2tx = (double)p0t.y() * (double)p2t.x();
			e1 = (float)(p0typ2tx - p0txp2ty);
			double p1txp0ty = (double)p1t.x() * (double)p0t.y();
			double p1typ0tx = (double)p1t.y() * (double)p0t.x();
			e2 = (float)(p1typ0tx - p1txp0ty);
		}

		// Perform triangle edge and determinant tests
		if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
			return false;
		float det = e0 + e1 + e2;
		if (det == 0) return false;

		// Compute scaled hit distance to triangle and test against ray $t$ range
		p0t.z() *= Sz;
		p1t.z() *= Sz;
		p2t.z() *= Sz;
		float tScaled = e0 * p0t.z() + e1 * p1t.z() + e2 * p2t.z();
		if (det < 0 && (tScaled >= 0 || tScaled < ray.tMax * det))
			return false;
		else if (det > 0 && (tScaled <= 0 || tScaled > ray.tMax * det))
			return false;

		// Compute barycentric coordinates and $t$ value for triangle intersection
		float invDet = 1 / det;
		float b0 = e0 * invDet;
		float b1 = e1 * invDet;
		float b2 = e2 * invDet;
		float t = tScaled * invDet;

		*tHit = t;
		++nHits;
		return true;
	}
	bool Triangle::IntersectP(const Ray& ray, bool testAlphaTexture) const
	{
		return false;
	}
	double Triangle::Area() const
	{
		// Get triangle vertices in _p0_, _p1_, and _p2_
		const Eigen::Vector3d& p0 = mesh->p[v[0]];
		const Eigen::Vector3d& p1 = mesh->p[v[1]];
		const Eigen::Vector3d& p2 = mesh->p[v[2]];
		return 0.5 * ((p1 - p0).cross(p2 - p0)).norm();
	}
}
