

#include "triangle.hpp"

namespace ZR
{
	TriangleMesh::TriangleMesh(
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
	void Triangle::GetUVs(Eigen::Vector2d uv[3]) const
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
	bool Triangle::Intersect(const Ray& ray, double* tHit, SurfaceInteraction* isect, bool testAlphaTexture) const
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
		double Sx = -d.x() / d.z();
		double Sy = -d.y() / d.z();
		double Sz = 1.f / d.z();
		p0t.x() += Sx * p0t.z();
		p0t.y() += Sy * p0t.z();
		p1t.x() += Sx * p1t.z();
		p1t.y() += Sy * p1t.z();
		p2t.x() += Sx * p2t.z();
		p2t.y() += Sy * p2t.z();

		// Compute edge function coefficients _e0_, _e1_, and _e2_
		double e0 = p1t.x() * p2t.y() - p1t.y() * p2t.x();
		double e1 = p2t.x() * p0t.y() - p2t.y() * p0t.x();
		double e2 = p0t.x() * p1t.y() - p0t.y() * p1t.x();

		// Fall back to double precision test at triangle edges
		if (e0 == 0.0f || e1 == 0.0f || e2 == 0.0f)
		{
			double p2txp1ty = (double)p2t.x() * (double)p1t.y();
			double p2typ1tx = (double)p2t.y() * (double)p1t.x();
			e0 = (double)(p2typ1tx - p2txp1ty);
			double p0txp2ty = (double)p0t.x() * (double)p2t.y();
			double p0typ2tx = (double)p0t.y() * (double)p2t.x();
			e1 = (double)(p0typ2tx - p0txp2ty);
			double p1txp0ty = (double)p1t.x() * (double)p0t.y();
			double p1typ0tx = (double)p1t.y() * (double)p0t.x();
			e2 = (double)(p1typ0tx - p1txp0ty);
		}

		// Perform triangle edge and determinant tests
		if ((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0))
			return false;
		double det = e0 + e1 + e2;
		if (det == 0) return false;

		// Compute scaled hit distance to triangle and test against ray $t$ range
		p0t.z() *= Sz;
		p1t.z() *= Sz;
		p2t.z() *= Sz;
		double tScaled = e0 * p0t.z() + e1 * p1t.z() + e2 * p2t.z();
		if (det < 0 && (tScaled >= 0 || tScaled < ray.tMax * det))
			return false;
		else if (det > 0 && (tScaled <= 0 || tScaled > ray.tMax * det))
			return false;

		// Compute barycentric coordinates and $t$ value for triangle intersection
		double invDet = 1 / det;
		double b0 = e0 * invDet;
		double b1 = e1 * invDet;
		double b2 = e2 * invDet;
		double t = tScaled * invDet;

		// Compute $\delta_z$ term for triangle $t$ error bounds
		double maxZt = MaxComponent(abs(Eigen::Vector3d(p0t.z(), p1t.z(), p2t.z())));
		double deltaZ = gamma(3) * maxZt;
		// Compute $\delta_x$ and $\delta_y$ terms for triangle $t$ error bounds
		double maxXt = MaxComponent(abs(Eigen::Vector3d(p0t.x(), p1t.x(), p2t.x())));
		double maxYt = MaxComponent(abs(Eigen::Vector3d(p0t.y(), p1t.y(), p2t.y())));
		double deltaX = gamma(5) * (maxXt + maxZt);
		double deltaY = gamma(5) * (maxYt + maxZt);
		// Compute $\delta_e$ term for triangle $t$ error bounds
		double deltaE =
				2 * (gamma(2) * maxXt * maxYt + deltaY * maxXt + deltaX * maxYt);
		// Compute $\delta_t$ term for triangle $t$ error bounds and check _t_
		double maxE = MaxComponent(abs(Eigen::Vector3d(e0, e1, e2)));
		double deltaT = 3 *
					   (gamma(3) * maxE * maxZt + deltaE * maxZt + deltaZ * maxE) *
					   std::abs(invDet);
		if (t <= deltaT) return false;

		// Compute triangle partial derivatives
		Eigen::Vector3d dpdu, dpdv;
		Eigen::Vector2d uv[3];
		GetUVs(uv);

		// Compute deltas for triangle partial derivatives
		Eigen::Vector2d duv02 = uv[0] - uv[2], duv12 = uv[1] - uv[2];
		Eigen::Vector3d dp02 = p0 - p2, dp12 = p1 - p2;
		// Compute error bounds for triangle intersection
		double xAbsSum =
				(std::abs(b0 * p0.x()) + std::abs(b1 * p1.x()) + std::abs(b2 * p2.x()));
		double yAbsSum =
				(std::abs(b0 * p0.y()) + std::abs(b1 * p1.y()) + std::abs(b2 * p2.y()));
		double zAbsSum =
				(std::abs(b0 * p0.z()) + std::abs(b1 * p1.z()) + std::abs(b2 * p2.z()));
		Eigen::Vector3d pError = gamma(7) * Eigen::Vector3d(xAbsSum, yAbsSum, zAbsSum);
		Eigen::Vector2d uvHit = b0 * uv[0] + b1 * uv[1] + b2 * uv[2];
		Eigen::Vector3d pHit = b0 * p0 + b1 * p1 + b2 * p2;
		
		// Fill in _SurfaceInteraction_ from triangle hit
		*isect = SurfaceInteraction(pHit, pError, uvHit, -ray.direction, dpdu, dpdv,
				Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(0, 0, 0), ray.time,
				this, faceIndex);
		
		// Override surface normal in _isect_ for triangle
		isect->normal = isect->shading.n = Eigen::Vector3d((dp02.cross(dp12)).normalized());
		//if (reverseOrientation ^ transformSwapsHandedness)
		//	isect->n = isect->shading.n = -isect->n;
		
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