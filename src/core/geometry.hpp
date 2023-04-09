#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cassert>
#include "ZRender.hpp"

namespace ZR
{
	class Ray
	{
	public:
		Eigen::Vector3d origin, direction;
		mutable double tMax;
		double time;

		Ray() : tMax(ZR::Infinity), time(0.f)
		{
		}
		Ray(const Eigen::Vector3d& _o, const Eigen::Vector3d& _d, double tMax = Infinity, double time = 0.f)
				: origin(_o), direction(_d), tMax(tMax), time(time)
		{
		}
		Eigen::Vector3d operator()(double t) const
		{
			return origin + direction * t;
		}
		bool HasNaNs()
		{
			return (origin.hasNaN() || direction.hasNaN() || std::isnan(tMax));
		}
	};

	class Bounds2
	{
	public:
		// Bounds2 Public Data
		Eigen::Vector2d pMin, pMax;

		// Bounds2 Public Methods
		Bounds2()
		{
			double minNum = std::numeric_limits<double>::lowest();
			double maxNum = std::numeric_limits<double>::max();
			pMin = Eigen::Vector2d(maxNum, maxNum);
			pMax = Eigen::Vector2d(minNum, minNum);
		}
		explicit Bounds2(const Eigen::Vector2d& p) : pMin(p), pMax(p)
		{
		}
		Bounds2(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
		{
			pMin = Eigen::Vector2d(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()));
			pMax = Eigen::Vector2d(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()));
		}

		Eigen::Vector2d Diagonal() const
		{
			return pMax - pMin;
		}
		double Area() const
		{
			Eigen::Vector2d d = pMax - pMin;
			return (d.x() * d.y());
		}
		int MaximumExtent() const
		{
			Eigen::Vector2d diag = Diagonal();
			if (diag.x() > diag.y())
				return 0;
			else
				return 1;
		}
		inline const Eigen::Vector2d& operator[](int i) const
		{
			return (i == 0) ? pMin : pMax;
		}
		inline Eigen::Vector2d& operator[](int i)
		{
			return (i == 0) ? pMin : pMax;
		}
		bool operator==(const Bounds2& b) const
		{
			return b.pMin == pMin && b.pMax == pMax;
		}
		bool operator!=(const Bounds2& b) const
		{
			return b.pMin != pMin || b.pMax != pMax;
		}
		Eigen::Vector2d Lerp(double t, const Eigen::Vector2d& v0, const Eigen::Vector2d& v1)
		{
			return (1 - t) * v0 + t * v1;
		}
		Eigen::Vector2d Lerp(const Eigen::Vector2d& t) const
		{
			return Eigen::Vector2d(ZR::Lerp(t.x(), pMin.x(), pMax.x()),
					ZR::Lerp(t.y(), pMin.y(), pMax.y()));
		}
		Eigen::Vector2d Offset(const Eigen::Vector2d& p) const
		{
			Eigen::Vector2d o = p - pMin;
			if (pMax.x() > pMin.x()) o.x() /= pMax.x() - pMin.x();
			if (pMax.y() > pMin.y()) o.y() /= pMax.y() - pMin.y();
			return o;
		}
		bool Inside(const Eigen::Vector2d& pt, const Bounds2& b) const
		{
			return (pt.x() >= b.pMin.x() && pt.x() <= b.pMax.x() && pt.y() >= b.pMin.y() &&
					pt.y() <= b.pMax.y());
		}
		inline double Distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const
		{
			return (p1 - p2).norm();
		}
		void BoundingSphere(Eigen::Vector2d* c, double* rad) const
		{
			*c = (pMin + pMax) / 2;
			*rad = Inside(*c, *this) ? Distance(*c, pMax) : 0;
		}
	};

	class Bounds2i
	{
	public:
		// Bounds2 Public Data
		Eigen::Vector2i pMin, pMax;

		// Bounds2 Public Methods
		Bounds2i()
		{
			double minNum = std::numeric_limits<int>::lowest();
			double maxNum = std::numeric_limits<int>::max();
			pMin = Eigen::Vector2i(maxNum, maxNum);
			pMax = Eigen::Vector2i(minNum, minNum);
		}
		explicit Bounds2i(const Eigen::Vector2i& p) : pMin(p), pMax(p)
		{
		}
		Bounds2i(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2)
		{
			pMin = Eigen::Vector2i(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()));
			pMax = Eigen::Vector2i(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()));
		}

		Eigen::Vector2i Diagonal() const
		{
			return pMax - pMin;
		}
		int Area() const
		{
			Eigen::Vector2i d = pMax - pMin;
			return (d.x() * d.y());
		}
		int MaximumExtent() const
		{
			Eigen::Vector2i diag = Diagonal();
			if (diag.x() > diag.y())
				return 0;
			else
				return 1;
		}
		inline const Eigen::Vector2i& operator[](int i) const
		{
			return (i == 0) ? pMin : pMax;
		}
		inline Eigen::Vector2i& operator[](int i)
		{
			return (i == 0) ? pMin : pMax;
		}
		bool operator==(const Bounds2i& b) const
		{
			return b.pMin == pMin && b.pMax == pMax;
		}
		bool operator!=(const Bounds2i& b) const
		{
			return b.pMin != pMin || b.pMax != pMax;
		}
//		Eigen::Vector2i Lerp(double t, const Eigen::Vector2i& v0, const Eigen::Vector2i& v1)
//		{
//			return (1 - t) * v0 + t * v1;
//		}
		Eigen::Vector2i Lerp(const Eigen::Vector2i& t) const
		{
			return Eigen::Vector2i(ZR::Lerp(t.x(), pMin.x(), pMax.x()),
					ZR::Lerp(t.y(), pMin.y(), pMax.y()));
		}
		Eigen::Vector2i Offset(const Eigen::Vector2i& p) const
		{
			Eigen::Vector2i o = p - pMin;
			if (pMax.x() > pMin.x()) o.x() /= pMax.x() - pMin.x();
			if (pMax.y() > pMin.y()) o.y() /= pMax.y() - pMin.y();
			return o;
		}
		bool Inside(const Eigen::Vector2i& pt, const Bounds2i& b) const
		{
			return (pt.x() >= b.pMin.x() && pt.x() <= b.pMax.x() && pt.y() >= b.pMin.y() &&
					pt.y() <= b.pMax.y());
		}
		inline double Distance(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2) const
		{
			return (p1 - p2).norm();
		}
		void BoundingSphere(Eigen::Vector2i* c, double* rad) const
		{
			*c = (pMin + pMax) / 2;
			*rad = Inside(*c, *this) ? Distance(*c, pMax) : 0;
		}
	};

	class Bounds3
	{
	public:
		Eigen::Vector3d pMin, pMax;
		Bounds3()
		{
			double minNum = std::numeric_limits<double>::lowest();
			double maxNum = std::numeric_limits<double>::max();
			pMin = Eigen::Vector3d(maxNum, maxNum, maxNum);
			pMax = Eigen::Vector3d(minNum, minNum, minNum);
		}
		explicit Bounds3(const Eigen::Vector3d& p) : pMin(p), pMax(p)
		{
		}
		Bounds3(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
				: pMin(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()),
				std::min(p1.z(), p2.z())),
				  pMax(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()),
						  std::max(p1.z(), p2.z()))
		{
		}

		bool operator==(const Bounds3& b) const
		{
			return b.pMin == pMin && b.pMax == pMax;
		}
		bool operator!=(const Bounds3& b) const
		{
			return b.pMin != pMin || b.pMax != pMax;
		}
		const Eigen::Vector3d& operator[](int i) const
		{
			//DCHECK(i == 0 || i == 1);
			return (i == 0) ? pMin : pMax;
		}
		Eigen::Vector3d& operator[](int i)
		{
			//DCHECK(i == 0 || i == 1);
			return (i == 0) ? pMin : pMax;
		}
		Eigen::Vector3d Corner(int corner) const
		{
			//DCHECK(corner >= 0 && corner < 8);
			return Eigen::Vector3d((*this)[(corner & 1)].x(),
					(*this)[(corner & 2) ? 1 : 0].y(),
					(*this)[(corner & 4) ? 1 : 0].z());
		}
		Eigen::Vector3d Diagonal() const
		{
			return pMax - pMin;
		}
		double SurfaceArea() const
		{
			Eigen::Vector3d d = Diagonal();
			return 2 * (d.x() * d.y() + d.x() * d.z() + d.y() * d.z());
		}
		double Volume() const
		{
			Eigen::Vector3d d = Diagonal();
			return d.x() * d.y() * d.z();
		}
		int MaximumExtent() const
		{
			Eigen::Vector3d d = Diagonal();
			if (d.x() > d.y() && d.x() > d.z())
				return 0;
			else if (d.y() > d.z())
				return 1;
			else
				return 2;
		}
		Eigen::Vector3d Offset(const Eigen::Vector3d& p) const
		{
			Eigen::Vector3d o = p - pMin;
			if (pMax.x() > pMin.x()) o.x() /= pMax.x() - pMin.x();
			if (pMax.y() > pMin.y()) o.y() /= pMax.y() - pMin.y();
			if (pMax.z() > pMin.z()) o.z() /= pMax.z() - pMin.z();
			return o;
		}
		inline bool IntersectP(const Ray& ray, double* hitt0 = nullptr, double* hitt1 = nullptr) const;
		inline bool IntersectP(const Ray& ray, const Eigen::Vector3d& invDir, const int dirIsNeg[3]) const;
	};

	bool Bounds3::IntersectP(const Ray& ray, double* hitt0, double* hitt1) const
	{
		double t0 = 0, t1 = ray.tMax;
		for (int i = 0; i < 3; ++i)
		{
			// Update interval for _i_th bounding box slab
			double invRayDir = 1 / ray.direction[i];
			double tNear = (pMin[i] - ray.origin[i]) * invRayDir;
			double tFar = (pMax[i] - ray.origin[i]) * invRayDir;

			// Update parametric interval from slab intersection $t$ values
			if (tNear > tFar) std::swap(tNear, tFar);

			// Update _tFar_ to ensure robust ray--bounds intersection
			tFar *= 1 + 2 * gamma(3);
			t0 = tNear > t0 ? tNear : t0;
			t1 = tFar < t1 ? tFar : t1;
			if (t0 > t1) return false;
		}
		if (hitt0) *hitt0 = t0;
		if (hitt1) *hitt1 = t1;
	}
	bool Bounds3::IntersectP(const Ray& ray, const Eigen::Vector3d& invDir, const int* dirIsNeg) const //tmd是你小子
	{
		const Bounds3& bounds = *this;

		double t1, t2, tmin = 0, tmax = ZR::Infinity;
		t1 = (bounds[1].x() - ray.origin.x()) * invDir.x();
		t2 = (bounds[0].x() - ray.origin.x()) * invDir.x();
		if (t2 < t1) std::swap(t1, t2);
		tmin = std::max(tmin, t1);
		tmax = std::min(tmax, t2);

		t1 = (bounds[1].y() - ray.origin.y()) * invDir.y();
		t2 = (bounds[0].y() - ray.origin.y()) * invDir.y();
		if (t2 < t1) std::swap(t1, t2);
		tmin = std::max(tmin, t1);
		tmax = std::min(tmax, t2);

		t1 = (bounds[1].z() - ray.origin.z()) * invDir.z();
		t2 = (bounds[0].z() - ray.origin.z()) * invDir.z();
		if (t2 < t1) std::swap(t1, t2);
		tmin = std::max(tmin, t1);
		tmax = std::min(tmax, t2);

		return ((tmax - tmin) > 1e-6);

//		// Check for ray intersection against $x$ and $y$ slabs
//		double tMin = (bounds[dirIsNeg[0]].x() - ray.origin.x()) * invDir.x();
//		double tMax = (bounds[1 - dirIsNeg[0]].x() - ray.origin.x()) * invDir.x();
//		double tyMin = (bounds[dirIsNeg[1]].y() - ray.origin.y()) * invDir.y();
//		double tyMax = (bounds[1 - dirIsNeg[1]].y() - ray.origin.y()) * invDir.y();
//
//		// Update _tMax_ and _tyMax_ to ensure robust bounds intersection
//		if (tMin > tyMax || tyMin > tMax) return false;
//		if (tyMin > tMin) tMin = tyMin;
//		if (tyMax < tMax) tMax = tyMax;
//
//		// Check for ray intersection against $z$ slab
//		double tzMin = (bounds[dirIsNeg[2]].z() - ray.origin.z()) * invDir.z();
//		double tzMax = (bounds[1 - dirIsNeg[2]].z() - ray.origin.z()) * invDir.z();
//
//		// Update _tzMax_ to ensure robust bounds intersection
//		if (tMin > tzMax || tzMin > tMax) return false;
//		if (tzMin > tMin) tMin = tzMin;
//		if (tzMax < tMax) tMax = tzMax;
//		return (tMin < ray.tMax) && (tMax > 0);
	}

	Bounds3 Union(const Bounds3& b, const Eigen::Vector3d& p)
	{
		Bounds3 ret;
		ret.pMin = min(b.pMin, p);
		ret.pMax = max(b.pMax, p);
		return ret;
	}
	Bounds3 Union(const Bounds3& b1, const Bounds3& b2)
	{
		Bounds3 ret;
		ret.pMin = min(b1.pMin, b2.pMin);
		ret.pMax = max(b1.pMax, b2.pMax);
		return ret;
	}
}