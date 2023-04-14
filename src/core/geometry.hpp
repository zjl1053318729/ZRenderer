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
		bool HasNaNs();
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
		const Eigen::Vector2d& operator[](int i) const
		{
			return (i == 0) ? pMin : pMax;
		}
		Eigen::Vector2d& operator[](int i)
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
		Eigen::Vector2d Diagonal() const;
		double Area() const;
		int MaximumExtent() const;
		Eigen::Vector2d Lerp(double t, const Eigen::Vector2d& v0, const Eigen::Vector2d& v1);
		Eigen::Vector2d Lerp(const Eigen::Vector2d& t) const;
		Eigen::Vector2d Offset(const Eigen::Vector2d& p) const;
		bool Inside(const Eigen::Vector2d& pt, const Bounds2& b) const;
		double Distance(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) const;
		void BoundingSphere(Eigen::Vector2d* c, double* rad) const;
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
		const Eigen::Vector2i& operator[](int i) const
		{
			return (i == 0) ? pMin : pMax;
		}
		Eigen::Vector2i& operator[](int i)
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
		Eigen::Vector2i Diagonal() const;
		int Area() const;
		int MaximumExtent() const;
//		Eigen::Vector2i Lerp(double t, const Eigen::Vector2i& v0, const Eigen::Vector2i& v1)
//		{
//			return v0 * (1 - t) + v1 * t;
//		}
		Eigen::Vector2i Lerp(const Eigen::Vector2i& t) const;
		Eigen::Vector2i Offset(const Eigen::Vector2i& p) const;
		bool Inside(const Eigen::Vector2i& pt, const Bounds2i& b) const;
		double Distance(const Eigen::Vector2i& p1, const Eigen::Vector2i& p2) const;
		void BoundingSphere(Eigen::Vector2i* c, double* rad) const;
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
			return (i == 0) ? pMin : pMax;
		}
		Eigen::Vector3d& operator[](int i)
		{
			return (i == 0) ? pMin : pMax;
		}
		Eigen::Vector3d Corner(int corner) const;
		Eigen::Vector3d Diagonal() const;
		double SurfaceArea() const;
		double Volume() const;
		int MaximumExtent() const;
		Eigen::Vector3d Offset(const Eigen::Vector3d& p) const;
		bool IntersectP(const Ray& ray, double* hitt0 = nullptr, double* hitt1 = nullptr) const;
		bool IntersectP(const Ray& ray, const Eigen::Vector3d& invDir, const int dirIsNeg[3]) const;
	};

	Bounds3 Union(const Bounds3& b, const Eigen::Vector3d& p);
	Bounds3 Union(const Bounds3& b1, const Bounds3& b2);
}