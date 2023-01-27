#pragma once
#include <Eigen/Eigen>
#include "ZRender.hpp"

namespace ZR
{
	class Ray
	{
	public:
		Eigen::Vector3d origin, direction;
		mutable double tMax;
		double time;

		Ray() :tMax(ZR::Infinity), time(0.f) {}
		Ray(const Eigen::Vector3d &_o, const Eigen::Vector3d &_d, float tMax = Infinity, float time = 0.f)
				: origin(_o), direction(_d), tMax(tMax), time(time) {}
		Eigen::Vector3d operator()(float t) const
		{
			return origin + direction * t;
		}
		bool HasNaNs()
		{
			return (origin.hasNaN() || direction.hasNaN() || std::isnan(tMax));
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
		explicit Bounds3(const Eigen::Vector3d &p) : pMin(p), pMax(p) {}
		Bounds3(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
				: pMin(std::min(p1.x(), p2.x()), std::min(p1.y(), p2.y()),
					std::min(p1.z(), p2.z())),
				  pMax(std::max(p1.x(), p2.x()), std::max(p1.y(), p2.y()),
					std::max(p1.z(), p2.z())) {}

		bool operator==(const Bounds3 &b) const
		{
			return b.pMin == pMin && b.pMax == pMax;
		}
		bool operator!=(const Bounds3 &b) const
		{
			return b.pMin != pMin || b.pMax != pMax;
		}
		const Eigen::Vector3d &operator[](int i) const
		{
			//DCHECK(i == 0 || i == 1);
			return (i == 0) ? pMin : pMax;
		}
		Eigen::Vector3d &operator[](int i)
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
		Eigen::Vector3d Diagonal() const { return pMax - pMin; }
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
		Eigen::Vector3d Offset(const Eigen::Vector3d &p) const
		{
			Eigen::Vector3d o = p - pMin;
			if (pMax.x() > pMin.x()) o.x() /= pMax.x() - pMin.x();
			if (pMax.y() > pMin.y()) o.y() /= pMax.y() - pMin.y();
			if (pMax.z() > pMin.z()) o.z() /= pMax.z() - pMin.z();
			return o;
		}
		inline bool IntersectP(const Ray &ray, float *hitt0 = nullptr,float *hitt1 = nullptr) const;
		inline bool IntersectP(const Ray &ray, const Eigen::Vector3d &invDir,const int dirIsNeg[3]) const;
	};
}