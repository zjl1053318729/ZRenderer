#pragma once

#include <Eigen/Eigen>
#include "geometry.hpp"

namespace ZR
{
	class Transform
	{
	public:
		// Transform Public Methods
		Transform()
		{
		}
		Transform(const double mat[4][4])
		{
			m = Eigen::Matrix4d();
			m << mat[0][0], mat[0][1], mat[0][2], mat[0][3], mat[1][0],
					mat[1][1], mat[1][2], mat[1][3], mat[2][0], mat[2][1],
					mat[2][2], mat[2][3], mat[3][0], mat[3][1], mat[3][2],
					mat[3][3];
			mInv = m.inverse();
		}
		Transform(const Eigen::Matrix4d& m) : m(m), mInv(m.inverse())
		{
		}
		Transform(const Eigen::Matrix4d& m, const Eigen::Matrix4d& mInv) : m(m), mInv(mInv)
		{
		}
		friend Transform Inverse(const Transform& t)
		{
			return Transform(t.mInv, t.m);
		}
		friend Transform Transpose(const Transform& t)
		{
			return Transform(t.m.transpose(), t.mInv.transpose());
		}
		bool operator==(const Transform& t) const
		{
			return t.m == m && t.mInv == mInv;
		}
		bool operator!=(const Transform& t) const
		{
			return t.m != m || t.mInv != mInv;
		}
		bool operator<(const Transform& t2) const
		{
			for (int i = 0; i < 4; ++i)
				for (int j = 0; j < 4; ++j)
				{
					if (m(i, j) < t2.m(i, j)) return true;
					if (m(i, j) > t2.m(i, j)) return false;
				}
			return false;
		}
		bool IsIdentity() const
		{
			return m.isIdentity();
		}
		const Eigen::Matrix4d& GetMatrix() const
		{
			return m;
		}
		const Eigen::Matrix4d& GetInverseMatrix() const
		{
			return mInv;
		}
		bool HasScale() const
		{

			double la = fabs((*this)(Eigen::Vector3d(1, 0, 0)).norm());
			double lb = fabs((*this)(Eigen::Vector3d(0, 1, 0)).norm());
			double lc = fabs((*this)(Eigen::Vector3d(0, 0, 1)).norm());
			return (fabs(la - 1.0) > 1e-6 || fabs(lb - 1.0) > 1e-6 || fabs(lc - 1.0) > 1e-6);
		}
		inline Eigen::Vector3d operator()(const Eigen::Vector3d& p) const;
		Bounds3 operator()(const Bounds3& b) const;
		inline Ray operator()(const Ray& r) const;
		Transform operator*(const Transform& t2) const;
		bool SwapsHandedness() const;
	private:
		// Transform Private Data
		Eigen::Matrix4d m, mInv;
	};

	Eigen::Vector3d Transform::operator()(const Eigen::Vector3d& p) const
	{
		double x, y, z;
		x = p.x();
		y = p.y();
		z = p.z();
		return Eigen::Vector3d(m(0, 0) * x + m(0, 1) * y + m(0, 2) * z,
				m(1, 0) * x + m(1, 1) * y + m(1, 2) * z,
				m(2, 0) * x + m(2, 1) * y + m(2, 2) * z);
	}
	Ray Transform::operator()(const Ray& r) const
	{
		Eigen::Vector3d o, d;
		o = (*this)(r.origin);
		d = (*this)(r.direction);
		double tMax = r.tMax;
		return Ray(o, d, tMax, r.time);
	}
	Bounds3 Transform::operator()(const Bounds3& b) const
	{
		const Transform& M = *this;
		Bounds3 ret(M(Eigen::Vector3d(b.pMin.x(), b.pMin.y(), b.pMin.z())));
		ret = Union(ret, M(Eigen::Vector3d(b.pMax.x(), b.pMin.y(), b.pMin.z())));
		ret = Union(ret, M(Eigen::Vector3d(b.pMin.x(), b.pMax.y(), b.pMin.z())));
		ret = Union(ret, M(Eigen::Vector3d(b.pMin.x(), b.pMin.y(), b.pMax.z())));
		ret = Union(ret, M(Eigen::Vector3d(b.pMin.x(), b.pMax.y(), b.pMax.z())));
		ret = Union(ret, M(Eigen::Vector3d(b.pMax.x(), b.pMax.y(), b.pMin.z())));
		ret = Union(ret, M(Eigen::Vector3d(b.pMax.x(), b.pMin.y(), b.pMax.z())));
		ret = Union(ret, M(Eigen::Vector3d(b.pMax.x(), b.pMax.y(), b.pMax.z())));
		return ret;
	}
	bool Transform::SwapsHandedness() const
	{
		double det = m(0, 0) * (m(1, 1) * m(2, 2) - m(1, 2) * m(2, 1)) -
					 m(0, 1) * (m(1, 0) * m(2, 2) - m(1, 2) * m(2, 0)) +
					 m(0, 2) * (m(1, 0) * m(2, 1) - m(1, 1) * m(2, 0));
		return det < 0;
	}
}