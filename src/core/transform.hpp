#pragma once

#include <Eigen/Eigen>

namespace ZR
{
	class Transform {
	public:
		// Transform Public Methods
		Transform() {}
		Transform(const double mat[4][4]) {
			m = Eigen::Matrix4d();
			m << mat[0][0], mat[0][1], mat[0][2], mat[0][3], mat[1][0],
					mat[1][1], mat[1][2], mat[1][3], mat[2][0], mat[2][1],
					mat[2][2], mat[2][3], mat[3][0], mat[3][1], mat[3][2],
					mat[3][3];
			mInv = m.inverse();
		}
		Transform(const Eigen::Matrix4d &m) : m(m), mInv(m.inverse()) {}
		Transform(const Eigen::Matrix4d &m, const Eigen::Matrix4d &mInv) : m(m), mInv(mInv) {}
		friend Transform Inverse(const Transform &t) {
			return Transform(t.mInv, t.m);
		}
		friend Transform Transpose(const Transform &t) {
			return Transform(Transpose(t.m), Transpose(t.mInv));
		}
		bool operator==(const Transform &t) const {
			return t.m == m && t.mInv == mInv;
		}
		bool operator!=(const Transform &t) const {
			return t.m != m || t.mInv != mInv;
		}
		bool operator<(const Transform &t2) const {
			for (int i = 0; i < 4; ++i)
				for (int j = 0; j < 4; ++j) {
					if (m(i, j) < t2.m(i, j)) return true;
					if (m(i, j) > t2.m(i, j)) return false;
				}
			return false;
		}
		bool IsIdentity() const
		{
			return m.isIdentity();
		}
		const Eigen::Matrix4d &GetMatrix() const { return m; }
		const Eigen::Matrix4d &GetInverseMatrix() const { return mInv; }
		bool HasScale() const
		{

			double la2 = (*this)(Eigen::Vector3d(1, 0, 0)).LengthSquared();
			double lb2 = (*this)(Eigen::Vector3d(0, 1, 0)).LengthSquared();
			double lc2 = (*this)(Eigen::Vector3d(0, 0, 1)).LengthSquared();
#define NOT_ONE(x) ((x) < .999f || (x) > 1.001f)
			return (NOT_ONE(la2) || NOT_ONE(lb2) || NOT_ONE(lc2));
#undef NOT_ONE
		}
		inline Point3<T> operator()(const Point3<T> &p) const;
		Bounds3 operator()(const Bounds3 &b) const;
		inline Ray operator()(const Ray &r) const;
		Transform operator*(const Transform &t2) const;
		bool SwapsHandedness() const;
	private:
		// Transform Private Data
		Eigen::Matrix4d m, mInv;
	};
}