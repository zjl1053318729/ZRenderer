#pragma once

#include <Eigen/Eigen>
#include "geometry.hpp"

namespace ZR
{
	class Transform
	{
	public:
		Transform()
		{
			m << 1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1;
			mInv = m;
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
		bool operator==(const Transform& t) const;
		bool operator!=(const Transform& t) const;
		bool operator<(const Transform& t2) const;
		bool IsIdentity() const;
		const Eigen::Matrix4d& GetMatrix() const;
		const Eigen::Matrix4d& GetInverseMatrix() const;
		bool HasScale() const;
		Eigen::Vector3d operator()(const Eigen::Vector3d& p) const;
		Bounds3 operator()(const Bounds3& b) const;
		Ray operator()(const Ray& r) const;
		Transform operator*(const Transform& t2) const;
		bool SwapsHandedness() const;
	private:
		Eigen::Matrix4d m, mInv;
	};

	Transform Translate(const Eigen::Vector3d& delta);
	Transform Scale(double x, double y, double z);
	Transform RotateX(double theta);
	Transform RotateY(double theta);
	Transform RotateZ(double theta);
	Transform Rotate(double theta, const Eigen::Vector3d& axis);
	Transform LookAt(const Eigen::Vector3d& pos, const Eigen::Vector3d& look, const Eigen::Vector3d& up);
	Transform Orthographic(double zNear, double zFar);
	Transform Perspective(double fov, double n, double f);
}