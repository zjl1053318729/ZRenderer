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
		return Eigen::Vector3d(m(0, 0) * x + m(0, 1) * y + m(0, 2) * z + m(0, 3),
				m(1, 0) * x + m(1, 1) * y + m(1, 2) * z + m(1, 3),
				m(2, 0) * x + m(2, 1) * y + m(2, 2) * z + m(2, 3));
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
	Transform Transform::operator*(const Transform& t2) const
	{
		return Transform(m * t2.m, t2.mInv * mInv);
	}
	Transform Translate(const Eigen::Vector3d& delta)
	{
		Eigen::Matrix4d m;
		m << 1, 0, 0, delta.x(),
				0, 1, 0, delta.y(),
				0, 0, 1, delta.z(),
				0, 0, 0, 1;
		Eigen::Matrix4d minv;
		minv << 1, 0, 0, -delta.x(),
				0, 1, 0, -delta.y(),
				0, 0, 1, -delta.z(),
				0, 0, 0, 1;
		return Transform(m, minv);
	}
	Transform Scale(float x, float y, float z)
	{
		Eigen::Matrix4d m;
		m << x, 0, 0, 0,
				0, y, 0, 0,
				0, 0, z, 0,
				0, 0, 0, 1;
		Eigen::Matrix4d minv;
		minv << 1 / x, 0, 0, 0,
				0, 1 / y, 0, 0,
				0, 0, 1 / z, 0,
				0, 0, 0, 1;
		return Transform(m, minv);
	}
	Transform RotateX(float theta)
	{
		float sinTheta = std::sin(Radians(theta));
		float cosTheta = std::cos(Radians(theta));
		Eigen::Matrix4d m;
		m << 1, 0, 0, 0,
				0, cosTheta, -sinTheta, 0,
				0, sinTheta, cosTheta, 0,
				0, 0, 0, 1;
		return Transform(m, m.transpose());
	}
	Transform RotateY(float theta)
	{
		float sinTheta = std::sin(Radians(theta));
		float cosTheta = std::cos(Radians(theta));
		Eigen::Matrix4d m;
		m << cosTheta, 0, sinTheta, 0,
				0, 1, 0, 0,
				-sinTheta, 0, cosTheta, 0,
				0, 0, 0, 1;
		return Transform(m, m.transpose());
	}
	Transform RotateZ(float theta)
	{
		float sinTheta = std::sin(Radians(theta));
		float cosTheta = std::cos(Radians(theta));
		Eigen::Matrix4d m;
		m << cosTheta, -sinTheta, 0, 0,
				sinTheta, cosTheta, 0, 0,
				0, 0, 1, 0,
				0, 0, 0, 1;
		return Transform(m, m.transpose());
	}
	Transform Rotate(float theta, const Eigen::Vector3d& axis)
	{
		Eigen::Vector3d a = axis.normalized();
		float sinTheta = std::sin(Radians(theta));
		float cosTheta = std::cos(Radians(theta));
		Eigen::Matrix4d m;
		// Compute rotation of first basis vector
		m(0, 0) = a.x() * a.x() + (1 - a.x() * a.x()) * cosTheta;
		m(0, 1) = a.x() * a.y() * (1 - cosTheta) - a.z() * sinTheta;
		m(0, 2) = a.x() * a.z() * (1 - cosTheta) + a.y() * sinTheta;
		m(0, 3) = 0;

		// Compute rotations of second and third basis vectors
		m(1, 0) = a.x() * a.y() * (1 - cosTheta) + a.z() * sinTheta;
		m(1, 1) = a.y() * a.y() + (1 - a.y() * a.y()) * cosTheta;
		m(1, 2) = a.y() * a.z() * (1 - cosTheta) - a.x() * sinTheta;
		m(1, 3) = 0;

		m(2, 0) = a.x() * a.z() * (1 - cosTheta) - a.y() * sinTheta;
		m(2, 1) = a.y() * a.z() * (1 - cosTheta) + a.x() * sinTheta;
		m(2, 2) = a.z() * a.z() + (1 - a.z() * a.z()) * cosTheta;
		m(2, 3) = 0;
		return Transform(m, m.transpose());
	}
	Transform LookAt(const Eigen::Vector3d& pos, const Eigen::Vector3d& look, const Eigen::Vector3d& up)
	{
		Eigen::Matrix4d cameraToWorld;
		// Initialize fourth column of viewing matrix
		cameraToWorld(0, 3) = pos.x();
		cameraToWorld(1, 3) = pos.y();
		cameraToWorld(2, 3) = pos.z();
		cameraToWorld(3, 3) = 1;

		// Initialize first three columns of viewing matrix
		Eigen::Vector3d dir = (look - pos).normalized();
		if (up.normalized().cross(dir).norm() == 0)
		{
			//wrong
			return Transform();
		}
		Eigen::Vector3d right = up.normalized().cross(dir).normalized();
		Eigen::Vector3d newUp = dir.cross(right);
		cameraToWorld(0, 0) = right.x();
		cameraToWorld(1, 0) = right.y();
		cameraToWorld(2, 0) = right.z();
		cameraToWorld(3, 0) = 0;
		cameraToWorld(0, 1) = newUp.x();
		cameraToWorld(1, 1) = newUp.y();
		cameraToWorld(2, 1) = newUp.z();
		cameraToWorld(3, 1) = 0;
		cameraToWorld(0, 2) = dir.x();
		cameraToWorld(1, 2) = dir.y();
		cameraToWorld(2, 2) = dir.z();
		cameraToWorld(3, 2) = 0;
		return Transform(cameraToWorld.inverse(), cameraToWorld);
	}
	Transform Orthographic(float zNear, float zFar) {
		return Scale(1, 1, 1 / (zFar - zNear)) * Translate(Eigen::Vector3d(0, 0, -zNear));
	}

	Transform Perspective(float fov, float n, float f)
	{
		// Perform projective divide for perspective projection
		Eigen::Matrix4d persp;
		persp << 1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, f / (f - n), -f * n / (f - n),
				0, 0, 1, 0;

		// Scale canonical perspective view to specified field of view
		float invTanAng = 1 / std::tan(Radians(fov) / 2);
		return Scale(invTanAng, invTanAng, 1) * Transform(persp);
	}
}