#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "../shape/shape.hpp"
#include "../material/material.hpp"
#include "primitive.hpp"


namespace ZR
{
	class BSDF;

	Eigen::Vector3d OffsetRayOrigin(const Eigen::Vector3d& p, const Eigen::Vector3d& pError,
			const Eigen::Vector3d& n, const Eigen::Vector3d& w);

	struct Interaction
	{
		Eigen::Vector3d position, wo, normal, pError;
		double time;

		Interaction() : time(0)
		{
		}
		Interaction(const Eigen::Vector3d& _p, const Eigen::Vector3d& _n, const Eigen::Vector3d& _pError,
				const Eigen::Vector3d& _wo, double _time)
				: position(_p),
				  time(_time),
				  pError(_pError),
				  wo(_wo.normalized()),
				  normal(_n)
		{
		}
		Interaction(const Eigen::Vector3d& p, const Eigen::Vector3d& wo, double time)
				: position(p), time(time), wo(wo)
		{
		}
		Interaction(const Eigen::Vector3d& p, double time)
				: position(p), time(time)
		{
		}
		Ray SpawnRay(const Eigen::Vector3d& d) const
		{
			Eigen::Vector3d o = OffsetRayOrigin(position, pError, normal, d);
			return Ray(o, d, Infinity, time);
		}
		Ray SpawnRayTo(const Eigen::Vector3d& p2) const
		{
			Eigen::Vector3d origin = OffsetRayOrigin(position, pError, normal, p2 - position);
			Eigen::Vector3d d = p2 - position;
			return Ray(origin, d, 1 - 1e-6, time);
		}
		Ray SpawnRayTo(const Interaction& it) const
		{
			Eigen::Vector3d origin = OffsetRayOrigin(position, pError, normal, it.position - position);
			Eigen::Vector3d target = OffsetRayOrigin(it.position, it.pError, it.normal, origin - it.position);
			Eigen::Vector3d d = target - origin;
			return Ray(origin, d, 1 - 1e-6, time);
		}
		bool IsSurfaceInteraction() const
		{
			return !(isZero(normal.x()) && isZero(normal.y()) && isZero(normal.z()));
		}
	};


	class SurfaceInteraction : public Interaction
	{
	public:
		// SurfaceInteraction Public Methods
		SurfaceInteraction() = default;
		SurfaceInteraction(const Eigen::Vector3d& p, const Eigen::Vector3d& pError,
				const Eigen::Vector2d& uv, const Eigen::Vector3d& wo,
				const Eigen::Vector3d& dpdu, const Eigen::Vector3d& dpdv,
				const Eigen::Vector3d& dndu, const Eigen::Vector3d& dndv, double time,
				const Shape* sh,
				int faceIndex = 0);
		//~SurfaceInteraction();
		void ComputeScatteringFunctions(
				const Ray& ray,
				bool allowMultipleLobes = false,
				TransportMode mode = TransportMode::Radiance);
		Spectrum Le(const Eigen::Vector3d& w) const;

		const Primitive* primitive = nullptr;
		std::shared_ptr<BSDF> bsdf = nullptr;
		Eigen::Vector2d uv;
		Eigen::Vector3d dpdu, dpdv;
		Eigen::Vector3d dndu, dndv;
		const Shape* shape = nullptr;
		struct
		{
			Eigen::Vector3d n;
			Eigen::Vector3d dpdu, dpdv;
			Eigen::Vector3d dndu, dndv;
		} shading;
	};


}