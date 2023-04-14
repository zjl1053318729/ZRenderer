#pragma once

#include <Eigen/Eigen>
#include <memory>
#include "../shape/shape.hpp"
#include "../material/material.hpp"
#include "primitive.hpp"


namespace ZR
{
	class BSDF;

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
		~SurfaceInteraction();
		void ComputeScatteringFunctions(
				const Ray& ray,
				bool allowMultipleLobes = false,
				TransportMode mode = TransportMode::Radiance);

		const Primitive* primitive = nullptr;
		std::shared_ptr<BSDF> bsdf = nullptr;
		Eigen::Vector2d uv;
		Eigen::Vector3d dpdu, dpdv;
		Eigen::Vector3d dndu, dndv;
		const Shape* shape = nullptr;
		struct _shading
		{
			Eigen::Vector3d n;
			Eigen::Vector3d dpdu, dpdv;
			Eigen::Vector3d dndu, dndv;
		}shading;
	};


}