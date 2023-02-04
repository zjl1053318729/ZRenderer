#pragma once

#include <Eigen/Eigen>
#include "ZRender.hpp"
#include "primitive.hpp"
#include "../shape/shape.hpp"

namespace ZR
{
	struct Interaction
	{
		Eigen::Vector3d position;
		Eigen::Vector3d wo;
		Eigen::Vector3d normal;
		double time;

		Interaction() : time(0)
		{
		}
		Interaction(const Eigen::Vector3d& _p, const Eigen::Vector3d& _n, const Eigen::Vector3d& _pError,
				const Eigen::Vector3d& _wo, float _time)
				: position(_p),
				  time(_time),
				  wo(_wo.normalized()),
				  normal(_n)
		{
		}
	};

	class SurfaceInteraction : public Interaction
	{
	public:
		// SurfaceInteraction Public Methods
		SurfaceInteraction()
		{
		}
		void ComputeScatteringFunctions();
		const Shape* shape = nullptr;
		const Primitive* primitive = nullptr;
	};

}