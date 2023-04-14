

#include "interaction.hpp"
#include "../material/reflection.hpp"

namespace ZR
{
	SurfaceInteraction::SurfaceInteraction(const Eigen::Vector3d& p, const Eigen::Vector3d& pError,
			const Eigen::Vector2d& uv, const Eigen::Vector3d& wo, const Eigen::Vector3d& dpdu,
			const Eigen::Vector3d& dpdv, const Eigen::Vector3d& dndu, const Eigen::Vector3d& dndv, double time,
			const Shape* sh, int faceIndex) :
			Interaction(p, Eigen::Vector3d((dpdu.cross(dpdv)).normalized()),
					pError, wo, time),
			uv(uv),
			dpdu(dpdu),
			dpdv(dpdv),
			dndu(dndu),
			dndv(dndv),
			shape(sh)
	{
		// Initialize shading geometry from true geometry
		shading.n = normal;
		shading.dpdu = dpdu;
		shading.dpdv = dpdv;
		shading.dndu = dndu;
		shading.dndv = dndv;
	}
	void SurfaceInteraction::ComputeScatteringFunctions(const Ray& ray, bool allowMultipleLobes, TransportMode mode)
	{
		primitive->ComputeScatteringFunctions(this, mode, allowMultipleLobes);
	}
	SurfaceInteraction::~SurfaceInteraction()
	{
		if (bsdf)
			bsdf->~BSDF();
	}
}