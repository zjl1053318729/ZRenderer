

#include "perspective.hpp"
#include "../sampler/sampling.hpp"

namespace ZR
{
	PerspectiveCamera::PerspectiveCamera(const int RasterWidth, const int RasterHeight, const Transform& CameraToWorld,
			const Bounds2& screenWindow, double lensRadius, double focalDistance, double fov)
			: ProjectiveCamera(RasterWidth, RasterHeight, CameraToWorld, Perspective(fov, 1e-2f, 1000.f),
			screenWindow, lensRadius, focalDistance)
	{
		// Compute image plane bounds at $z=1$ for _PerspectiveCamera_
		Eigen::Vector2i res = Eigen::Vector2i(RasterWidth, RasterHeight);
		Eigen::Vector3d pMin = RasterToCamera(Eigen::Vector3d(0, 0, 0));
		Eigen::Vector3d pMax = RasterToCamera(Eigen::Vector3d(res.x(), res.y(), 0));
		pMin /= pMin.z();
		pMax /= pMax.z();
	}
	double PerspectiveCamera::GenerateRay(const CameraSample& sample,
			Ray* ray) const
	{
		// Compute raster and camera sample positions
		Eigen::Vector3d pFilm = Eigen::Vector3d(sample.pFilm.x(), sample.pFilm.y(), 0);
		Eigen::Vector3d pCamera = RasterToCamera(pFilm);
		*ray = Ray(Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(pCamera).normalized());
		// Modify ray for depth of field
		if (lensRadius > 0)
		{
			// Sample point on lens
			Eigen::Vector2d pLens = lensRadius * ConcentricSampleDisk(sample.pLens);

			// Compute point on plane of focus
			double ft = focalDistance / ray->direction.z();
			Eigen::Vector3d pFocus = (*ray)(ft);

			// Update ray for effect of lens
			ray->origin = Eigen::Vector3d(pLens.x(), pLens.y(), 0);
			ray->direction = (pFocus - ray->origin).normalized();
		}
		*ray = CameraToWorld(*ray);
		return 1;
	}
	Camera*
	CreatePerspectiveCamera(const int RasterWidth, const int RasterHeight, const Transform& cam2world)
	{
		double lensradius = 0.02;
		double focaldistance = 3;
		double frame = (double)RasterWidth / (double)RasterHeight;
		Bounds2 screen;
		if (frame > 1.f)
		{
			screen.pMin.x() = -frame;
			screen.pMax.x() = frame;
			screen.pMin.y() = -1.f;
			screen.pMax.y() = 1.f;
		}
		else
		{
			screen.pMin.x() = -1.f;
			screen.pMax.x() = 1.f;
			screen.pMin.y() = -1.f / frame;
			screen.pMax.y() = 1.f / frame;
		}
		double fov = 90.0f;
		double halffov = 45.0f;
		return new PerspectiveCamera(RasterWidth, RasterHeight, cam2world, screen, lensradius, focaldistance, fov);
	}
}