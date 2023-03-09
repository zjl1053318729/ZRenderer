#pragma once

#include <Eigen/Eigen>
#include "camera.hpp"
#include "../sampler/sampler.hpp"


namespace ZR
{
	// OrthographicCamera Declarations
	class OrthographicCamera : public ProjectiveCamera
	{
	public:
		// OrthographicCamera Public Methods
		OrthographicCamera(const int RasterWidth, const int RasterHeight, const Transform& CameraToWorld,
				const Bounds2& screenWindow, double lensRadius,
				double focalDistance)
				: ProjectiveCamera(RasterWidth, RasterHeight, CameraToWorld, Orthographic(0, 10), screenWindow,
				lensRadius, focalDistance)
		{
		}
		double GenerateRay(const CameraSample& sample, Ray*) const;
	};

	double OrthographicCamera::GenerateRay(const CameraSample& sample,
			Ray* ray) const
	{
		// Compute raster and camera sample positions
		Eigen::Vector3d pFilm = Eigen::Vector3d(sample.pFilm.x(), sample.pFilm.y(), 0);
		Eigen::Vector3d pCamera = RasterToCamera(pFilm);
		*ray = Ray(pCamera, Eigen::Vector3d(0, 0, 1));
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

	OrthographicCamera*
	CreateOrthographicCamera(const int RasterWidth, const int RasterHeight, const Transform& cam2world)
	{

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

		double ScreenScale = 2.0f;

		{
			screen.pMin.x() *= ScreenScale;
			screen.pMax.x() *= ScreenScale;
			screen.pMin.y() *= ScreenScale;
			screen.pMax.y() *= ScreenScale;
		}


		double lensradius = 0.0f;
		double focaldistance = 0.0f;
		return new OrthographicCamera(RasterWidth, RasterHeight, cam2world, screen, lensradius, focaldistance);
	}
}