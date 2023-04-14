#pragma once

#include "transform.hpp"

namespace ZR
{
	struct CameraSample
	{
		Eigen::Vector2d pFilm;
		Eigen::Vector2d pLens;
		double time;
	};

	class Camera
	{
	public:
		// Camera Interface
		Camera(const Transform& CameraToWorld) : CameraToWorld(CameraToWorld)
		{
		}
		virtual ~Camera()
		{
		}
		virtual double GenerateRay(const CameraSample& sample, Ray* ray) const;

		Transform CameraToWorld;
	};

	class ProjectiveCamera : public Camera
	{
	public:
		// ProjectiveCamera Public Methods
		ProjectiveCamera(const int RasterWidth, const int RasterHeight, const Transform& CameraToWorld,
				const Transform& CameraToScreen,
				const Bounds2& screenWindow, double lensr, double focald);

	protected:
		// ProjectiveCamera Protected Data
		Transform CameraToScreen, RasterToCamera;
		Transform ScreenToRaster, RasterToScreen;
		double lensRadius, focalDistance;
	};

	Camera*
	CreatePerspectiveCamera(const int RasterWidth, const int RasterHeight, const Transform& cam2world);
	OrthographicCamera*
	CreateOrthographicCamera(const int RasterWidth, const int RasterHeight, const Transform& cam2world);

}