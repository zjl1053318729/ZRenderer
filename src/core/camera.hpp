#pragma once

#include<Eigen/Eigen>
#include "transform.hpp"
#include "perspective.hpp"

namespace ZR
{
	struct CameraSample
	{
		Eigen::Vector2d pFilm;
		Eigen::Vector2d pLens;
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
		virtual double GenerateRay(const CameraSample& sample, Ray* ray) const
		{
			return 1;
		};

		// Camera Public Data
		Transform CameraToWorld;
	};

	class ProjectiveCamera : public Camera
	{
	public:
		// ProjectiveCamera Public Methods
		ProjectiveCamera(const int RasterWidth, const int RasterHeight, const Transform& CameraToWorld,
				const Transform& CameraToScreen,
				const Bounds2& screenWindow, double lensr, double focald)
				: Camera(CameraToWorld),
				  CameraToScreen(CameraToScreen)
		{
			// Initialize depth of field parameters
			lensRadius = lensr;
			focalDistance = focald;
			// Compute projective camera screen transformations
			ScreenToRaster =
					Scale(RasterWidth, RasterHeight, 1) *
					Scale(1 / (screenWindow.pMax.x() - screenWindow.pMin.x()),
							1 / (screenWindow.pMin.y() - screenWindow.pMax.y()), 1) *
					Translate(Eigen::Vector3d(-screenWindow.pMin.x(), -screenWindow.pMax.y(), 0));
			RasterToScreen = Inverse(ScreenToRaster);
			RasterToCamera = Inverse(CameraToScreen) * RasterToScreen;
		}

	protected:
		// ProjectiveCamera Protected Data
		Transform CameraToScreen, RasterToCamera;
		Transform ScreenToRaster, RasterToScreen;
		double lensRadius, focalDistance;
	};

	//PerspectiveCamera *CreatePerspectiveCamera(const int RasterWidth, const int RasterHeight, const Transform &cam2world);

}