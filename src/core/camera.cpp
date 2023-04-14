#include "camera.hpp"

namespace ZR
{
	ProjectiveCamera::ProjectiveCamera(const int RasterWidth, const int RasterHeight, const Transform& CameraToWorld,
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
	double Camera::GenerateRay(const CameraSample& sample, Ray* ray) const
	{
		return 1;
	};
}