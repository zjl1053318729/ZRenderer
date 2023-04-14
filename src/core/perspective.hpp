#pragma once

#include <Eigen/Eigen>
#include "camera.hpp"

namespace ZR
{
	class PerspectiveCamera : public ProjectiveCamera
	{
	public:
		// PerspectiveCamera Public Methods
		PerspectiveCamera(const int RasterWidth, const int RasterHeight, const Transform& CameraToWorld,
				const Bounds2& screenWindow, double lensRadius, double focalDistance,
				double fov);
		double GenerateRay(const CameraSample& sample, Ray*) const;
	};
}