//#pragma once
//
//#include <Eigen/Eigen>
//#include "camera.hpp"
//
//namespace ZR
//{
//	// OrthographicCamera Declarations
//	class OrthographicCamera : public ProjectiveCamera
//	{
//	public:
//		// OrthographicCamera Public Methods
//		OrthographicCamera(const int RasterWidth, const int RasterHeight, const Transform& CameraToWorld,
//				const Bounds2& screenWindow, double lensRadius,
//				double focalDistance)
//				: ProjectiveCamera(RasterWidth, RasterHeight, CameraToWorld, Orthographic(0, 10), screenWindow,
//				lensRadius, focalDistance)
//		{
//		}
//		double GenerateRay(const CameraSample& sample, Ray*) const;
//	};
//}