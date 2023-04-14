#pragma once

#include <algorithm>
#include <limits>
#include <Eigen/Eigen>
#include <pcg_random.hpp>
#include <random>

namespace ZR
{
	template<int nSpectrumSamples>
	class CoefficientSpectrum;

	class RGBSpectrum;

	typedef RGBSpectrum Spectrum;

	class Transform;

	class Ray;

	class Shape;

	struct Interaction;

	class SurfaceInteraction;

	class Primitive;

	class GeometricPrimitive;

	class Aggregate;

	class Camera;

	class ProjectiveCamera;

	class PerspectiveCamera;

	class OrthographicCamera;

	class Sampler;

	class PixelSampler;

	class GlobalSampler;

	class HaltonSampler;

	class ClockRandSampler;

	class Sampler;

	class Scene;

	class BxDF;

	class BSDF;

	class Material;

	static constexpr double Pi = 3.14159265358979323846;
	static constexpr double InvPi = 0.31830988618379067154;
	static constexpr double Inv2Pi = 0.15915494309189533577;
	static constexpr double Inv4Pi = 0.07957747154594766788;
	static constexpr double PiOver2 = 1.57079632679489661923;
	static constexpr double PiOver4 = 0.78539816339744830961;
	static constexpr double Sqrt2 = 1.41421356237309504880;
	static constexpr double MachineEpsilon = 1e-6;
	static double Infinity = std::numeric_limits<double>::max();
	constexpr double Radians(double deg)
	{
		return (Pi / 180) * deg;
	}
	Eigen::Vector3d min(Eigen::Vector3d _a, Eigen::Vector3d _b);
	Eigen::Vector3d max(Eigen::Vector3d _a, Eigen::Vector3d _b);
	double random_double(double _l = 0.0, double _r = 1.0);
	int random_int(int _l = 0, int _r = 1);
	constexpr double gamma(int n)
	{
		return (n * MachineEpsilon) / (1 - n * MachineEpsilon);
	}
	Eigen::Vector3d abs(const Eigen::Vector3d& v);
	int MaxDimension(Eigen::Vector3d v);
	Eigen::Vector3d Permute(const Eigen::Vector3d& v, int x, int y, int z);
	double Clamp(double val, double low, double high);
	double Lerp(double t, double a, double b);
	int Mod(int a, int b);
	double MaxComponent(Eigen::Vector3d _v);

}