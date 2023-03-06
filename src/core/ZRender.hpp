#pragma once

#include <limits>
#include <Eigen/Eigen>
#include <pcg_random.hpp>
#include <random>

namespace ZR
{
	static constexpr double Pi = 3.14159265358979323846;
	static constexpr double InvPi = 0.31830988618379067154;
	static constexpr double Inv2Pi = 0.15915494309189533577;
	static constexpr double Inv4Pi = 0.07957747154594766788;
	static constexpr double PiOver2 = 1.57079632679489661923;
	static constexpr double PiOver4 = 0.78539816339744830961;
	static constexpr double Sqrt2 = 1.41421356237309504880;
	static constexpr double MachineEpsilon = 1e-6;
	inline constexpr double Radians(double deg)
	{
		return (Pi / 180) * deg;
	}
	double Infinity = std::numeric_limits<double>::max();
	inline Eigen::Vector3d min(Eigen::Vector3d _a, Eigen::Vector3d _b)
	{
		double x, y, z;
		x = std::min(_a.x(), _b.x());
		y = std::min(_a.y(), _b.y());
		z = std::min(_a.z(), _b.z());
		return Eigen::Vector3d(x, y, z);
	}
	inline Eigen::Vector3d max(Eigen::Vector3d _a, Eigen::Vector3d _b)
	{
		double x, y, z;
		x = std::max(_a.x(), _b.x());
		y = std::max(_a.y(), _b.y());
		z = std::max(_a.z(), _b.z());
		return Eigen::Vector3d(x, y, z);
	}
	inline double random_double(double _l = 0.0, double _r = 1.0)
	{
		// Seed with a real random value, if available
		static pcg_extras::seed_seq_from<std::random_device> seed_source;

		// Make a random number engine
		static pcg32 rng(seed_source);

		std::uniform_real_distribution<> uniform_dist(_l, _r);
		return uniform_dist(rng);
	}
	inline int random_int(int _l = 0, int _r = 1)
	{
		// Seed with a real random value, if available
		static pcg_extras::seed_seq_from<std::random_device> seed_source;

		// Make a random number engine
		static pcg32 rng(seed_source);

		std::uniform_int_distribution<> uniform_dist(_l, _r);
		return uniform_dist(rng);
	}
	inline constexpr double gamma(int n)
	{
		return (n * MachineEpsilon) / (1 - n * MachineEpsilon);
	}
	Eigen::Vector3d abs(const Eigen::Vector3d& v)
	{
		return Eigen::Vector3d(fabs(v.x()), fabs(v.y()), fabs(v.z()));
	}
	int MaxDimension(Eigen::Vector3d v)
	{
		return (v.x() > v.y()) ? ((v.x() > v.z()) ? 0 : 2) : ((v.y() > v.z()) ? 1 : 2);
	}
	Eigen::Vector3d Permute(const Eigen::Vector3d& v, int x, int y, int z)
	{
		return Eigen::Vector3d(v[x], v[y], v[z]);
	}

	class Transform;

	class Ray;

	class Shape;

	struct Interaction;

	class SurfaceInteraction;

	class Primitive;

	class GeometricPrimitive;

	class Aggregate;
}