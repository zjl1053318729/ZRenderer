#include "ZRender.hpp"

namespace ZR
{
	Eigen::Vector3d min(Eigen::Vector3d _a, Eigen::Vector3d _b)
	{
		double x, y, z;
		x = std::min(_a.x(), _b.x());
		y = std::min(_a.y(), _b.y());
		z = std::min(_a.z(), _b.z());
		return Eigen::Vector3d(x, y, z);
	}
	Eigen::Vector3d max(Eigen::Vector3d _a, Eigen::Vector3d _b)
	{
		double x, y, z;
		x = std::max(_a.x(), _b.x());
		y = std::max(_a.y(), _b.y());
		z = std::max(_a.z(), _b.z());
		return Eigen::Vector3d(x, y, z);
	}
	double random_double(double _l, double _r)
	{
		// Seed with a real random value, if available
		static pcg_extras::seed_seq_from<std::random_device> seed_source;

		// Make a random number engine
		static pcg32 rng(seed_source);

		std::uniform_real_distribution<> uniform_dist(_l, _r);
		return uniform_dist(rng);
	}
	int random_int(int _l, int _r)
	{
		// Seed with a real random value, if available
		static pcg_extras::seed_seq_from<std::random_device> seed_source;

		// Make a random number engine
		static pcg32 rng(seed_source);

		std::uniform_int_distribution<> uniform_dist(_l, _r);
		return uniform_dist(rng);
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
	double Clamp(double val, double low, double high)
	{
		if (val < low) return low;
		if (val > high) return high;
		return val;
	}
	double Lerp(double t, double a, double b)
	{
		return a + (b - a) * t;
	}

	int Mod(int a, int b)
	{
		return ((a % b) + b) % b;
	}
	double MaxComponent(Eigen::Vector3d _v)
	{
		return std::max(_v.x(),std::max(_v.y(),_v.z()));
	}
}