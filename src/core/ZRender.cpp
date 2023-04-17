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
		return std::max(_v.x(), std::max(_v.y(), _v.z()));
	}
	uint64_t FloatToBits(double f)
	{
		uint64_t ui;
		memcpy(&ui, &f, sizeof(double));
		return ui;
	}

	double BitsToFloat(uint64_t ui)
	{
		double f;
		memcpy(&f, &ui, sizeof(uint64_t));
		return f;
	}
	double NextFloatUp(double v)
	{
		// Handle infinity and negative zero for _NextFloatUp()_
		if (std::isinf(v) && v > 0.) return v;
		if (v == -0.f) v = 0.f;

		// Advance _v_ to next higher double
		uint64_t ui = FloatToBits(v);
		if (v >= 0)
			++ui;
		else
			--ui;
		return BitsToFloat(ui);
	}
	double NextFloatDown(double v)
	{
		// Handle infinity and positive zero for _NextFloatDown()_
		if (std::isinf(v) && v < 0.) return v;
		if (v == 0.f) v = -0.f;
		uint64_t ui = FloatToBits(v);
		if (v > 0)
			--ui;
		else
			++ui;
		return BitsToFloat(ui);
	}
	Eigen::Vector3d FaceForward(Eigen::Vector3d a, Eigen::Vector3d b)
	{
		if (a.dot(b) < 0.0) return -b;
		return b;
	}
	bool isZero(double x)
	{
		return std::fabs(x) < 1e-6;
	}
}