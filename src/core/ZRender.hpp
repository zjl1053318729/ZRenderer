#pragma once

#include <limits>
#include <Eigen/Eigen>

namespace ZR
{
	static constexpr double Pi = 3.14159265358979323846;
	static constexpr double InvPi = 0.31830988618379067154;
	static constexpr double Inv2Pi = 0.15915494309189533577;
	static constexpr double Inv4Pi = 0.07957747154594766788;
	static constexpr double PiOver2 = 1.57079632679489661923;
	static constexpr double PiOver4 = 0.78539816339744830961;
	static constexpr double Sqrt2 = 1.41421356237309504880;
	inline constexpr double Radians(float deg) { return (Pi / 180) * deg; }
	double Infinity = std::numeric_limits<double>::max();
}