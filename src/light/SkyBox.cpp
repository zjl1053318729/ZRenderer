#include "SkyBox.hpp"

#define STB_IMAGE_IMPLEMENTATION

#include "../ext/stb/stb_image.h"

namespace ZR
{
	Spectrum HDRtoLDR(Spectrum& c, double exposure)
	{
		double InvExposure = 1.0 / (1.0 - exposure);
		double temp_c[3];
		temp_c[0] = 1.0 - exp(-c[0] * InvExposure);
		temp_c[1] = 1.0 - exp(-c[1] * InvExposure);
		temp_c[2] = 1.0 - exp(-c[2] * InvExposure);
		return Spectrum::FromRGB(temp_c);
	}
	void get_sphere_uv(const Eigen::Vector3d& p, double& u, double& v)
	{
		double phi = atan2(p.z(), p.x());
		double theta = asin(p.y());
		u = 1 - (phi + Pi) * Inv2Pi;
		v = (theta + PiOver2) * InvPi;
	}

	bool SkyBoxLight::loadImage(const char* imageFile)
	{
		stbi_set_flip_vertically_on_load(true);
		data = stbi_loadf(imageFile, &imageWidth, &imageHeight, &nrComponents, 0);
		if (data)return true;
		else return false;
	}
	Spectrum SkyBoxLight::getLightValue(double u, double v) const
	{
		int w = u * imageWidth, h = v * imageHeight;
		int offset = (w + h * imageWidth) * nrComponents;
		Spectrum Lv;
		// 防止环境光太亮
		const double scale = 0.1;
		Lv[0] = data[offset + 0] * scale;
		Lv[1] = data[offset + 1] * scale;
		Lv[2] = data[offset + 2] * scale;
		return Lv;
	}
	Spectrum SkyBoxLight::Sample_Li(const Interaction& ref, const Eigen::Vector2d& u, Eigen::Vector3d* wi,
			double* pdf, VisibilityTester* vis) const
	{
		double theta = u.y() * Pi, phi = u.x() * 2 * Pi;
		double cosTheta = std::cos(theta), sinTheta = std::sin(theta);
		double sinPhi = std::sin(phi), cosPhi = std::cos(phi);
		*wi = LightToWorld(Eigen::Vector3d(sinTheta * cosPhi, sinTheta * sinPhi, cosTheta));
		*pdf = 1.f / (4 * Pi);
		*vis = VisibilityTester(ref, Interaction(ref.position + *wi * (2 * worldRadius), ref.time));
		return getLightValue(u.x(), u.y());
	}
	Spectrum SkyBoxLight::Le(const Ray& ray) const
	{

		Eigen::Vector3d oc = ray.origin - worldCenter;
		double a = ray.direction.dot(ray.direction);
		double b = 2.0 * oc.dot(ray.direction);
		double c = oc.dot(oc) - worldRadius * worldRadius;
		double discriminant = b * b - 4 * a * c;
		double t;
		if (discriminant < 0)
			return 0.f;

		t = (-b + sqrt(discriminant)) / (2.0 * a);

		Eigen::Vector3d hitPos = ray.origin + t * ray.direction;
		//相对于球心偏移后的交点位置
		Eigen::Vector3d hitPos_temp = hitPos - worldCenter;

		double u, v;
		get_sphere_uv(hitPos_temp / worldRadius, u, v);

		Spectrum Col;
		if (data)
		{
			Col = getLightValue(u, v);
		}
		else
		{
			Col[0] = (hitPos_temp.x() + worldRadius) / (2.f * worldRadius);
			Col[1] = (hitPos_temp.y() + worldRadius) / (2.f * worldRadius);
			Col[2] = (hitPos_temp.z() + worldRadius) / (2.f * worldRadius);
		}
		return ZR::HDRtoLDR(Col, 0.9);
	}
	void SkyBoxLight::generatePhoton(Eigen::Vector3d& ori, Eigen::Vector3d& dir, double& powScale)
	{

	}

}