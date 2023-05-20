#pragma once

#include "../core/spectrum.hpp"
#include "light.hpp"

namespace ZR
{
	// SkyBoxLight Declarations
	class SkyBoxLight : public Light
	{
	public:
		// SkyBoxLight Public Methods
		SkyBoxLight(const Transform& LightToWorld, const Eigen::Vector3d& worldCenter, double worldRadius,
				const char* file, int nSamples)
				: Light((int)LightFlags::Infinite, LightToWorld, nSamples),
				  worldCenter(worldCenter),
				  worldRadius(worldRadius)
		{
			imageWidth = 0;
			imageHeight = 0;
			nrComponents = 0;
			data = nullptr;
			loadImage(file);
		}
		~SkyBoxLight()
		{
			if (data) free(data);
		}
		void Preprocess(const Scene& scene)
		{
		}
		bool loadImage(const char* imageFile);
		Spectrum getLightValue(double u, double v) const;
		Spectrum Power() const
		{
			return Spectrum(0.f);
		}
		Spectrum Le(const Ray& ray) const;
		Spectrum Sample_Li(const Interaction& ref, const Eigen::Vector2d& u, Eigen::Vector3d* wi,
				double* pdf, VisibilityTester* vis) const;
		double Pdf_Li(const Interaction&, const Eigen::Vector3d&) const
		{
			return 0.f;
		}
		Spectrum Sample_Le(const Eigen::Vector2d& u1, const Eigen::Vector2d& u2, double time,
				Ray* ray, Eigen::Vector3d* nLight, double* pdfPos,
				double* pdfDir) const
		{
			return Spectrum(0.f);
		}
		void Pdf_Le(const Ray&, const Eigen::Vector3d&, double* pdfPos, double* pdfDir) const
		{
		}
		void generatePhoton(Eigen::Vector3d& ori, Eigen::Vector3d& dir, double& powScale);

	private:
		// SkyBoxLight Private Data
		Eigen::Vector3d worldCenter;
		double worldRadius;
		int imageWidth, imageHeight, nrComponents;
		float* data;

	};
}