

#include "Integrator.hpp"
#include "../sampler/sampler.hpp"
#include "../core/interaction.hpp"
#include "../core/scene.hpp"
#include "../material/material.hpp"
#include "../material/reflection.hpp"
#include "../light/light.hpp"

namespace ZR
{
	void SamplerIntegrator::Render(const Scene& scene, double& timeConsume)
	{
		time_t st,ed;
		st = clock();
		Eigen::Vector3d a1 = scene.WorldBound().pMin, a2 = scene.WorldBound().pMax;
		std::cerr << a1.x() << " " << a1.y() << " " << a1.z() << "\n";
		std::cerr << a2.x() << " " << a2.y() << " " << a2.z() << "\n";

		int cntt = 0;
		//Eigen::Vector3d Light(10, 10, -10);
		for (int i = 0; i < pixelBounds.pMax.x(); i++)
		{
			//std::cerr<<i<<"\n";
			for (int j = 0; j < pixelBounds.pMax.y(); j++)
			{
				double u = double(i + ZR::random_double()) / double(pixelBounds.pMax.x());
				double v = double(j + ZR::random_double()) / double(pixelBounds.pMax.y());
				int offset = (pixelBounds.pMax.x() * j + i);

				std::unique_ptr<ZR::Sampler> pixel_sampler = sampler->Clone(offset);
				Eigen::Vector2i pixel(j, i);
				ZR::Spectrum colObj(0.0);
				pixel_sampler->StartPixel(pixel);

				ZR::CameraSample cs;
				ZR::Ray r;
				//colObj = Li(r, scene, *pixel_sampler, 0);

				do
				{
					cs = pixel_sampler->GetCameraSample(pixel);
					camera->GenerateRay(cs, &r);
					colObj += Li(r, scene, *pixel_sampler, 0);
				}while(pixel_sampler->StartNextSample());

				colObj /= (double)pixel_sampler->samplesPerPixel;


				m_FrameBuffer(i, j) = colObj;

			}
			//std::cout << "\n";
		}
		ed = clock();
		timeConsume = (ed-st)/1000.0;
		std::cerr << "cntt " << cntt << "\n";
	}
	Spectrum SamplerIntegrator::SpecularReflect(
			const Ray& ray, const SurfaceInteraction& isect,
			const Scene& scene, Sampler& sampler, int depth) const
	{
		// Compute specular reflection direction _wi_ and BSDF value
		Eigen::Vector3d wo = isect.wo, wi;
		double pdf;
		BxDFType type = BxDFType(BSDF_REFLECTION | BSDF_SPECULAR);
		Spectrum f = isect.bsdf->Sample_f(wo, &wi, sampler.Get2D(), &pdf, type);

		// Return contribution of specular reflection
		const Eigen::Vector3d& ns = isect.shading.n;

		if (std::isnan(wi.x()) || std::isnan(wi.y()) || std::isnan(wi.z())) return 1.0;

		if (pdf > 0.f && !f.IsBlack() && (!isZero(std::fabs(wi.dot(ns)))))
		{
			// Compute ray differential _rd_ for specular reflection
			Ray rd = isect.SpawnRay(wi);
			return f * Li(rd, scene, sampler, depth + 1) * std::fabs(wi.dot(ns)) / pdf;
		}
		else
			return Spectrum(0.f);
	}
	Spectrum SamplerIntegrator::Li(const Ray& ray, const Scene& scene,
			Sampler& sampler, int depth) const
	{

		ZR::SurfaceInteraction isect;

		ZR::Spectrum colObj;
		if (scene.Intersect(ray, &isect))
		{
			for (int count = 0; count < scene.lights.size(); count++)
			{
				VisibilityTester vist;
				Eigen::Vector3d wi;
				Interaction p1;
				double pdf_light;
				Spectrum Li = scene.lights[count]->Sample_Li(isect, sampler.Get2D(), &wi, &pdf_light, &vist);

				if (vist.Unoccluded(scene))
				{
					//计算散射
					isect.ComputeScatteringFunctions(ray);
					// 对于漫反射材质来说，wo不会影响后面的结果
					Eigen::Vector3d wo = isect.wo;
					Spectrum f = isect.bsdf->f(wo, wi);
					double pdf_scattering = isect.bsdf->Pdf(wo, wi);
					colObj += Li * pdf_scattering * f * 3.0f / pdf_light;
				}
			}
			colObj /= scene.lights.size();
		}

		return colObj;
	}
}