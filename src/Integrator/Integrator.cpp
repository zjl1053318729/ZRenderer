

#include "Integrator.hpp"
#include "../sampler/sampler.hpp"
#include "../core/interaction.hpp"
#include "../core/scene.hpp"
#include "../material/material.hpp"
#include "../material/reflection.hpp"
#include "../light/light.hpp"
#include <thread>
#include <mutex>

namespace ZR
{
	std::mutex mtx;
	void SamplerIntegrator::Render(const Scene& scene, double& timeConsume)
	{
		Preprocess(scene, *sampler);
		time_t st, ed;
		st = clock();
		Eigen::Vector3d a1 = scene.WorldBound().pMin, a2 = scene.WorldBound().pMax;
		std::cerr << a1.x() << " " << a1.y() << " " << a1.z() << "\n";
		std::cerr << a2.x() << " " << a2.y() << " " << a2.z() << "\n";

		int progress = 0;
		const int num_threads = 64, thread_interval = pixelBounds.pMax.y() * pixelBounds.pMax.x() / num_threads;
		std::thread th[num_threads];
		auto rendering = [&](int l1, int r1){
			for(int pix=l1;pix<=r1;++pix)
			{
				int ii,jj;
				jj = pix / pixelBounds.pMax.x();
				ii = pix % pixelBounds.pMax.x();

				int offset = (pixelBounds.pMax.x() * jj + ii);

				std::unique_ptr<ZR::Sampler> pixel_sampler = sampler->Clone(offset);
				Eigen::Vector2i pixel(jj, ii);
				ZR::Spectrum colObj(0.0);
				pixel_sampler->StartPixel(pixel);

				ZR::CameraSample cs;
				ZR::Ray r;
				do
				{
					cs = pixel_sampler->GetCameraSample(pixel);
					camera->GenerateRay(cs, &r);
					colObj += Li(r, scene, *pixel_sampler, 0);
				}while(pixel_sampler->StartNextSample());

				colObj /= (double)pixel_sampler->samplesPerPixel;
				(*m_FrameBuffer)(ii, jj) = colObj;
				mtx.lock();
				progress++;
				if(progress%100==0) std::cerr<<100.0 * progress / (pixelBounds.pMax.y() * pixelBounds.pMax.x())<<"\n";
				mtx.unlock();
			}
		};
		for(int i = 0; i < num_threads; i++)
		{
			if(i < (num_threads - 1))
				th[i] = std::thread(rendering, i * thread_interval, (i+1) * thread_interval-1);
			else
				th[i] = std::thread(rendering, i * thread_interval, pixelBounds.pMax.y() * pixelBounds.pMax.x() - 1);
		}
		for(int i = 0; i < num_threads; i++)
		{
			th[i].join();
		}


//		int cntt = 0;
//		//Eigen::Vector3d Light(10, 10, -10);
//		for (int i = 0; i < pixelBounds.pMax.x(); i++)
//		{
//			//std::cerr<<i<<"\n";
//			for (int j = 0; j < pixelBounds.pMax.y(); j++)
//			{
//
//				double u = double(i + ZR::random_double()) / double(pixelBounds.pMax.x());
//				double v = double(j + ZR::random_double()) / double(pixelBounds.pMax.y());
//				int offset = (pixelBounds.pMax.x() * j + i);
//
//				std::unique_ptr<ZR::Sampler> pixel_sampler = sampler->Clone(offset);
//				Eigen::Vector2i pixel(j, i);
//				ZR::Spectrum colObj(0.0);
//				pixel_sampler->StartPixel(pixel);
//
//				ZR::CameraSample cs;
//				ZR::Ray r;
//				do
//				{
//					cs = pixel_sampler->GetCameraSample(pixel);
//					camera->GenerateRay(cs, &r);
//					colObj += Li(r, scene, *pixel_sampler, 0);
//				}while(pixel_sampler->StartNextSample());
//
//				colObj /= (double)pixel_sampler->samplesPerPixel;
//				(*m_FrameBuffer)(i, j) = colObj;
//
//				//std::cerr << 100.0 * j / pixelBounds.pMax.y() << "\n";
//			}
//			std::cerr << 100.0 * i / pixelBounds.pMax.x() << "\n";
//			//std::cout << "\n";
//
//		}
		ed = clock();
		timeConsume = (ed - st) / 1000.0;
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

	Spectrum UniformSampleAllLights(const Interaction& it, const Scene& scene, Sampler& sampler,
			const std::vector<int>& nLightSamples, bool handleMedia)
	{
		Spectrum L(0.f);
		for (size_t j = 0; j < scene.lights.size(); ++j)
		{
			// Accumulate contribution of _j_th light to _L_
			const std::shared_ptr<Light>& light = scene.lights[j];
			int nSamples = nLightSamples[j];
			const Eigen::Vector2d* uLightArray = sampler.Get2DArray(nSamples);
			const Eigen::Vector2d* uScatteringArray = sampler.Get2DArray(nSamples);
			if (!uLightArray || !uScatteringArray)
			{
				// Use a single sample for illumination from _light_
				Eigen::Vector2d uLight = sampler.Get2D();
				Eigen::Vector2d uScattering = sampler.Get2D();
				L += EstimateDirect(it, uScattering, *light, uLight, scene, sampler, handleMedia);
			}
			else
			{
				// Estimate direct lighting using sample arrays
				Spectrum Ld(0.f);
				for (int k = 0; k < nSamples; ++k)
					Ld += EstimateDirect(it, uScatteringArray[k], *light,
							uLightArray[k], scene, sampler, handleMedia);
				L += Ld / nSamples;
			}
		}
		return L;
	}
	Spectrum UniformSampleOneLight(const Interaction& it, const Scene& scene, Sampler& sampler,
			bool handleMedia, const Distribution1D* lightDistrib)
	{

		// Randomly choose a single light to sample, _light_
		int nLights = int(scene.lights.size());
		if (nLights == 0) return Spectrum(0.f);
		int lightNum;
		double lightPdf;
		if (lightDistrib)
		{
			lightNum = lightDistrib->SampleDiscrete(sampler.Get1D(), &lightPdf);
			if (lightPdf == 0) return Spectrum(0.f);
		}
		else
		{
			lightNum = std::min((int)(sampler.Get1D() * nLights), nLights - 1);
			lightPdf = double(1) / nLights;
		}

		const std::shared_ptr<Light>& light = scene.lights[lightNum];
		Eigen::Vector2d uLight = sampler.Get2D();
		Eigen::Vector2d uScattering = sampler.Get2D();

		return EstimateDirect(it, uScattering, *light, uLight,
				scene, sampler, handleMedia) / lightPdf;
	}

	Spectrum EstimateDirect(const Interaction& it, const Eigen::Vector2d& uScattering, const Light& light,
			const Eigen::Vector2d& uLight, const Scene& scene, Sampler& sampler, bool handleMedia, bool specular)
	{
		BxDFType bsdfFlags =
				specular ? BSDF_ALL : BxDFType(BSDF_ALL & ~BSDF_SPECULAR);
		Spectrum Ld(0.f);
		// Sample light source with multiple importance sampling
		Eigen::Vector3d wi;

		double lightPdf = 0, scatteringPdf = 0;

		VisibilityTester visibility;
		Spectrum Li = light.Sample_Li(it, uLight, &wi, &lightPdf, &visibility);
		if (lightPdf > 0 && !Li.IsBlack())
		{
			// Compute BSDF or phase function's value for light sample
			Spectrum f;
			if (it.IsSurfaceInteraction())
			{
				// Evaluate BSDF for light sampling strategy
				const SurfaceInteraction& isect = (const SurfaceInteraction&)it;
				f = isect.bsdf->f(isect.wo, wi, bsdfFlags) *
					std::fabs(wi.dot(isect.shading.n));
				scatteringPdf = isect.bsdf->Pdf(isect.wo, wi, bsdfFlags);
			}
			if (!f.IsBlack())
			{
				if (!visibility.Unoccluded(scene))
				{
					Li = Spectrum(0.f);
				}
				// Add light's contribution to reflected radiance
				if (!Li.IsBlack())
				{
					if (IsDeltaLight(light.flags))
						Ld += f * Li / lightPdf;
					else
					{
						double weight = PowerHeuristic(1, lightPdf, 1, scatteringPdf);
						Ld += f * Li * weight / lightPdf;
					}
				}
			}
		}

		// Sample BSDF with multiple importance sampling
		if (!IsDeltaLight(light.flags))
		{
			Spectrum f;
			bool sampledSpecular = false;
			if (it.IsSurfaceInteraction())
			{
				// Sample scattered direction for surface interactions
				BxDFType sampledType;
				const SurfaceInteraction& isect = (const SurfaceInteraction&)it;
				f = isect.bsdf->Sample_f(isect.wo, &wi, uScattering, &scatteringPdf,
						bsdfFlags, &sampledType);
				f *= std::fabs(wi.dot(isect.shading.n));
				sampledSpecular = (sampledType & BSDF_SPECULAR) != 0;
			}
			if (!f.IsBlack() && scatteringPdf > 0)
			{
				// Account for light contributions along sampled direction _wi_
				double weight = 1;
				if (!sampledSpecular)
				{
					lightPdf = light.Pdf_Li(it, wi);
					if (lightPdf == 0) return Ld;
					weight = PowerHeuristic(1, scatteringPdf, 1, lightPdf);
				}
				// Find intersection and compute transmittance
				SurfaceInteraction lightIsect;
				Ray ray = it.SpawnRay(wi);
				bool foundSurfaceInteraction = scene.Intersect(ray, &lightIsect);

				// Add light contribution from material sampling
				Spectrum Li(0.f);
				if (foundSurfaceInteraction)
				{
					if (lightIsect.primitive->GetAreaLight() == &light)
						Li = lightIsect.Le(-wi);
				}
				else
					Li = light.Le(ray);
				if (!Li.IsBlack()) Ld += f * Li * weight / scatteringPdf;
			}
		}

		return Ld;
	}
}