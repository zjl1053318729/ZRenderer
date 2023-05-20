#include <thread>
#include <mutex>
#include "PhotonTracer.hpp"
#include "../core/scene.hpp"
#include "../sampler/sampler.hpp"
#include "../material/reflection.hpp"

namespace ZR
{
	void PhotonTracer::Render(Scene& scene, double& timeConsume)
	{
		std::mutex mtx;
		time_t st1,ed1;
		Preprocess(scene, *sampler);
		st1 = clock();
		worldInit_PhotonMap(scene);
		ed1 = clock();
		time_t st, ed;
		st = clock();
//		Eigen::Vector3d a1 = scene.WorldBound().pMin, a2 = scene.WorldBound().pMax;
//		std::cerr << a1.x() << " " << a1.y() << " " << a1.z() << "\n";
//		std::cerr << a2.x() << " " << a2.y() << " " << a2.z() << "\n";

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

				colObj /= pixel_sampler->samplesPerPixel;

				(*m_FrameBuffer)(ii, jj) = colObj;
				mtx.lock();
				progress++;
				if(progress%6400==0) std::cerr<<100.0 * progress / (pixelBounds.pMax.y() * pixelBounds.pMax.x())<<"\n";
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

		ed = clock();
		timeConsume = (ed - st) / 1000.0;
		std::cerr<<"construction Time: "<< (ed1-st1)/1000.0<<" s\n";
	}

	void
	PhotonTracer::traceGlobalPhoton(const Ray& r, Scene& scene, int depth, Spectrum Power, PhotonMap* mPhotonMap)
	{
		SurfaceInteraction hrec;
		if (scene.Intersect(r, &hrec))
		{
			Photon pn;
			pn.Pos = hrec.position;
			pn.Dir = r.direction;
			pn.power = Power * 0.6;
			mPhotonMap->store(pn);

			if(depth >= 3) return;

			hrec.ComputeScatteringFunctions(r);
			Eigen::Vector3d wi;
			double pdf;
			hrec.bsdf->Sample_f(hrec.wo, &wi, sampler->Get2D(), &pdf);

			traceGlobalPhoton(Ray(hrec.position, wi.normalized()), scene, depth + 1, Power * 0.4, mPhotonMap);
		}
	}

	void PhotonTracer::traceCausticsPhoton(const Ray& r, Scene& scene, int depth, Spectrum Power,
			PhotonMap* mPhotonMap)
	{
		//记录击中信息，例如击中点，表面向量，纹理坐标等
		SurfaceInteraction hrec;
		if (scene.Intersect(r, &hrec))
		{
			//记录当前点散射信息，比如是否为镜面反射等

			if (depth < 4)
			{
				if (hrec.is_specular)
				{
					Eigen::Vector3d wi;
					wi = hrec.wo - 2.0 * hrec.normal.dot(-hrec.wo) * hrec.normal;
					traceCausticsPhoton(Ray(hrec.position, wi.normalized()), scene, depth + 1, Power, mPhotonMap);
				}
				else
				{
					if (depth != 0) return;
					else
					{
						Photon pn;
						pn.Pos = hrec.position;
						pn.Dir = r.direction;
						pn.power = Power;
						mPhotonMap->store(pn);
					}
				}
			}
		}
	}
	void PhotonTracer::worldInit_PhotonMap(Scene& scene)
	{
		Spectrum Power(energyPerPhoton);
		Eigen::Vector3d Origin, Dir;
		double PowScale;

		mGlobalPhotonMap = new PhotonMap(globalPhotonsNum * 4 + causticsPhotonsNum);
		int idx;
		while (mGlobalPhotonMap->PhotonNum < globalPhotonsNum)
		{
			idx = ZR::random_int(0, scene.lights.size()-1);
			scene.lights[idx]->generatePhoton(Origin, Dir, PowScale);
			Ray r(Origin, Dir);
			traceGlobalPhoton(r, scene, 0, PowScale * Power, mGlobalPhotonMap);
		}
		// 只捕捉焦散光子
		while (mGlobalPhotonMap->PhotonNum < globalPhotonsNum + causticsPhotonsNum)
		{
			//std::cerr<<mGlobalPhotonMap->PhotonNum<<"\n";
			idx = ZR::random_int(0, scene.lights.size()-1);
			scene.lights[idx]->generatePhoton(Origin, Dir, PowScale);
			Ray r(Origin, Dir);
			traceCausticsPhoton(r, scene, 0,
					PowScale * Spectrum(Power[0] * 0.87, Power[1] * 0.49, Power[2] * 0.173),
					mGlobalPhotonMap);
		}
		mGlobalPhotonMap->balance();
	}
	Spectrum PhotonTracer::Li(const Ray& ray, const Scene& scene, Sampler& sampler, int depth) const
	{
		SurfaceInteraction hrec;
		if (scene.Intersect(ray, &hrec))
		{
			Spectrum emitted = hrec.Le(-ray.direction);
			if(!emitted.IsBlack())
				return emitted;
			if (depth < 10)
			{
				Eigen::Vector3d wi;
				if (hrec.is_specular)
				{
					wi = 2.0 * hrec.normal.dot(-hrec.wo) * hrec.normal - hrec.normal;
					return Li(Ray(hrec.position, wi.normalized()), scene, sampler, depth + 1);
				}
				else
				{
					Spectrum col = mGlobalPhotonMap->getIrradiance(hrec.position, hrec.normal, 0.1, 90);
					hrec.ComputeScatteringFunctions(ray);
					double pdf;
					BxDFType flags;
					return col * hrec.bsdf->Sample_f(hrec.wo, &wi, sampler.Get2D(), &pdf, BSDF_ALL, &flags);
				}
			}
		}
		else
			return Spectrum();
	}
}
