#pragma once

#include "Integrator.hpp"
#include "../core/Photon.hpp"

namespace ZR
{
	class PhotonTracer : public SamplerIntegrator
	{
	public:
		PhotonTracer(std::shared_ptr<const Camera> camera,
				std::shared_ptr<Sampler> sampler,
				const Bounds2i& pixelBounds, Buffer* m_FrameBuffer,double energy = 100, int glo = 100000, int cau = 10000)
				: SamplerIntegrator(camera, sampler, pixelBounds, m_FrameBuffer)
		{
			globalPhotonsNum = glo;
			causticsPhotonsNum = cau;
			sumEnergy = energy;
			energyPerPhoton = sumEnergy / (glo + cau);
		}
		void traceGlobalPhoton(const Ray&r, Scene& scene, int depth, Spectrum Power, PhotonMap* mPhotonMap);
		void traceCausticsPhoton(const Ray&r, Scene& scene, int depth, Spectrum Power, PhotonMap* mPhotonMap);
		void  worldInit_PhotonMap(Scene& scene);
		Spectrum Li(const Ray& ray, const Scene& scene, Sampler& sampler, int depth = 0) const;
		virtual void Render(Scene& scene, double& timeConsume);
	private:
		int globalPhotonsNum, causticsPhotonsNum;
		double sumEnergy, energyPerPhoton;
		PhotonMap * mGlobalPhotonMap;
		PhotonMap * mCausticsPhotonMap;
	};
}