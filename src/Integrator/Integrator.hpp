#pragma once


#include <memory>
#include "../core/buffer.hpp"
#include "../core/camera.hpp"
#include "../sampler/sampling.hpp"

namespace ZR
{
	Spectrum UniformSampleAllLights(const Interaction& it, const Scene& scene, Sampler& sampler,
			const std::vector<int>& nLightSamples, bool handleMedia = false);
	Spectrum UniformSampleOneLight(const Interaction& it, const Scene& scene,
			Sampler& sampler,
			bool handleMedia = false,
			const Distribution1D* lightDistrib = nullptr);
	Spectrum EstimateDirect(const Interaction& it, const Eigen::Vector2d& uShading,
			const Light& light, const Eigen::Vector2d& uLight,
			const Scene& scene, Sampler& sampler,
			bool handleMedia = false,
			bool specular = false);

	// Integrator Declarations
	class Integrator
	{
	public:
		// Integrator Interface
		virtual ~Integrator()
		{
		}
		virtual void Render(Scene& scene, double& timeConsume) = 0;
		double IntegratorRenderTime; //渲染一次用的时间
	};

// SamplerIntegrator Declarations
	class SamplerIntegrator : public Integrator
	{
	public:
		// SamplerIntegrator Public Methods
		SamplerIntegrator(std::shared_ptr<const Camera> camera,
				std::shared_ptr<Sampler> sampler,
				const Bounds2i& pixelBounds, Buffer* m_FrameBuffer)
				: camera(camera), sampler(sampler), pixelBounds(pixelBounds), m_FrameBuffer(m_FrameBuffer)
		{
		}
		virtual void Preprocess(const Scene& scene, Sampler& sampler)
		{
		}
		virtual void Render(Scene& scene, double& timeConsume);
		virtual Spectrum Li(const Ray& ray, const Scene& scene, Sampler& sampler, int depth = 0) const;
		Spectrum SpecularReflect(const Ray& ray,
				const SurfaceInteraction& isect,
				const Scene& scene, Sampler& sampler,
				int depth) const;

	protected:
		// SamplerIntegrator Protected Data
		std::shared_ptr<const Camera> camera;

		// SamplerIntegrator Private Data
		std::shared_ptr<Sampler> sampler;
		const Bounds2i pixelBounds;
		Buffer* m_FrameBuffer;
	};


}