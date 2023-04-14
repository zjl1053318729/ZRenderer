#pragma once


#include <memory>
#include "../core/buffer.hpp"
#include "../core/camera.hpp"

namespace ZR
{
	// Integrator Declarations
	class Integrator
	{
	public:
		// Integrator Interface
		virtual ~Integrator()
		{
		}
		virtual void Render(const Scene& scene, double& timeConsume) = 0;
		double IntegratorRenderTime; //渲染一次用的时间
	};

// SamplerIntegrator Declarations
	class SamplerIntegrator : public Integrator
	{
	public:
		// SamplerIntegrator Public Methods
		SamplerIntegrator(Camera* camera,
				std::shared_ptr<Sampler> sampler,
				const Bounds2i& pixelBounds, Buffer m_FrameBuffer)
				: camera(camera), sampler(sampler), pixelBounds(pixelBounds), m_FrameBuffer(m_FrameBuffer)
		{
		}
		virtual void Preprocess(const Scene& scene, Sampler& sampler)
		{
		}
		void Render(const Scene& scene, double& timeConsume);

	protected:
		// SamplerIntegrator Protected Data
		std::shared_ptr<const Camera> camera;

	private:
		// SamplerIntegrator Private Data
		std::shared_ptr<Sampler> sampler;
		const Bounds2i pixelBounds;
		Buffer m_FrameBuffer;
	};


}