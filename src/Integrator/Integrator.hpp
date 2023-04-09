#pragma once


#include <memory>
#include "../core/camera.hpp"
#include "../sampler/sampler.hpp"
#include "../core/scene.hpp"
#include "../core/spectrum.hpp"
#include "../core/buffer.hpp"

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

	void SamplerIntegrator::Render(const Scene& scene, double& timeConsume)
	{
		Eigen::Vector3d Light(0, 10, 0);
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
				pixel_sampler->StartPixel(pixel);

				ZR::CameraSample cs;
				cs = pixel_sampler->GetCameraSample(pixel);
				ZR::Ray r;
				camera->GenerateRay(cs, &r);
				ZR::SurfaceInteraction isect;
				ZR::Spectrum colObj(0.0);
				//std::cerr<<r.direction.x()<<" "<<r.direction.y()<<" "<<r.direction.z()<<"\n";

				if (scene.Intersect(r, &isect)) {
					//std::cerr<<"wtf\n";
					Eigen::Vector3d LightNorm = (Light - isect.position).normalized();
					Eigen::Vector3d viewInv = -r.direction;
					//半程向量
					Eigen::Vector3d H = (viewInv + LightNorm).normalized();
					//高光
					double Ls = H.dot(isect.normal); Ls = (Ls > 0.0) ? Ls : 0.0;
					Ls = pow(Ls, 20);
					//漫反射光
					double Ld = LightNorm.dot(isect.normal); Ld = (Ld > 0.0) ? Ld : 0.0;
					double Li = (0.2 + 0.2 * Ld + 0.6 * Ls);
					colObj[1]=1.0;
					colObj = std::fabs(Li) * colObj; //取绝对值，防止出现负值
				}
				//std::cerr<<"wtf????\n";
				m_FrameBuffer(i,j)=colObj;

			}
			//std::cout << "\n";
		}
	}
}