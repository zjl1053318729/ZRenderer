

#include "Integrator.hpp"
#include "../sampler/sampler.hpp"
#include "../core/interaction.hpp"
#include "../core/scene.hpp"
#include "../material/material.hpp"
#include "../material/reflection.hpp"

namespace ZR
{
	void SamplerIntegrator::Render(const Scene& scene, double& timeConsume)
	{
		int cntt=0;
		Eigen::Vector3d Light(-10.0, 10.0, -10.0);
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

				if (scene.Intersect(r, &isect))
				{
					//计算散射
					isect.ComputeScatteringFunctions(r);
					// 对于漫反射材质来说，wo不会影响后面的结果
					Eigen::Vector3d wo = isect.wo;
					Eigen::Vector3d LightNorm = Light - isect.position;
					LightNorm = LightNorm.normalized();
					Eigen::Vector3d wi = LightNorm;
					Spectrum f = isect.bsdf->f(wo, wi);
					double pdf = isect.bsdf->Pdf(wo, wi);

					Eigen::Vector3d viewInv = -r.direction;
					Eigen::Vector3d H = (viewInv + LightNorm).normalized();
					double Ld,Ls,Li;
					Ls = H.dot(isect.normal);
					Ls = Ls > 0 ? Ls : 0.0;
					Ls = std::pow(Ls,32);
					Ld = LightNorm.dot(isect.normal);
					Ld = Ld > 0 ? Ld : 0.0;
					Li = (Ld * 0.3 + Ls * 0.7 + 0.2) * 2;
					//乘以3.0的意义是为了不让图像过暗
					colObj += pdf * f * 4.0 * Li;
				}
				//std::cerr<<"wtf????\n";
				if(fabs(colObj[0])<1e-6 && fabs(colObj[1])<1e-6 && fabs(colObj[2])<1e-6)
					cntt++;
				m_FrameBuffer(i, j) = colObj;

			}
			//std::cout << "\n";
		}
		std::cerr<<"cntt "<<cntt<<"\n";
	}
}