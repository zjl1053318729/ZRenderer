#pragma once

#include "material.hpp"
#include "../texture/texture.hpp"

namespace ZR
{
	// MatteMaterial Declarations
	class MatteMaterial : public Material
	{
	public:
		// MatteMaterial Public Methods
		MatteMaterial(const std::shared_ptr<Texture<Spectrum>>& Kd,
				const std::shared_ptr<Texture<double>>& sigma,
				const std::shared_ptr<Texture<double>>& bumpMap)
				: Kd(Kd), sigma(sigma), bumpMap(bumpMap)
		{
			is_specular = false;
		}
		void ComputeScatteringFunctions(SurfaceInteraction* si,
				TransportMode mode,
				bool allowMultipleLobes) const;

	private:
		// MatteMaterial Private Data
		std::shared_ptr<Texture<Spectrum>> Kd;
		std::shared_ptr<Texture<double>> sigma, bumpMap;
	};


}