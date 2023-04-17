#pragma once

#include <memory>
#include "material.hpp"
#include "../texture/texture.hpp"

namespace ZR
{
	// MirrorMaterial Declarations
	class MirrorMaterial : public Material
	{
	public:
		// MirrorMaterial Public Methods
		MirrorMaterial(const std::shared_ptr<Texture<Spectrum>>& r,
				const std::shared_ptr<Texture<double>>& bump)
		{
			Kr = r;
			bumpMap = bump;
		}
		void ComputeScatteringFunctions(SurfaceInteraction* si, TransportMode mode,
				bool allowMultipleLobes) const;

	private:
		// MirrorMaterial Private Data
		std::shared_ptr<Texture<Spectrum>> Kr;
		std::shared_ptr<Texture<double>> bumpMap;
	};
}