

#include "MatteMaterial.hpp"
#include "../core/spectrum.hpp"
#include "reflection.hpp"

namespace ZR
{
	void
	MatteMaterial::ComputeScatteringFunctions(SurfaceInteraction* si, TransportMode mode, bool allowMultipleLobes) const
	{
		// Evaluate textures for _MatteMaterial_ material and allocate BRDF
		si->bsdf = std::make_shared<BSDF>(*si);

		Spectrum r = Kd->Evaluate(*si).Clamp();
		double sig = Clamp(sigma->Evaluate(*si), 0, 90);
		if (!r.IsBlack())
		{
			if (sig == 0)
				si->bsdf->Add(new LambertianReflection(r));
		}
	}
}