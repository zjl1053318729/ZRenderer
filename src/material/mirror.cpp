#include "mirror.hpp"
#include "reflection.hpp"

namespace ZR
{
	// MirrorMaterial Method Definitions
	void MirrorMaterial::ComputeScatteringFunctions(SurfaceInteraction* si,
			TransportMode mode,
			bool allowMultipleLobes) const
	{
		// Perform bump mapping with _bumpMap_, if present
		//if (bumpMap) Bump(bumpMap, si);

		si->bsdf = std::make_shared<BSDF>(*si);

		Spectrum R = Kr->Evaluate(*si).Clamp();
		if (!R.IsBlack())
			si->bsdf->Add(new SpecularReflection(R, new FresnelNoOp));
	}
}