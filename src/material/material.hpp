#pragma once

#include "../core/ZRender.hpp"

namespace ZR
{
	// TransportMode Declarations
	enum class TransportMode
	{
		Radiance, Importance
	};

	// Material Declarations
	class Material
	{
	public:
		// Material Interface
		virtual void ComputeScatteringFunctions(SurfaceInteraction* si,
				TransportMode mode,
				bool allowMultipleLobes) const = 0;
		virtual ~Material()
		{
		}
		bool is_specular;
	};
}