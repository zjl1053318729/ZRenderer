#pragma once

#include "../core/interaction.hpp"

namespace ZR
{
	template<typename T>
	class Texture
	{
	public:
		// Texture Interface
		virtual T Evaluate(const SurfaceInteraction&) const = 0;
		virtual ~Texture()
		{
		}
	};
}