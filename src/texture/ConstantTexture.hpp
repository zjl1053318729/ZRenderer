#pragma once

#include "texture.hpp"

namespace ZR
{
	template<typename T>
	class ConstantTexture : public Texture<T>
	{
	public:
		// ConstantTexture Public Methods
		ConstantTexture(const T& value) : value(value)
		{
		}
		T Evaluate(const SurfaceInteraction&) const
		{
			return value;
		}

	private:
		T value;
	};
}