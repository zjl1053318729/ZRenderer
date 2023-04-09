#pragma once

#include "spectrum.hpp"

namespace ZR
{
	class Buffer
	{
	public:
		Buffer()
		{
			HEIGHT = 1, WIDTH = 1;
			buffer = new RGBSpectrum[HEIGHT*WIDTH];
		}
		Buffer(int _H, int _W)
		{
			HEIGHT = _H, WIDTH = _W;
			buffer = new RGBSpectrum[HEIGHT*WIDTH];
		}
		RGBSpectrum& operator()(int _i ,int _j)
		{
			return buffer[_i*WIDTH + _j];
		}
	private:
		int HEIGHT, WIDTH;
		RGBSpectrum* buffer;
	};
}