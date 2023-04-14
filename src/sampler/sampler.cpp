

#include "sampler.hpp"

namespace ZR
{
	void Sampler::StartPixel(const Eigen::Vector2i& p)
	{
		currentPixel = p;
		currentPixelSampleIndex = 0;
		// Reset array offsets for next pixel sample
		array1DOffset = array2DOffset = 0;
	}
	bool Sampler::StartNextSample()
	{
		// Reset array offsets for next pixel sample
		array1DOffset = array2DOffset = 0;
		return ++currentPixelSampleIndex < samplesPerPixel;
	}
	bool Sampler::SetSampleNumber(int64_t sampleNum)
	{
		// Reset array offsets for next pixel sample
		array1DOffset = array2DOffset = 0;
		currentPixelSampleIndex = sampleNum;
		return currentPixelSampleIndex < samplesPerPixel;
	}
	void Sampler::Request1DArray(int n)
	{
		samples1DArraySizes.push_back(n);
		sampleArray1D.push_back(std::vector<double>(n * samplesPerPixel));
	}
	void Sampler::Request2DArray(int n)
	{
		samples2DArraySizes.push_back(n);
		sampleArray2D.push_back(std::vector<Eigen::Vector2d>(n * samplesPerPixel));
	}
	const double* Sampler::Get1DArray(int n)
	{
		if (array1DOffset == sampleArray1D.size()) return nullptr;
		return &sampleArray1D[array1DOffset++][currentPixelSampleIndex * n];
	}
	const Eigen::Vector2d* Sampler::Get2DArray(int n)
	{
		if (array2DOffset == sampleArray2D.size()) return nullptr;
		return &sampleArray2D[array2DOffset++][currentPixelSampleIndex * n];
	}
	CameraSample Sampler::GetCameraSample(const Eigen::Vector2i& pRaster)
	{
		CameraSample cs;
		Eigen::Vector2d tmp(pRaster.x(), pRaster.y());
		cs.pFilm = tmp + Get2D();
		cs.time = Get1D();
		cs.pLens = Get2D();
		return cs;
	}
	int64_t Sampler::CurrentSampleNumber() const
	{
		return currentPixelSampleIndex;
	}
	int Sampler::RoundCount(int n) const
	{
		return n;
	}
	PixelSampler::PixelSampler(int64_t samplesPerPixel, int nSampledDimensions)
			: Sampler(samplesPerPixel)
	{
		for (int i = 0; i < nSampledDimensions; ++i)
		{
			samples1D.push_back(std::vector<double>(samplesPerPixel));
			samples2D.push_back(std::vector<Eigen::Vector2d>(samplesPerPixel));
		}
	}
	bool PixelSampler::StartNextSample()
	{
		current1DDimension = current2DDimension = 0;
		return Sampler::StartNextSample();
	}
	bool PixelSampler::SetSampleNumber(int64_t sampleNum)
	{
		current1DDimension = current2DDimension = 0;
		return Sampler::SetSampleNumber(sampleNum);
	}
	double PixelSampler::Get1D()
	{
		if (current1DDimension < samples1D.size())
			return samples1D[current1DDimension++][currentPixelSampleIndex];
		else
			return rng.UniformFloat64();
	}
	Eigen::Vector2d PixelSampler::Get2D()
	{
		if (current2DDimension < samples2D.size())
			return samples2D[current2DDimension++][currentPixelSampleIndex];
		else
			return Eigen::Vector2d(rng.UniformFloat64(), rng.UniformFloat64());
	}
	void GlobalSampler::StartPixel(const Eigen::Vector2i& p)
	{
		Sampler::StartPixel(p);
		dimension = 0;
		intervalSampleIndex = GetIndexForSample(0);
		// Compute _arrayEndDim_ for dimensions used for array samples
		arrayEndDim =
				arrayStartDim + sampleArray1D.size() + 2 * sampleArray2D.size();

		// Compute 1D array samples for _GlobalSampler_
		for (size_t i = 0; i < samples1DArraySizes.size(); ++i)
		{
			int nSamples = samples1DArraySizes[i] * samplesPerPixel;
			for (int j = 0; j < nSamples; ++j)
			{
				int64_t index = GetIndexForSample(j);
				sampleArray1D[i][j] = SampleDimension(index, arrayStartDim + i);
			}
		}

		// Compute 2D array samples for _GlobalSampler_
		int dim = arrayStartDim + samples1DArraySizes.size();
		for (size_t i = 0; i < samples2DArraySizes.size(); ++i)
		{
			int nSamples = samples2DArraySizes[i] * samplesPerPixel;
			for (int j = 0; j < nSamples; ++j)
			{
				int64_t idx = GetIndexForSample(j);
				sampleArray2D[i][j].x() = SampleDimension(idx, dim);
				sampleArray2D[i][j].y() = SampleDimension(idx, dim + 1);
			}
			dim += 2;
		}
	}
	bool GlobalSampler::StartNextSample()
	{
		dimension = 0;
		intervalSampleIndex = GetIndexForSample(currentPixelSampleIndex + 1);
		return Sampler::StartNextSample();
	}
	bool GlobalSampler::SetSampleNumber(int64_t sampleNum)
	{
		dimension = 0;
		intervalSampleIndex = GetIndexForSample(sampleNum);
		return Sampler::SetSampleNumber(sampleNum);
	}
	double GlobalSampler::Get1D()
	{
		if (dimension >= arrayStartDim && dimension < arrayEndDim)
			dimension = arrayEndDim;
		return SampleDimension(intervalSampleIndex, dimension++);
	}
	Eigen::Vector2d GlobalSampler::Get2D()
	{
		if (dimension + 1 >= arrayStartDim && dimension < arrayEndDim)
			dimension = arrayEndDim;
		Eigen::Vector2d p(SampleDimension(intervalSampleIndex, dimension),
				SampleDimension(intervalSampleIndex, dimension + 1));
		dimension += 2;
		return p;
	}
}