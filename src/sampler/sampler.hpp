#pragma once

#include <pcg_random.hpp>
#include <Eigen/Eigen>
#include "../core/ZRender.hpp"
#include "RNG.hpp"

namespace ZR
{
	class Sampler
	{
	public:
		// Sampler Interface
		virtual ~Sampler() {};
		Sampler(int64_t samplesPerPixel): samplesPerPixel(samplesPerPixel) {};
		virtual void StartPixel(const Eigen::Vector2i& p);
		virtual double Get1D() = 0;
		virtual Eigen::Vector2d Get2D() = 0;
		CameraSample GetCameraSample(const Eigen::Vector2i& pRaster);
		void Request1DArray(int n);
		void Request2DArray(int n);
		virtual int RoundCount(int n) const
		{
			return n;
		}
		const double* Get1DArray(int n);
		const Eigen::Vector2d* Get2DArray(int n);
		virtual bool StartNextSample();
		virtual std::unique_ptr<Sampler> Clone(int seed) = 0;
		virtual bool SetSampleNumber(int64_t sampleNum);

		int64_t CurrentSampleNumber() const
		{
			return currentPixelSampleIndex;
		}
		// Sampler Public Data
		const int64_t samplesPerPixel;

	protected:
		// Sampler Protected Data
		Eigen::Vector2i currentPixel;
		int64_t currentPixelSampleIndex;
		std::vector<int> samples1DArraySizes, samples2DArraySizes;
		std::vector<std::vector<double>> sampleArray1D;
		std::vector<std::vector<Eigen::Vector2d>> sampleArray2D;

	private:
		// Sampler Private Data
		size_t array1DOffset, array2DOffset;
	};

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
		//CHECK_EQ(RoundCount(n), n);
		samples1DArraySizes.push_back(n);
		sampleArray1D.push_back(std::vector<double>(n * samplesPerPixel));
	}
	void Sampler::Request2DArray(int n)
	{
		//CHECK_EQ(RoundCount(n), n);
		samples2DArraySizes.push_back(n);
		sampleArray2D.push_back(std::vector<Eigen::Vector2d>(n * samplesPerPixel));
	}
	const double* Sampler::Get1DArray(int n)
	{
		if (array1DOffset == sampleArray1D.size()) return nullptr;
		//CHECK_EQ(samples1DArraySizes[array1DOffset], n);
		//CHECK_LT(currentPixelSampleIndex, samplesPerPixel);
		return &sampleArray1D[array1DOffset++][currentPixelSampleIndex * n];
	}
	const Eigen::Vector2d* Sampler::Get2DArray(int n)
	{
		if (array2DOffset == sampleArray2D.size()) return nullptr;
		//CHECK_EQ(samples2DArraySizes[array2DOffset], n);
		//CHECK_LT(currentPixelSampleIndex, samplesPerPixel);
		return &sampleArray2D[array2DOffset++][currentPixelSampleIndex * n];
	}
	CameraSample Sampler::GetCameraSample(const Eigen::Vector2i& pRaster)
	{
		CameraSample cs;
		Eigen::Vector2d tmp(pRaster.x(),pRaster.y());
		cs.pFilm = tmp + Get2D();
		cs.time = Get1D();
		cs.pLens = Get2D();
		return cs;
	}

	class PixelSampler : public Sampler
	{
	public:
		// PixelSampler Public Methods
		PixelSampler(int64_t samplesPerPixel, int nSampledDimensions);
		bool StartNextSample();
		bool SetSampleNumber(int64_t);
		double Get1D();
		Eigen::Vector2d Get2D();

	protected:
		// PixelSampler Protected Data
		std::vector<std::vector<double>> samples1D;
		std::vector<std::vector<Eigen::Vector2d>> samples2D;
		int current1DDimension = 0, current2DDimension = 0;
		RNG rng;
	};

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

	class GlobalSampler : public Sampler
	{
	public:
		// GlobalSampler Public Methods
		bool StartNextSample();
		void StartPixel(const Eigen::Vector2i& p);
		bool SetSampleNumber(int64_t sampleNum);
		double Get1D();
		Eigen::Vector2d Get2D();
		GlobalSampler(int64_t samplesPerPixel) : Sampler(samplesPerPixel)
		{
		}
		virtual int64_t GetIndexForSample(int64_t sampleNum) const = 0;
		virtual double SampleDimension(int64_t index, int dimension) const = 0;

	private:
		// GlobalSampler Private Data
		int dimension;
		int64_t intervalSampleIndex;
		static const int arrayStartDim = 5;
		int arrayEndDim;
	};

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