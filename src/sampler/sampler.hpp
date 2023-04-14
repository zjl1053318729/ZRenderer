#pragma once

#include <memory>
#include "../core/ZRender.hpp"
#include "../core/camera.hpp"
#include "RNG.hpp"


namespace ZR
{
	class Sampler
	{
	public:
		// Sampler Interface
		virtual ~Sampler()
		{
		};
		Sampler(int64_t samplesPerPixel) : samplesPerPixel(samplesPerPixel)
		{
		};
		virtual void StartPixel(const Eigen::Vector2i& p);
		virtual double Get1D() = 0;
		virtual Eigen::Vector2d Get2D() = 0;
		CameraSample GetCameraSample(const Eigen::Vector2i& pRaster);
		void Request1DArray(int n);
		void Request2DArray(int n);
		virtual int RoundCount(int n) const;
		const double* Get1DArray(int n);
		const Eigen::Vector2d* Get2DArray(int n);
		virtual bool StartNextSample();
		virtual std::unique_ptr<Sampler> Clone(int seed) = 0;
		virtual bool SetSampleNumber(int64_t sampleNum);

		int64_t CurrentSampleNumber() const;
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
}