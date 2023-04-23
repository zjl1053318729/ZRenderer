#include "LightDistribution.hpp"
#include "../core/scene.hpp"

namespace ZR
{
	UniformLightDistribution::UniformLightDistribution(const Scene& scene)
	{
		std::vector<double> prob(scene.lights.size(), double(1));
		distrib.reset(new Distribution1D(&prob[0], int(prob.size())));
	}

	const Distribution1D* UniformLightDistribution::Lookup(const Eigen::Vector3d& p) const
	{
		return distrib.get();
	}

	std::unique_ptr<LightDistribution>
	CreateLightSampleDistribution(const std::string& name, const Scene& scene)
	{
		return std::unique_ptr<LightDistribution>{
				new UniformLightDistribution(scene) };
	}
}