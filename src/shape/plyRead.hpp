#pragma once

#include <Eigen/Eigen>
#include <fstream>

namespace ZR
{
	class plyInfo
	{
	public:
		int nVertices, nTriangles;
		Eigen::Vector3d* vertexArray;
		int* vertexIndices;
		plyInfo(std::string filePath);

	};
}