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
		plyInfo(std::string filePath)
		{
			std::fstream f(filePath);
			std::string ed;
			for (int i = 0; i < 2; i++)
			{
				f >> ed;
				if (ed == "vertex") f >> nVertices;
				else if (ed == "face") f >> nTriangles;
			}

			vertexArray = (Eigen::Vector3d*)malloc(sizeof(Eigen::Vector3d) * nVertices);
			//vertexArray = new Eigen::Vector3d[nVertices];
			vertexIndices = new int[3 * nTriangles];

			for (int i = 0; i < nVertices; i++)
			{
				f >> vertexArray[i][0];
				f >> vertexArray[i][1];
				f >> vertexArray[i][2];
				vertexArray[i] *= 20;
			}
			for (int i = 0; i < nTriangles; i++)
			{
				f >> ed;
				f >> vertexIndices[i * 3 + 0];
				f >> vertexIndices[i * 3 + 1];
				f >> vertexIndices[i * 3 + 2];
			}
		}

	};
}