#include <Eigen/Eigen>
#include <memory>
#include "../shape/triangle.hpp"
#include "../shape/plyRead.hpp"
#include "../accelerator/BVHAccel.hpp"
#include "../shape/sphere.hpp"
#include "../core/transform.hpp"

ZR::Transform sphereT_Object2World, sphereT_World2Object;
ZR::Shape* s = new ZR::Sphere(&sphereT_Object2World, &sphereT_World2Object, false, 1.0);

int main()
{
	freopen("result.ppm", "w", stdout);

	int WIDTH = 500;
	int HEIGHT = 500;

	std::cout << "P3\n";
	std::cout << WIDTH << " " << HEIGHT << "\n";
	std::cout << "255\n";

	// 相机参数初始化：光追三部曲的风格
	Eigen::Vector3d lower_left_corner(-2.0, -2.0, -2.0);
	Eigen::Vector3d horizontal(4.0, 0.0, 0.0);
	Eigen::Vector3d vertical(0.0, 4.0, 0.0);
	Eigen::Vector3d origin(0.0, 0.0, -4.0);

	// 生成Mesh加速结构
	std::shared_ptr<ZR::TriangleMesh> mesh;
	std::vector<std::shared_ptr<ZR::Shape>> tris;
	std::vector<std::shared_ptr<ZR::Primitive>> prims;
	ZR::plyInfo* plyi;
	ZR::Aggregate* agg;
	ZR::Transform tri_Object2World, tri_World2Object;

	tri_Object2World = ZR::Translate(Eigen::Vector3d(0.0, -2.5, 0.0)) * tri_Object2World;
	tri_World2Object = Inverse(tri_Object2World);

	std::cerr << "importing model...";
	plyi = new ZR::plyInfo("dragon.3d");
	std::cerr << "  done\n";

	mesh = std::make_shared<ZR::TriangleMesh>(tri_Object2World, plyi->nTriangles, plyi->vertexIndices, plyi->nVertices,
			plyi->vertexArray, nullptr, nullptr, nullptr, nullptr);
	tris.reserve(plyi->nTriangles);
	for (int i = 0; i < plyi->nTriangles; ++i)
		tris.push_back(std::make_shared<ZR::Triangle>(&tri_Object2World, &tri_World2Object, false, mesh, i));
	for (int i = 0; i < plyi->nTriangles; ++i)
		prims.push_back(std::make_shared<ZR::GeometricPrimitive>(tris[i]));

	std::cerr << "building accelerator...";
	agg = new ZR::BVHAccel(prims, 1);
	std::cerr << "  done\n";


	// 开始执行渲染
//	double T;
//	ZR::SurfaceInteraction isect1;
//	s->Intersect(ZR::Ray(origin, Eigen::Vector3d(0,0,1)), &T, &isect1);



	for (int i = HEIGHT-1; i >= 0; i--)
	{
		for (int j = 0; j < WIDTH; j++)
		{

			float u = float(i + ZR::random_double()) / float(HEIGHT);
			float v = float(j + ZR::random_double()) / float(WIDTH);
			int offset = (WIDTH * j + i);

			ZR::Ray r(origin, (lower_left_corner + v * horizontal + u * vertical) - Eigen::Vector3d(origin));
			ZR::SurfaceInteraction isect;
			Eigen::Vector3d colObj(1.0, 1.0, 0.0);
			double tHit;
			if (agg->Intersect(r, &isect))
			{
				//colObj = Eigen::Vector3d(1.0, 0.0, 0.0);
				std::cout << "255 0 0 ";
				//std::cerr<<"hello?\n";
			}
			else
			{
				std::cout << "255 255 0 ";
			}
			//std::cerr << i << "  " << j << "\n";
		}
		std::cout << "\n";
	}
	return 0;
}