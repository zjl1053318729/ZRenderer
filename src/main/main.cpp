#include <Eigen/Eigen>
#include <memory>
#include "../shape/triangle.hpp"
#include "../shape/plyRead.hpp"
#include "../accelerator/BVHAccel.hpp"
#include "../shape/sphere.hpp"
#include "../core/transform.hpp"
#include "../core/spectrum.hpp"
#include "../core/camera.hpp"
#include "../core/perspective.hpp"
#include "../core/scene.hpp"
#include "../Integrator/Integrator.hpp"
#include "../sampler/halton.hpp"
#include "../sampler/clockRand.hpp"


const int WIDTH = 800, HEIGHT = 800;
ZR::Transform sphereT_Object2World, sphereT_World2Object;
ZR::Shape* s = new ZR::Sphere(&sphereT_Object2World, &sphereT_World2Object, false, 0.5);

ZR::Buffer buffer(HEIGHT, WIDTH);

int main()
{
	freopen("result.ppm", "w", stdout);

	std::cout << "P3\n";
	std::cout << WIDTH << " " << HEIGHT << "\n";
	std::cout << "255\n";

	std::cerr << "Init Camera\n";
	ZR::Camera* cam;
	//初始化程序
	Eigen::Vector3d eye(-3,1.5,-3), look(0.0, 0.0, 0.0);
	Eigen::Vector3d up(0.0, 1.0, 0.0);
	ZR::Transform lookat = ZR::LookAt(eye, look, up);
	//取逆是因为LookAt返回的是世界坐标到相机坐标系的变换
	//而我们需要相机坐标系到世界坐标系的变换
	ZR::Transform Camera2World = Inverse(lookat);

	cam = ZR::CreatePerspectiveCamera(WIDTH, HEIGHT, Camera2World);


	// 生成Mesh加速结构
	std::shared_ptr<ZR::TriangleMesh> mesh;
	std::vector<std::shared_ptr<ZR::Shape>> tris;
	std::vector<std::shared_ptr<ZR::Primitive>> prims;
	ZR::plyInfo* plyi;
	ZR::Aggregate* agg;
	ZR::Transform tri_Object2World, tri_World2Object;

	tri_Object2World = ZR::Translate(Eigen::Vector3d(0.0, -2.5, 0.0)) * tri_Object2World;
	tri_World2Object = Inverse(tri_Object2World);
	std::cerr << "Read Mesh\n";
	plyi = new ZR::plyInfo("dragon.3d");
	mesh = std::make_shared<ZR::TriangleMesh>(tri_Object2World, plyi->nTriangles, plyi->vertexIndices, plyi->nVertices,
			plyi->vertexArray, nullptr, nullptr, nullptr, nullptr);
	tris.reserve(plyi->nTriangles);
	std::cerr << "Init Triangles\n";
	for (int i = 0; i < plyi->nTriangles; ++i)
		tris.push_back(std::make_shared<ZR::Triangle>(&tri_Object2World, &tri_World2Object, false, mesh, i));
	std::cerr << "Init Primitives\n";
	for (int i = 0; i < plyi->nTriangles; ++i)
		prims.push_back(std::make_shared<ZR::GeometricPrimitive>(tris[i]));
	agg = new ZR::BVHAccel(prims, 1);

	std::cerr << "Init Sampler\n";
	ZR::Bounds2i imageBound(Eigen::Vector2i(0, 0), Eigen::Vector2i(WIDTH, HEIGHT));
	std::shared_ptr<ZR::ClockRandSampler> sampler = std::make_unique<ZR::ClockRandSampler>(8, imageBound);

	std::unique_ptr<ZR::Scene> worldScene = std::make_unique<ZR::Scene>(agg);
	ZR::Bounds2i ScreenBound(Eigen::Vector2i(0, 0), Eigen::Vector2i(WIDTH, HEIGHT));

	std::shared_ptr<ZR::Integrator> integrator = std::make_shared<ZR::SamplerIntegrator>(cam, sampler, ScreenBound, buffer);


	std::cerr << agg->WorldBound().pMin << "\n\n";
	std::cerr << agg->WorldBound().pMax << "\n\n";



	std::cerr << "Start Rendering\n";


	// 开始执行渲染
	double frameTime=0;
	integrator->Render(*worldScene, frameTime);

	std::cerr << "rendering time: "<<frameTime<<"\n";
	std::cerr << "writing buffer to file\n";
	int r, g, b;
	for (int i = 0; i < HEIGHT; i++)
	{
		for (int j = 0; j < WIDTH; j++)
		{
			r = 255.0 * buffer(i,j)[0];
			g = 255.0 * buffer(i,j)[1];
			b = 255.0 * buffer(i,j)[2];
			std::cout << r << " " << g << " " << b << " ";
		}
		std::cout << "\n";
	}

	return 0;
}