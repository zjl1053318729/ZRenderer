#include <Eigen/Eigen>
#include <memory>
#include <iostream>
#include "../core/ZRender.hpp"
#include "../Integrator/Integrator.hpp"
#include "../texture/ConstantTexture.hpp"
#include "../material/MatteMaterial.hpp"
#include "../shape/triangle.hpp"
#include "../shape/plyRead.hpp"
#include "../accelerator/BVHAccel.hpp"
#include "../core/scene.hpp"
#include "../sampler/clockRand.hpp"
#include "../light/PointLight.hpp"
#include "../light/DiffuseAreaLight.hpp"
#include "../Integrator/WhittedIntegrator.hpp"
#include "../core/perspective.hpp"
#include "../material/mirror.hpp"

const int WIDTH = 800, HEIGHT = 800;

ZR::Buffer buffer(HEIGHT, WIDTH);

int main()
{
	freopen("result.ppm", "w", stdout);

	std::cout << "P3\n";
	std::cout << WIDTH << " " << HEIGHT << "\n";
	std::cout << "255\n";

	std::cerr << "Init Camera\n";
	//初始化程序
	Eigen::Vector3d eye(-3.0, 1.5, -3), look(0.0, 0.0, 0.0);
	Eigen::Vector3d up(0.0, 1.0, 0.0);
	ZR::Transform lookat = ZR::LookAt(eye, look, up);
	//取逆是因为LookAt返回的是世界坐标到相机坐标系的变换
	//而我们需要相机坐标系到世界坐标系的变换
	ZR::Transform Camera2World = Inverse(lookat);
	std::shared_ptr<ZR::Camera> cam = std::shared_ptr<ZR::Camera>(
			ZR::CreatePerspectiveCamera(WIDTH, HEIGHT, Camera2World));

	// 生成材质与纹理
	std::cerr << "Init Material\n";
	ZR::Spectrum floorColor;
	floorColor[0] = 0.2;
	floorColor[1] = 0.3;
	floorColor[2] = 0.9;
	ZR::Spectrum dragonColor;
	dragonColor[0] = 0.0;
	dragonColor[1] = 1.0;
	dragonColor[2] = 0.0;
	std::shared_ptr<ZR::Texture<ZR::Spectrum>> KdDragon = std::make_shared<ZR::ConstantTexture<ZR::Spectrum>>(
			dragonColor);
	std::shared_ptr<ZR::Texture<ZR::Spectrum>> KdFloor = std::make_shared<ZR::ConstantTexture<ZR::Spectrum>>(
			floorColor);
	std::shared_ptr<ZR::Texture<ZR::Spectrum>> KrDragon = std::make_shared<ZR::ConstantTexture<ZR::Spectrum>>(
			dragonColor);
	std::shared_ptr<ZR::Texture<double>> sigma = std::make_shared<ZR::ConstantTexture<double>>(0.0f);
	std::shared_ptr<ZR::Texture<double>> bumpMap = std::make_shared<ZR::ConstantTexture<double>>(0.0f);
	//材质
	std::shared_ptr<ZR::Material> dragonMaterial = std::make_shared<ZR::MatteMaterial>(KdDragon, sigma, bumpMap);
	std::shared_ptr<ZR::Material> floorMaterial = std::make_shared<ZR::MatteMaterial>(KdFloor, sigma, bumpMap);
	std::shared_ptr<ZR::Material> whiteLightMaterial = std::make_shared<ZR::MatteMaterial>(KdFloor, sigma, bumpMap);
	std::shared_ptr<ZR::Material> mirrorMaterial = std::make_shared<ZR::MirrorMaterial>(KrDragon, bumpMap);

	//地板
	std::cerr << "Init Floor\n";
	int nTrianglesFloor = 2;
	int vertexIndicesFloor[6] = { 0, 1, 2, 3, 4, 5 };
	int nVerticesFloor = 6;
	const double yPos_Floor = -2.0;
	Eigen::Vector3d P_Floor[6] = {
			Eigen::Vector3d(-6.0, yPos_Floor, 6.0), Eigen::Vector3d(6.0, yPos_Floor, 6.0),
			Eigen::Vector3d(-6.0, yPos_Floor, -6.0),
			Eigen::Vector3d(6.0, yPos_Floor, 6.0), Eigen::Vector3d(6.0, yPos_Floor, -6.0),
			Eigen::Vector3d(-6.0, yPos_Floor, -6.0)
	};
	ZR::Transform floor_Object2World;
	ZR::Transform floor_World2Object = Inverse(floor_Object2World);
	std::shared_ptr<ZR::TriangleMesh> meshFloor = std::make_shared<ZR::TriangleMesh>
			(floor_Object2World, nTrianglesFloor, vertexIndicesFloor, nVerticesFloor, P_Floor, nullptr, nullptr,
					nullptr, nullptr);
	std::vector<std::shared_ptr<ZR::Shape>> trisFloor;
	for (int i = 0; i < nTrianglesFloor; ++i)
		trisFloor.push_back(
				std::make_shared<ZR::Triangle>(&floor_Object2World, &floor_World2Object, false, meshFloor, i));

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
		prims.push_back(std::make_shared<ZR::GeometricPrimitive>(tris[i], dragonMaterial, nullptr));
	for (int i = 0; i < nTrianglesFloor; ++i)
		prims.push_back(std::make_shared<ZR::GeometricPrimitive>(trisFloor[i], mirrorMaterial, nullptr));

	//面光源
	std::cerr << "Init AreaLight\n";
	// 光源
	std::vector<std::shared_ptr<ZR::Light>> lights;

	// 定义面光源
	int nTrianglesAreaLight = 2; //面光源数（三角形数）
	int vertexIndicesAreaLight[6] = { 0, 1, 2, 3, 4, 5 }; //面光源顶点索引
	int nVerticesAreaLight = 6; //面光源顶点数
	const double yPos_AreaLight = 8.0;
	Eigen::Vector3d P_AreaLight[6] = { Eigen::Vector3d(-6, yPos_AreaLight, 6),
									   Eigen::Vector3d(-6, yPos_AreaLight, -6),
									   Eigen::Vector3d(6, yPos_AreaLight, 6),
									   Eigen::Vector3d(6, yPos_AreaLight, 6),
									   Eigen::Vector3d(-6, yPos_AreaLight, -6),
									   Eigen::Vector3d(6, yPos_AreaLight, -6) };
	//面光源的变换矩阵
	ZR::Transform tri_Object2World_AreaLight = ZR::Translate(Eigen::Vector3d(0.7, 0.0, -2.0));
	ZR::Transform tri_World2Object_AreaLight = Inverse(tri_Object2World_AreaLight);
	//构造三角面片集
	std::shared_ptr<ZR::TriangleMesh> meshAreaLight = std::make_shared<ZR::TriangleMesh>
			(tri_Object2World_AreaLight, nTrianglesAreaLight, vertexIndicesAreaLight, nVerticesAreaLight, P_AreaLight,
					nullptr, nullptr, nullptr, nullptr);
	std::vector<std::shared_ptr<ZR::Shape>> trisAreaLight;
	//生成三角形数组
	for (int i = 0; i < nTrianglesAreaLight; ++i)
		trisAreaLight.push_back(
				std::make_shared<ZR::Triangle>(&tri_Object2World_AreaLight, &tri_World2Object_AreaLight, false,
						meshAreaLight, i));
	//填充光源类物体到基元
	std::shared_ptr<ZR::AreaLight> area;
	for (int i = 0; i < nTrianglesAreaLight; ++i)
	{
		area = std::make_shared<ZR::DiffuseAreaLight>(tri_Object2World_AreaLight, ZR::Spectrum(2.5), 10,
				trisAreaLight[i], false);
		lights.push_back(area);
		prims.push_back(std::make_shared<ZR::GeometricPrimitive>(trisAreaLight[i], whiteLightMaterial, area));
	}

//	ZR::Spectrum LightI(15);
//	ZR::Transform LightToWorld = ZR::Translate(Eigen::Vector3d(1,4.5,-6)) * LightToWorld;
//	std::shared_ptr<ZR::Light> pointLight = std::make_shared<ZR::PointLight>(LightToWorld, LightI);
//	lights.push_back(pointLight);

	std::cerr << "Init Accelerator\n";
	//agg = std::make_unique<ZR::BVHAccel>(prims,1);
	agg = new ZR::BVHAccel(prims, 1);

	//生成采样器
	std::cerr << "Init Sampler\n";
	ZR::Bounds2i imageBound(Eigen::Vector2i(0, 0), Eigen::Vector2i(WIDTH, HEIGHT));
	std::shared_ptr<ZR::ClockRandSampler> sampler = std::make_unique<ZR::ClockRandSampler>(8, imageBound);

	std::unique_ptr<ZR::Scene> worldScene = std::make_unique<ZR::Scene>(agg, lights);
	ZR::Bounds2i ScreenBound(Eigen::Vector2i(0, 0), Eigen::Vector2i(WIDTH, HEIGHT));

	std::shared_ptr<ZR::Integrator> integrator = std::make_shared<ZR::WhittedIntegrator>(8, cam, sampler, ScreenBound,
			buffer);


	std::cerr << "Start Rendering\n";


	// 开始执行渲染
	double frameTime = 0;
	integrator->Render(*worldScene, frameTime);

	//从buffer写入到文件
	std::cerr << "rendering time: " << frameTime << "\n";
	std::cerr << "writing buffer to file\n";
	int r, g, b;
	for (int i = 0; i < HEIGHT; i++)
	{
		for (int j = 0; j < WIDTH; j++)
		{
			//防止过曝
			r = ZR::Clamp(255.0 * buffer(i, j)[0], 0, 255);
			g = ZR::Clamp(255.0 * buffer(i, j)[1], 0, 255);
			b = ZR::Clamp(255.0 * buffer(i, j)[2], 0, 255);
			std::cout << r << " " << g << " " << b << " ";
		}
		std::cout << "\n";
	}

	return 0;
}