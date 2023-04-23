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
#include "../light/SkyBox.hpp"
#include "../Integrator/PathIntegrator.hpp"

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
	Eigen::Vector3d eye(0, 0, 6.0), look(0.0, 0.0, 0.0);
	Eigen::Vector3d up(0.0, 1.0, 0.0);
	ZR::Transform lookat = ZR::LookAt(eye, look, up);
	//取逆是因为LookAt返回的是世界坐标到相机坐标系的变换
	//而我们需要相机坐标系到世界坐标系的变换
	ZR::Transform Camera2World = Inverse(lookat);
	std::shared_ptr<ZR::Camera> cam = std::shared_ptr<ZR::Camera>(
			ZR::CreatePerspectiveCamera(WIDTH, HEIGHT, Camera2World));

	// 生成材质与纹理
	std::cerr << "Init Material\n";
	std::shared_ptr<ZR::Material> dragonMaterial;
	std::shared_ptr<ZR::Material> whiteWallMaterial;
	std::shared_ptr<ZR::Material> redWallMaterial;
	std::shared_ptr<ZR::Material> blueWallMaterial;
	std::shared_ptr<ZR::Material> whiteLightMaterial;
	//材质
	ZR::Spectrum whiteColor;
	whiteColor[0] = 0.91;
	whiteColor[1] = 0.91;
	whiteColor[2] = 0.91;
	ZR::Spectrum dragonColor;
	dragonColor[0] = 0.1;
	dragonColor[1] = 1.0;
	dragonColor[2] = 0.1;
	ZR::Spectrum redWallColor;
	redWallColor[0] = 0.9;
	redWallColor[1] = 0.1;
	redWallColor[2] = 0.17;
	ZR::Spectrum blueWallColor;
	blueWallColor[0] = 0.14;
	blueWallColor[1] = 0.21;
	blueWallColor[2] = 0.87;
	std::shared_ptr<ZR::Texture<ZR::Spectrum>> KdDragon = std::make_shared<ZR::ConstantTexture<ZR::Spectrum>>(
			dragonColor);
	std::shared_ptr<ZR::Texture<ZR::Spectrum>> KrDragon = std::make_shared<ZR::ConstantTexture<ZR::Spectrum>>(
			dragonColor);
	std::shared_ptr<ZR::Texture<ZR::Spectrum>> KdWhite = std::make_shared<ZR::ConstantTexture<ZR::Spectrum>>(
			whiteColor);
	std::shared_ptr<ZR::Texture<ZR::Spectrum>> KdRed = std::make_shared<ZR::ConstantTexture<ZR::Spectrum>>(
			redWallColor);
	std::shared_ptr<ZR::Texture<ZR::Spectrum>> KdBlue = std::make_shared<ZR::ConstantTexture<ZR::Spectrum>>(
			blueWallColor);
	std::shared_ptr<ZR::Texture<double>> sigma = std::make_shared<ZR::ConstantTexture<double>>(0.0f);
	std::shared_ptr<ZR::Texture<double>> bumpMap = std::make_shared<ZR::ConstantTexture<double>>(0.0f);
	// 材质
	dragonMaterial = std::make_shared<ZR::MatteMaterial>(KdDragon, sigma, bumpMap);

	whiteWallMaterial = std::make_shared<ZR::MatteMaterial>(KdWhite, sigma, bumpMap);
	redWallMaterial = std::make_shared<ZR::MatteMaterial>(KdRed, sigma, bumpMap);
	blueWallMaterial = std::make_shared<ZR::MatteMaterial>(KdBlue, sigma, bumpMap);

	whiteLightMaterial = std::make_shared<ZR::MatteMaterial>(KdWhite, sigma, bumpMap);

	std::vector<std::shared_ptr<ZR::Primitive>> prims;

	//墙和地板
	const int nTrianglesWall = 2 * 5;
	int vertexIndicesWall[nTrianglesWall * 3];
	for (int i = 0; i < nTrianglesWall * 3; i++)
		vertexIndicesWall[i] = i;
	const int nVerticesWall = nTrianglesWall * 3;
	const double length_Wall = 5.0f;
	Eigen::Vector3d P_Wall[nVerticesWall] = {
			//底座
			Eigen::Vector3d(0.f, 0.f, length_Wall), Eigen::Vector3d(length_Wall, 0.f, length_Wall),
			Eigen::Vector3d(0.f, 0.f, 0.f),
			Eigen::Vector3d(length_Wall, 0.f, length_Wall), Eigen::Vector3d(length_Wall, 0.f, 0.f),
			Eigen::Vector3d(0.f, 0.f, 0.f),
			//天花板
			Eigen::Vector3d(0.f, length_Wall, length_Wall), Eigen::Vector3d(0.f, length_Wall, 0.f),
			Eigen::Vector3d(length_Wall, length_Wall, length_Wall),
			Eigen::Vector3d(length_Wall, length_Wall, length_Wall), Eigen::Vector3d(0.f, length_Wall, 0.f),
			Eigen::Vector3d(length_Wall, length_Wall, 0.f),
			//后墙
			Eigen::Vector3d(0.f, 0.f, 0.f), Eigen::Vector3d(length_Wall, 0.f, 0.f),
			Eigen::Vector3d(length_Wall, length_Wall, 0.f),
			Eigen::Vector3d(0.f, 0.f, 0.f), Eigen::Vector3d(length_Wall, length_Wall, 0.f),
			Eigen::Vector3d(0.f, length_Wall, 0.f),
			//右墙
			Eigen::Vector3d(0.f, 0.f, 0.f), Eigen::Vector3d(0.f, length_Wall, length_Wall),
			Eigen::Vector3d(0.f, 0.f, length_Wall),
			Eigen::Vector3d(0.f, 0.f, 0.f), Eigen::Vector3d(0.f, length_Wall, 0.f),
			Eigen::Vector3d(0.f, length_Wall, length_Wall),
			//左墙
			Eigen::Vector3d(length_Wall, 0.f, 0.f), Eigen::Vector3d(length_Wall, length_Wall, length_Wall),
			Eigen::Vector3d(length_Wall, 0.f, length_Wall),
			Eigen::Vector3d(length_Wall, 0.f, 0.f), Eigen::Vector3d(length_Wall, length_Wall, 0.f),
			Eigen::Vector3d(length_Wall, length_Wall, length_Wall)
	};
	ZR::Transform tri_ConBox2World = ZR::Translate(
			Eigen::Vector3d(-0.5 * length_Wall, -0.5 * length_Wall, -0.5 * length_Wall));
	ZR::Transform tri_World2ConBox = Inverse(tri_ConBox2World);
	std::shared_ptr<ZR::TriangleMesh> meshConBox = std::make_shared<ZR::TriangleMesh>
			(tri_ConBox2World, nTrianglesWall, vertexIndicesWall, nVerticesWall, P_Wall, nullptr, nullptr, nullptr,
					nullptr);
	std::vector<std::shared_ptr<ZR::Shape>> trisConBox;
	for (int i = 0; i < nTrianglesWall; ++i)
		trisConBox.push_back(
				std::make_shared<ZR::Triangle>(&tri_ConBox2World, &tri_World2ConBox, false, meshConBox, i));
	//将物体填充到基元
	for (int i = 0; i < nTrianglesWall; ++i)
	{
		if (i == 6 || i == 7)
			prims.push_back(std::make_shared<ZR::GeometricPrimitive>(trisConBox[i], redWallMaterial, nullptr));
		else if (i == 8 || i == 9)
			prims.push_back(std::make_shared<ZR::GeometricPrimitive>(trisConBox[i], blueWallMaterial, nullptr));
		else
			prims.push_back(std::make_shared<ZR::GeometricPrimitive>(trisConBox[i], whiteWallMaterial, nullptr));
	}


	// 生成Mesh加速结构
	std::shared_ptr<ZR::TriangleMesh> mesh;
	std::vector<std::shared_ptr<ZR::Shape>> tris;

	ZR::Transform tri_Object2World, tri_World2Object;

	tri_Object2World = ZR::Translate(Eigen::Vector3d(0.f, -2.9f, 0.f)) * tri_Object2World;
	tri_World2Object = Inverse(tri_Object2World);

	std::cerr << "Read Mesh...\n";
	ZR::plyInfo plyi("dragon.3d");
	mesh = std::make_shared<ZR::TriangleMesh>(tri_Object2World, plyi.nTriangles, plyi.vertexIndices, plyi.nVertices,
			plyi.vertexArray, nullptr, nullptr, nullptr, nullptr);
	tris.reserve(plyi.nTriangles);
	std::cerr << "Init Triangles...\n";
	for (int i = 0; i < plyi.nTriangles; ++i)
		tris.push_back(std::make_shared<ZR::Triangle>(&tri_Object2World, &tri_World2Object, false, mesh, i));

	std::cerr << "Init Primitives...\n";
	for (int i = 0; i < plyi.nTriangles; ++i)
		prims.push_back(std::make_shared<ZR::GeometricPrimitive>(tris[i], dragonMaterial, nullptr));
	plyi.Release();

	//面光源
	std::cerr << "Init AreaLight\n";
	std::vector<std::shared_ptr<ZR::Light>> lights;
	// 定义面光源
	int nTrianglesAreaLight = 2; //面光源数（三角形数）
	int vertexIndicesAreaLight[6] = { 0, 1, 2, 3, 4, 5 }; //面光源顶点索引
	int nVerticesAreaLight = 6; //面光源顶点数
	const double yPos_AreaLight = 0.0;
	Eigen::Vector3d P_AreaLight[6] = { Eigen::Vector3d(-1.4, 0.0, 1.4), Eigen::Vector3d(-1.4, 0.0, -1.4),
									   Eigen::Vector3d(1.4, 0.0, 1.4),
									   Eigen::Vector3d(1.4, 0.0, 1.4), Eigen::Vector3d(-1.4, 0.0, -1.4),
									   Eigen::Vector3d(1.4, 0.0, -1.4) };
	//面光源的变换矩阵
	ZR::Transform tri_Object2World_AreaLight = ZR::Translate(Eigen::Vector3d(0.0f, 2.45f, 0.0f));
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
	for (int i = 0; i < nTrianglesAreaLight; ++i)
	{
		std::shared_ptr<ZR::AreaLight> area =
				std::make_shared<ZR::DiffuseAreaLight>(tri_Object2World_AreaLight, ZR::Spectrum(5.f), 5,
						trisAreaLight[i], false);
		lights.push_back(area);
		prims.push_back(std::make_shared<ZR::GeometricPrimitive>(trisAreaLight[i], whiteLightMaterial, area));
	}

	//无限环境光源
	std::cerr << "Init SkyBoxLight...\n";
	ZR::Transform SkyBoxToWorld;
	Eigen::Vector3d SkyBoxCenter(0.f, 0.f, 0.f);
	double SkyBoxRadius = 10.0f;

//	ZR::Spectrum LightI(15);
//	ZR::Transform LightToWorld = ZR::Translate(Eigen::Vector3d(1,4.5,-6)) * LightToWorld;
//	std::shared_ptr<ZR::Light> pointLight = std::make_shared<ZR::PointLight>(LightToWorld, LightI);
//	lights.push_back(pointLight);

	std::cerr << "Init Accelerator...\n";
	std::shared_ptr<ZR::Aggregate> aggregate;
	aggregate = std::make_unique<ZR::BVHAccel>(prims, 1);


	std::cerr << "Init Sampler...\n";
	ZR::Bounds2i imageBound(Eigen::Vector2i(0, 0), Eigen::Vector2i(WIDTH, HEIGHT));
	std::shared_ptr<ZR::ClockRandSampler> sampler = std::make_unique<ZR::ClockRandSampler>(128, imageBound);

	std::cerr << "Build Scene...\n";
	std::unique_ptr<ZR::Scene> worldScene = std::make_unique<ZR::Scene>(aggregate, lights);
	ZR::Bounds2i ScreenBound(Eigen::Vector2i(0, 0), Eigen::Vector2i(WIDTH, HEIGHT));

	std::cerr << "Build Integrator...\n";
	std::shared_ptr<ZR::Integrator> integrator = std::make_shared<ZR::PathIntegrator>(20, cam, sampler, ScreenBound,
			1.f, "spatial", &buffer);


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