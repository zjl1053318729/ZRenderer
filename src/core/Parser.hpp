#pragma once

#include <string>
#include "Scene.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>


namespace ZR
{
	class Parser
	{
	public:
		Parser() = default;
		Scene get_scene(std::string& path);

	};

	Scene Parser::get_scene(std::string& path)
	{

	}
}