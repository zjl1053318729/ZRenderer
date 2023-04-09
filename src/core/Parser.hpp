#pragma once

#include <string>
#include "scene.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>


namespace ZR
{
	class Parser
	{
	public:
		Parser() = default;
		scene get_scene(std::string& path);

	};

	scene Parser::get_scene(std::string& path)
	{

	}
}