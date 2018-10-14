#pragma once

#include "scenes/Scene.h"

class cSceneBuilder
{
public:

	static void BuildScene(const std::string& scene_name, std::shared_ptr<cScene>& out_scene);
	static void BuildDrawScene(const std::string& scene_name, std::shared_ptr<cScene>& out_scene);
	
protected:
};