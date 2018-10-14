#include "SceneBuilder.h"

#include <assert.h>

#include "DrawSceneKinChar.h"
#include "DrawSceneImitate.h"
#include "SceneImitate.h"

void cSceneBuilder::BuildScene(const std::string& scene_name, std::shared_ptr<cScene>& out_scene)
{
	if (scene_name == "")
	{
		printf("No scene specified\n");
		assert(false);
	}
	else if (scene_name == "kin_char")
	{
		out_scene = std::shared_ptr<cSceneKinChar>(new cSceneKinChar());
	}
	else if (scene_name == "imitate")
	{
		out_scene = std::shared_ptr<cSceneImitate>(new cSceneImitate());
	}
	else
	{
		printf("Unsupported scene: %s\n", scene_name.c_str());
		assert(false);
	}
}

void cSceneBuilder::BuildDrawScene(const std::string& scene_name, std::shared_ptr<cScene>& out_scene)
{
	if (scene_name == "")
	{
		printf("No scene specified\n");
		assert(false);
	}
	else if (scene_name == "kin_char")
	{
		out_scene = std::shared_ptr<cDrawSceneKinChar>(new cDrawSceneKinChar());
	}
	else if (scene_name == "imitate")
	{
		out_scene = std::shared_ptr<cDrawSceneImitate>(new cDrawSceneImitate());
	}
	else
	{
		printf("Unsupported draw scene: %s\n", scene_name.c_str());
		assert(false);
	}
}