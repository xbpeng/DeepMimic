#include "SceneBuilder.h"

#include <assert.h>

#include "DrawSceneKinChar.h"
#include "DrawSceneImitate.h"
#include "SceneImitate.h"
#include "DrawSceneImitateAMP.h"
#include "SceneImitateAMP.h"
#include "DrawSceneTargetAMP.h"
#include "SceneTargetAMP.h"
#include "DrawSceneHeadingAMP.h"
#include "SceneHeadingAMP.h"
#include "DrawSceneDribbleAMP.h"
#include "SceneDribbleAMP.h"
#include "DrawSceneStrikeAMP.h"
#include "SceneStrikeAMP.h"
#include "DrawSceneHeadingAMPGetup.h"
#include "SceneHeadingAMPGetup.h"

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
	else if (scene_name == "imitate_amp")
	{
		out_scene = std::shared_ptr<cSceneImitateAMP>(new cSceneImitateAMP());
	}
	else if (scene_name == "target_amp")
	{
		out_scene = std::shared_ptr<cSceneTargetAMP>(new cSceneTargetAMP());
	}
	else if (scene_name == "heading_amp")
	{
		out_scene = std::shared_ptr<cSceneHeadingAMP>(new cSceneHeadingAMP());
	}
	else if (scene_name == "dribble_amp")
	{
		out_scene = std::shared_ptr<cSceneDribbleAMP>(new cSceneDribbleAMP());
	}
	else if (scene_name == "strike_amp")
	{
		out_scene = std::shared_ptr<cSceneStrikeAMP>(new cSceneStrikeAMP());
	}
	else if (scene_name == "heading_amp_getup")
	{
		out_scene = std::shared_ptr<cSceneHeadingAMPGetup>(new cSceneHeadingAMPGetup());
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
	else if (scene_name == "imitate_amp")
	{
		out_scene = std::shared_ptr<cDrawSceneImitateAMP>(new cDrawSceneImitateAMP());
	}
	else if (scene_name == "target_amp")
	{
		out_scene = std::shared_ptr<cDrawSceneTargetAMP>(new cDrawSceneTargetAMP());
	}
	else if (scene_name == "heading_amp")
	{
		out_scene = std::shared_ptr<cDrawSceneHeadingAMP>(new cDrawSceneHeadingAMP());
	}
	else if (scene_name == "dribble_amp")
	{
		out_scene = std::shared_ptr<cDrawSceneDribbleAMP>(new cDrawSceneDribbleAMP());
	}
	else if (scene_name == "strike_amp")
	{
		out_scene = std::shared_ptr<cDrawSceneStrikeAMP>(new cDrawSceneStrikeAMP());
	}
	else if (scene_name == "heading_amp_getup")
	{
		out_scene = std::shared_ptr<cDrawSceneHeadingAMPGetup>(new cDrawSceneHeadingAMPGetup());
	}
	else
	{
		printf("Unsupported draw scene: %s\n", scene_name.c_str());
		assert(false);
	}
}