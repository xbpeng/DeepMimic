#include "DrawSceneImitateAMP.h"
#include "SceneImitateAMP.h"

cDrawSceneImitateAMP::cDrawSceneImitateAMP()
{
}

cDrawSceneImitateAMP::~cDrawSceneImitateAMP()
{
}

void cDrawSceneImitateAMP::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cSceneImitateAMP>(new cSceneImitateAMP());
}