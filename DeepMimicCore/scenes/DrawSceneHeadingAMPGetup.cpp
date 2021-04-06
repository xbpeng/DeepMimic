#include "DrawSceneHeadingAMPGetup.h"
#include "SceneHeadingAMPGetup.h"

cDrawSceneHeadingAMPGetup::cDrawSceneHeadingAMPGetup()
{
}

cDrawSceneHeadingAMPGetup::~cDrawSceneHeadingAMPGetup()
{
}

void cDrawSceneHeadingAMPGetup::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cSceneHeadingAMPGetup>(new cSceneHeadingAMPGetup());
}