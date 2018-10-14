#include "DrawSceneImitate.h"
#include "SceneImitate.h"
#include "render/DrawCharacter.h"
#include "render/DrawUtil.h"
#include "sim/RBDUtil.h"

const double gLinkWidth = 0.025f;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.6f, 0.65f, 0.675f, 1);

cDrawSceneImitate::cDrawSceneImitate()
{
	mDrawKinChar = false;
}

cDrawSceneImitate::~cDrawSceneImitate()
{
}

void cDrawSceneImitate::Init()
{
	cDrawSceneSimChar::Init();
	cRLScene::Init();
}

void cDrawSceneImitate::Clear()
{
	cDrawSceneSimChar::Clear();
	cDrawRLScene::Clear();
}

bool cDrawSceneImitate::IsEpisodeEnd() const
{
	return cDrawRLScene::IsEpisodeEnd();
}

bool cDrawSceneImitate::CheckValidEpisode() const
{
	return cDrawRLScene::CheckValidEpisode();
}

void cDrawSceneImitate::Keyboard(unsigned char key, double device_x, double device_y)
{
	cDrawSceneSimChar::Keyboard(key, device_x, device_y);

	switch (key)
	{
	case 'k':
		DrawKinChar(!mDrawKinChar);
		break;
	default:
		break;
	}
}

void cDrawSceneImitate::DrawKinChar(bool enable)
{
	mDrawKinChar = enable;
	if (mDrawKinChar)
	{
		printf("Enabled draw kinematic character\n");
	}
	else
	{
		printf("Disabled draw kinematic character\n");
	}
}

std::string cDrawSceneImitate::GetName() const
{
	return cDrawRLScene::GetName();
}

cRLScene* cDrawSceneImitate::GetRLScene() const
{
	return dynamic_cast<cRLScene*>(mScene.get());
}

void cDrawSceneImitate::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cSceneImitate>(new cSceneImitate());
}

void cDrawSceneImitate::DrawCharacters() const
{
	if (mDrawKinChar)
	{
		DrawKinCharacters();
	}
	cDrawSceneSimChar::DrawCharacters();
}

void cDrawSceneImitate::DrawKinCharacters() const
{
	const auto& kin_char = GetKinChar();
	DrawKinCharacter(kin_char);
}
void cDrawSceneImitate::DrawKinCharacter(const std::shared_ptr<cKinCharacter>& kin_char) const
{
	cDrawCharacter::Draw(*kin_char, gLinkWidth, gFilLColor, gLineColor);
}

const std::shared_ptr<cKinCharacter>& cDrawSceneImitate::GetKinChar() const
{
	const cSceneImitate* scene = dynamic_cast<const cSceneImitate*>(mScene.get());
	return scene->GetKinChar();
}