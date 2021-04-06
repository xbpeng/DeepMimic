#pragma once

#include "scenes/DrawSceneTargetAMP.h"

class cDrawSceneDribbleAMP : virtual public cDrawSceneTargetAMP
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawSceneDribbleAMP();
	virtual ~cDrawSceneDribbleAMP();

protected:

	std::unique_ptr<cTextureDesc> mBallTex;

	virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;
	virtual void HandleRayTest(const cWorld::tRayTestResult& result);
	virtual void SetBallPos(const tVector& pos);

	virtual bool LoadTextures();

	virtual int GetTarObjID() const;
	virtual void DrawObj(int obj_id) const;
	virtual void DrawBall(int obj_id) const;
};
