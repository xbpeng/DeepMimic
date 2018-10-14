#pragma once
#include <memory>

#include "DrawScene.h"
#include "SceneKinChar.h"

class cDrawSceneKinChar : virtual public cDrawScene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawSceneKinChar();
	virtual ~cDrawSceneKinChar();

	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);

	std::string GetName() const;

protected:
	std::shared_ptr<cSceneKinChar> mScene;

	virtual void BuildScene(std::shared_ptr<cSceneKinChar>& out_scene) const;
	virtual void SetupScene(std::shared_ptr<cSceneKinChar>& out_scene);
	virtual void UpdateScene(double time_elapsed);

	virtual tVector GetCamTrackPos() const;
	virtual tVector GetCamStillPos() const;
	virtual tVector GetDefaultCamFocus() const;

	virtual void DrawGround() const;
	virtual void DrawGround3D() const;
	virtual void DrawCharacters() const;
};