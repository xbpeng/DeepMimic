#pragma once

#include <string>

#include "Scene.h"
#include "anim/KinCharacter.h"
#include "anim/KinCtrlBuilder.h"

class cSceneKinChar : virtual public cScene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSceneKinChar();
	virtual ~cSceneKinChar();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual const std::shared_ptr<cKinCharacter>& GetCharacter() const;
	virtual tVector GetCharPos() const;
	virtual double GetTime() const;

	virtual std::string GetName() const;

protected:
	cKinCharacter::tParams mCharParams;
	std::shared_ptr<cKinCharacter> mChar;
	cKinCtrlBuilder::tCtrlParams mCtrlParams;

	virtual void ParseCharParams(const std::shared_ptr<cArgParser>& parser, cKinCharacter::tParams& out_params) const;
	virtual void ParseCharCtrlParams(const std::shared_ptr<cArgParser>& parser, cKinCtrlBuilder::tCtrlParams& out_params) const;

	virtual bool BuildCharacter();
	virtual void ResetCharacter();
	virtual void UpdateCharacter(double timestep);
	virtual bool BuildController();
};