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

	virtual int GetNumCharacter();
	virtual const std::shared_ptr<cKinCharacter>& GetCharacter(int id) const;
	virtual tVector GetCharPos() const;
	virtual double GetTime() const;

	virtual std::string GetName() const;

protected:
	std::vector<cKinCharacter::tParams> mCharParams;
	std::vector<std::shared_ptr<cKinCharacter>> mChars;
	std::vector<cKinCtrlBuilder::tCtrlParams> mCtrlParams;
	
	virtual void ParseCharParams(const std::shared_ptr<cArgParser>& parser, std::vector<cKinCharacter::tParams>& out_params) const;
	virtual void ParseCharCtrlParams(const std::shared_ptr<cArgParser>& parser, std::vector<cKinCtrlBuilder::tCtrlParams>& out_params) const;

	virtual bool BuildCharacters();
	virtual bool BuildCharacter(const cKinCharacter::tParams& params, std::shared_ptr<cKinCharacter>& out_char) const;
	virtual void ResetCharacters();
	virtual void UpdateCharacters(double timestep);
	virtual bool BuildController();
};