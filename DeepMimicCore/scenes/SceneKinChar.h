#pragma once

#include <string>

#include "Scene.h"
#include "anim/KinCharacter.h"

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
	
	virtual void ParseCharParams(const std::shared_ptr<cArgParser>& parser, cKinCharacter::tParams& out_params) const;

	virtual bool BuildCharacters();
	virtual bool BuildCharacter(const cKinCharacter::tParams& params, std::shared_ptr<cKinCharacter>& out_char) const;
	virtual void ResetCharacters();
	virtual void UpdateCharacters(double timestep);
};