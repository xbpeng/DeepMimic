#pragma once

#include "CharController.h"
#include "util/IndexManager.h"

class cAgentRegistry
{
public:
	cAgentRegistry();
	virtual ~cAgentRegistry();

	virtual void Clear();
	virtual int GetNumAgents() const;
	virtual int AddAgent(const std::shared_ptr<cCharController>& agent, cSimCharacter* character);

	virtual const std::shared_ptr<cCharController>& GetAgent(int id) const;
	virtual cSimCharacter* GetChar(int id) const;
	virtual void PrintInfo() const;

protected:

	cIndexManager mIDManager;
	std::vector<std::shared_ptr<cCharController>> mAgents;
	std::vector<cSimCharacter*> mChars;
};