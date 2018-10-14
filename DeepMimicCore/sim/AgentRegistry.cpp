#include "AgentRegistry.h"

cAgentRegistry::cAgentRegistry()
{
}

cAgentRegistry::~cAgentRegistry()
{
}

void cAgentRegistry::Clear()
{
	mIDManager.Clear();
	mAgents.clear();
	mChars.clear();
}

int cAgentRegistry::GetNumAgents() const
{
	return mIDManager.GetNumUsed();
}

int cAgentRegistry::AddAgent(const std::shared_ptr<cCharController>& agent, cSimCharacter* character)
{
	if (mIDManager.IsFull())
	{
		int curr_size = mIDManager.GetSize();
		mIDManager.Resize(curr_size + 1);
	}

	int num_agents = static_cast<int>(mAgents.size());
	int id = mIDManager.RequestIndex();
	if (id >= num_agents)
	{
		assert(id == num_agents);
		mAgents.push_back(agent);
		mChars.push_back(character);
	}
	else
	{
		mAgents[id] = agent;
		mChars[id] = character;
	}
	return id;
}

const std::shared_ptr<cCharController>& cAgentRegistry::GetAgent(int id) const
{
	return mAgents[id];
}

cSimCharacter* cAgentRegistry::GetChar(int id) const
{
	return mChars[id];
}

void cAgentRegistry::PrintInfo() const
{
	int num_agents = GetNumAgents();
	printf("\n");
	printf("Agent Registry\n");
	printf("Num Agents: %i\n", num_agents);

	for (int i = 0; i < num_agents; ++i)
	{
		const auto& agent = GetAgent(i);
		std::string name = agent->GetName();
		printf("Agent %i: %s\n", i, name.c_str());
	}
	printf("\n");
}
