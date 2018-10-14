#include "SimCharBuilder.h"

#include "sim/SimCharGeneral.h"

const std::string gCharName[cSimCharBuilder::eCharMax] =
{
	"none",
	"general"
};

void cSimCharBuilder::CreateCharacter(eCharType char_type, std::shared_ptr<cSimCharacter>& out_char)
{
	if (char_type == cCharGeneral)
	{
		out_char = std::shared_ptr<cSimCharGeneral>(new cSimCharGeneral());
	}
	else
	{
		printf("No valid character specified\n");
		assert(false);
	}
}


void cSimCharBuilder::ParseCharType(const std::string& char_type_str, eCharType& out_char_type)
{
	bool found = false;
	if (char_type_str == "")
	{
		out_char_type = eCharNone;
		found = true;
	}
	else
	{
		for (int i = 0; i < eCharMax; ++i)
		{
			const std::string& name = gCharName[i];
			if (char_type_str == name)
			{
				out_char_type = static_cast<eCharType>(i);
				found = true;
				break;
			}
		}
	}

	if (!found)
	{
		printf("Unsupported character type: %s\n", char_type_str.c_str());
		assert(false);
	}
}