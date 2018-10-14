#pragma once

#include "sim/SimCharacter.h"

class cSimCharBuilder
{
public:
	
	enum eCharType
	{
		eCharNone,
		cCharGeneral,
		eCharMax
	};

	static void CreateCharacter(eCharType char_type, std::shared_ptr<cSimCharacter>& out_char);
	static void ParseCharType(const std::string& char_type_str, eCharType& out_char_type);
	
protected:

};
