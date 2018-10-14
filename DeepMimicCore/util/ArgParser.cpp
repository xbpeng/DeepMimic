#include "ArgParser.h"

#include <assert.h>
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include "util/FileUtil.h"

const char gKeyStart = '-';
const char gCommentStart = '#';

cArgParser::cArgParser()
{
}

cArgParser::~cArgParser()
{
}

cArgParser::cArgParser(const std::vector<std::string>& args)
{
	LoadArgs(args);
}

cArgParser::cArgParser(const std::string& file)
{
	LoadFile(file);
}

void cArgParser::LoadArgs(const std::vector<std::string>& arg_strs)
{
	std::vector<std::string> vals;
	std::string curr_key = "";

	for (size_t i = 0; i < arg_strs.size(); ++i)
	{
		const std::string& str = arg_strs[i];
		if (!IsComment(str))
		{
			bool is_key = IsKey(str);
			if (is_key)
			{
				if (curr_key != "")
				{
					bool in_table = mTable.find(curr_key) != mTable.end();
					if (!in_table)
					{
						mTable[curr_key] = vals;
						curr_key = "";
					}
				}
				vals.clear();
				curr_key = str.substr(2, str.size());
			}
			else
			{
				vals.push_back(str);
			}
		}
	}

	if (curr_key != "")
	{
		bool in_table = mTable.find(curr_key) != mTable.end();
		if (!in_table)
		{
			mTable[curr_key] = vals;
		}
		curr_key = "";
	}
	vals.clear();
}

void cArgParser::Clear()
{
	mTable.clear();
}

bool cArgParser::LoadFile(const std::string& file)
{
	FILE* file_ptr = cFileUtil::OpenFile(file.c_str(), "r");
	bool succ = (file_ptr != nullptr);

	std::ifstream file_stream(file.c_str());
	std::string line_str;

	std::string str_buffer = "";
	std::vector<std::string> arg_strs;
	const std::string delims = " \t\n\r,";

	while (std::getline(file_stream, line_str))
	{
		if (line_str.size() > 0 && !IsComment(line_str))
		{
			for (size_t i = 0; i < line_str.size(); ++i)
			{
				char curr_char = line_str[i];
				if (delims.find_first_of(curr_char) != std::string::npos)
				{
					if (str_buffer != "")
					{
						arg_strs.push_back(str_buffer);
						str_buffer = "";
					}
				}
				else
				{
					str_buffer += curr_char;
				}
			}

			// dump everything else out
			if (str_buffer != "")
			{
				arg_strs.push_back(str_buffer);
				str_buffer = "";
			}
		}
	}

	cFileUtil::CloseFile(file_ptr);
	LoadArgs(arg_strs);

	return succ;
}

bool cArgParser::ParseString(const std::string& key, std::string& out) const
{
	auto it = mTable.find(key);
	if (it != mTable.end())
	{
		const auto& vals = it->second;
		out = vals[0];
		return true;
	}
	return false;
}

bool cArgParser::ParseStrings(const std::string& key, std::vector<std::string>& out) const
{
	auto it = mTable.find(key);
	if (it != mTable.end())
	{
		const auto& vals = it->second;
		out = vals;
		return true;
	}
	return false;
}

bool cArgParser::ParseInt(const std::string& key, int& out) const
{
	auto it = mTable.find(key);
	if (it != mTable.end())
	{
		const auto& vals = it->second;
		out = std::atoi(vals[0].c_str());
		return true;
	}
	return false;
}

bool cArgParser::ParseInts(const std::string& key, std::vector<int>& out) const
{
	auto it = mTable.find(key);
	if (it != mTable.end())
	{
		const auto& vals = it->second;
		size_t num_vals = vals.size();
		out.clear();
		out.reserve(num_vals);
		for (int i = 0; i < num_vals; ++i)
		{
			out.push_back(std::atoi(vals[i].c_str()));
		}
		return true;
	}
	return false;
}

bool cArgParser::ParseDouble(const std::string& key, double& out) const
{
	auto it = mTable.find(key);
	if (it != mTable.end())
	{
		const auto& vals = it->second;
		out = std::atof(vals[0].c_str());
		return true;
	}
	return false;
}

bool cArgParser::ParseDoubles(const std::string& key, std::vector<double>& out) const
{
	auto it = mTable.find(key);
	if (it != mTable.end())
	{
		const auto& vals = it->second;
		size_t num_vals = vals.size();
		out.clear();
		out.reserve(num_vals);
		for (int i = 0; i < num_vals; ++i)
		{
			out.push_back(std::atof(vals[i].c_str()));
		}
		return true;
	}
	return false;
}

bool cArgParser::ParseBool(const std::string& key, bool& out) const
{
	auto it = mTable.find(key);
	if (it != mTable.end())
	{
		const auto& vals = it->second;
		out = ParseBool(vals[0]);
		return true;
	}
	return false;
}

bool cArgParser::ParseBools(const std::string& key, std::vector<bool>& out) const
{
	auto it = mTable.find(key);
	if (it != mTable.end())
	{
		const auto& vals = it->second;
		size_t num_vals = vals.size();
		out.clear();
		out.reserve(num_vals);
		for (int i = 0; i < num_vals; ++i)
		{
			out.push_back(ParseBool(vals[i]));
		}
		return true;
	}
	return false;
}

bool cArgParser::ParseVector(const std::string& key, tVector& out) const
{
	auto it = mTable.find(key);
	if (it != mTable.end())
	{
		const auto& vals = it->second;
		size_t num_vals = std::min(vals.size(), static_cast<size_t>(out.size()));
		for (int i = 0; i < num_vals; ++i)
		{
			out[i] = std::atof(vals[i].c_str());
		}
		return true;
	}
	return false;
}

bool cArgParser::IsComment(const std::string& str) const
{
	bool is_comment = false;
	if (str.size() > 0)
	{
		is_comment = str[0] == gCommentStart;
	}
	return is_comment;
}

bool cArgParser::IsKey(const std::string& str) const
{
	size_t len = str.size();
	if (len < 3)
	{
		return false;
	}
	else
	{
		if (str[0] == gKeyStart && str[1] == gKeyStart)
		{
			return true;
		}
	}
	return false;
}

int cArgParser::GetNumArgs() const
{
	return static_cast<int>(mTable.size());
}

bool cArgParser::ParseBool(const std::string& str) const
{
	bool val = false;
	if (str == "true" || str == "1"
		|| str == "True" || str == "T"
		|| str == "t")
	{
		val = true;
	}
	return val;
}