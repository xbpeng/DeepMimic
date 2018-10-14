#pragma once

#include <vector>
#include <string>
#include <map>
#include "util/MathUtil.h"

class cArgParser
{
public:
	cArgParser();
	cArgParser(const std::vector<std::string>& args);
	cArgParser(const std::string& file);
	virtual ~cArgParser();

	virtual void Clear();
	virtual void LoadArgs(const std::vector<std::string>& arg_strs);
	virtual bool LoadFile(const std::string& file);

	virtual int GetNumArgs() const;
	virtual bool ParseString(const std::string& key, std::string& out) const;
	virtual bool ParseStrings(const std::string& key, std::vector<std::string>& out) const;
	virtual bool ParseInt(const std::string& key, int& out) const;
	virtual bool ParseInts(const std::string& key, std::vector<int>& out) const;
	virtual bool ParseDouble(const std::string& key, double& out) const;
	virtual bool ParseDoubles(const std::string& key, std::vector<double>& out) const;
	virtual bool ParseBool(const std::string& key, bool& out) const;
	virtual bool ParseBools(const std::string& key, std::vector<bool>& out) const;
	virtual bool ParseVector(const std::string& key, tVector& out) const;

protected:
	std::map<std::string, std::vector<std::string>> mTable;

	virtual bool IsComment(const std::string& str) const;
	virtual bool IsKey(const std::string& str) const;
	virtual bool ParseBool(const std::string& str) const;
};