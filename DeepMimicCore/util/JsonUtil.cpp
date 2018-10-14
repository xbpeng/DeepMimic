#include "JsonUtil.h"

std::string cJsonUtil::BuildVectorJson(const tVector& vec)
{
	std::string json = "";
	for (int i = 0; i < vec.size(); ++i)
	{
		if (i != 0)
		{
			json += ", ";
		}
		json += std::to_string(vec[i]);
	}
	json = "[" + json + "]";
	return json;
}

bool cJsonUtil::ReadVectorJson(const Json::Value& root, tVector& out_vec)
{
	bool succ = false;
	int num_vals = root.size();
	assert(num_vals <= 4);
	num_vals = std::min(num_vals, static_cast<int>(out_vec.size()));

	if (root.isArray())
	{
		out_vec.setZero();
		for (int i = 0; i < num_vals; ++i)
		{
			Json::Value json_elem = root.get(i, 0);
			out_vec[i] = json_elem.asDouble();
		}
		succ = true;
	}

	return succ;
}

std::string cJsonUtil::BuildVectorJson(const Eigen::VectorXd& vec)
{
	std::string json = BuildVectorString(vec);
	json = "[" + json + "]";
	return json;
}

std::string cJsonUtil::BuildVectorString(const Eigen::VectorXd& vec)
{
	std::string str = "";
	char str_buffer[32];
	for (int i = 0; i < vec.size(); ++i)
	{
		if (i != 0)
		{
			str += ",";
		}
		sprintf(str_buffer, "%20.10f", vec[i]);
		str += std::string(str_buffer);
	}
	return str;
}

bool cJsonUtil::ReadVectorJson(const Json::Value& root, Eigen::VectorXd& out_vec)
{
	bool succ = false;
	int num_vals = root.size();
	
	if (root.isArray())
	{
		out_vec.resize(num_vals);
		for (int i = 0; i < num_vals; ++i)
		{
			Json::Value json_elem = root.get(i, 0);
			out_vec[i] = json_elem.asDouble();
		}
		succ = true;
	}

	return succ;
}