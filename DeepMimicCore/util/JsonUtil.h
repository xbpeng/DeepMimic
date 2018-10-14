#pragma once

#include <string>
#include "json/json.h"
#include "util/MathUtil.h"

class cJsonUtil
{
public:
	static std::string BuildVectorJson(const tVector& vec);
	static bool ReadVectorJson(const Json::Value& root, tVector& out_vec);
	static std::string BuildVectorJson(const Eigen::VectorXd& vec);
	static std::string BuildVectorString(const Eigen::VectorXd& vec);
	static bool ReadVectorJson(const Json::Value& root, Eigen::VectorXd& out_vec);

private:
	
};
