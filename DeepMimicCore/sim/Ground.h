#pragma once

#include "sim/SimRigidBody.h"

class cGround : public cSimRigidBody
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eClass
	{
		eClassPlane,
		eClassMax,
		eClassInvalid
	};

	enum eType
	{
		eTypePlane,
		eTypeMax
	};

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eType mType;
		double mFriction;
		tVector mOrigin;
		double mBlend;

		double mGroundWidth;
		double mVertSpacingX;
		double mVertSpacingZ;

		unsigned long mRandSeed;
		bool mHasRandSeed;

		Eigen::MatrixXd mParamArr;

		tParams();
	};

	static const std::string gTypeKey;
	static const std::string gGroundWidthKey;
	static const std::string gVertSpacingXKey;
	static const std::string gVertSpacingZKey;
	static const std::string gParamsKey;
	
	static eClass GetClassFromType(eType ground_type);
	static void ParseType(const std::string& str, eType& out_type);
	
	virtual ~cGround();

	virtual void Init(const std::shared_ptr<cWorld>& world, const tParams& params);
	virtual void Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max);
	virtual void Clear();

	virtual double SampleHeight(const tVector& pos) const;
	virtual double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
	virtual void SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const;
	virtual void SampleHeightVel(const tVector& pos, double& out_h, tVector& out_vel, 
									bool& out_valid_sample) const;

	virtual eClass GetGroundClass() const;
	virtual void SetBlendParams(const Eigen::VectorXd& params);
	virtual void SetParamBlend(double blend);

	virtual size_t GetUpdateCount() const;
	virtual void SeedRand(unsigned long seed);
	virtual void SamplePlacement(const tVector& origin, tVector& out_pos, tQuaternion& out_rot);
	virtual tVector GetSize() const override;
	virtual double GetVertSpacingX() const;
	virtual double GetVertSpacingZ() const;

	virtual bool Output(const std::string& out_file) const;

protected:

	cRand mRand;
	tParams mParams;
	Eigen::VectorXd mBlendParams;
	size_t mUpdateCount;
	double mTime;

	cGround();

	virtual void SetupRandGen();
	virtual void CalcBlendParams(double blend, Eigen::VectorXd& out_params) const;
	virtual int GetNumParamSets() const;
	virtual int GetBlendParamSize() const;

	virtual void FlagUpdate();
};
