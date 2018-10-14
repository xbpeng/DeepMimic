#include "GroundPlane.h"

bool cGroundPlane::ParseParamsJson(const Json::Value& json, Eigen::VectorXd& out_params)
{
	out_params.resize(0);
	return true;
}

cGroundPlane::cGroundPlane()
{
	mPrevCenter.setZero();
}

cGroundPlane::~cGroundPlane()
{
}

void cGroundPlane::Init(const std::shared_ptr<cWorld>& world, const tParams& params)
{
	mParams = params;
	mPrevCenter.setZero();

	btVector3 normal = btVector3(0, 1, 0);
	btVector3 origin = btVector3(static_cast<btScalar>(params.mOrigin[0]),
								static_cast<btScalar>(params.mOrigin[1]),
								static_cast<btScalar>(params.mOrigin[2]));
	normal.normalize();
	btScalar w = normal.dot(origin);
	mColShape = std::unique_ptr<btCollisionShape>(new btStaticPlaneShape(normal, w));

	btRigidBody::btRigidBodyConstructionInfo cons_info(0, this, mColShape.get(), btVector3(0, 0, 0));
	mSimBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mSimBody->setFriction(static_cast<btScalar>(params.mFriction));

	cGround::Init(world, params);
}

void cGroundPlane::Update(double time_elapsed, const tVector& bound_min, const tVector& bound_max)
{
	const double dist_threshold = 100;
	cGround::Update(time_elapsed, bound_min, bound_max);
	tVector new_center = 0.5 * (bound_max + bound_min);
	tVector delta = new_center - mPrevCenter;
	double dist_sq = delta.squaredNorm();
	if (dist_sq > dist_threshold)
	{
		mPrevCenter = new_center;
		FlagUpdate();
	}
}

void cGroundPlane::Clear()
{
	cGround::Clear();
	mPrevCenter.setZero();
}

double cGroundPlane::SampleHeight(const tVector& pos) const
{
	return cGround::SampleHeight(pos);
}

double cGroundPlane::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	const btStaticPlaneShape* shape = reinterpret_cast<btStaticPlaneShape*>(mColShape.get());
	btVector3 n = shape->getPlaneNormal();
	btScalar c = shape->getPlaneConstant();

	out_valid_sample = true;
	return n[1] * c;
}

void cGroundPlane::SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const
{
	double h = SampleHeight(tVector::Zero());
	int num_pos = static_cast<int>(pos.rows());
	out_h = Eigen::VectorXd::Ones(num_pos) * h;
}

const tVector& cGroundPlane::GetPrevCenter() const
{
	return mPrevCenter;
}

cGroundPlane::eClass cGroundPlane::GetGroundClass() const
{
	return eClassPlane;
}

void cGroundPlane::CalcAABB(tVector& out_min, tVector& out_max) const
{
	out_min = -std::numeric_limits<double>::infinity() * tVector::Ones();
	out_max = std::numeric_limits<double>::infinity() * tVector::Ones();
}