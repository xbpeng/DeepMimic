#include "CharController.h"
#include "SimCharacter.h"

const int cCharController::gNormGroupSingle = 0;
const int cCharController::gNormGroupNone = -1;

cCharController::cCharController()
{
}

cCharController::~cCharController()
{
}

void cCharController::Update(double time_step)
{
	cController::Update(time_step);
}

void cCharController::PostUpdate(double time_step)
{
}

bool cCharController::NeedNewAction() const
{
	return false;
}

void cCharController::ApplyAction(const Eigen::VectorXd& action)
{
}

void cCharController::RecordState(Eigen::VectorXd& out_state)
{
}

void cCharController::RecordAction(Eigen::VectorXd& out_action) const
{
}

int cCharController::GetStateSize() const
{
	return 0;
}

int cCharController::GetActionSize() const
{
	return 0;
}

int cCharController::GetNumActions() const
{
	return 0;
}

void cCharController::HandlePoseReset()
{
}

void cCharController::HandleVelReset()
{
}

void cCharController::BuildStateOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int state_size = GetStateSize();
	out_offset = Eigen::VectorXd::Zero(state_size);
	out_scale = Eigen::VectorXd::Ones(state_size);
}

void cCharController::BuildActionOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int action_size = GetActionSize();
	out_offset = Eigen::VectorXd::Zero(action_size);
	out_scale = Eigen::VectorXd::Ones(action_size);
}

void cCharController::BuildActionBounds(Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	int action_size = GetActionSize();
	out_min = -std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(action_size);
	out_max = std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(action_size);
}

void cCharController::BuildStateNormGroups(Eigen::VectorXi& out_groups) const
{
	int state_size = GetStateSize();
	out_groups = gNormGroupSingle * Eigen::VectorXi::Ones(state_size);
}

double cCharController::GetRewardMin() const
{
	return -std::numeric_limits<double>::infinity();
}

double cCharController::GetRewardMax() const
{
	return std::numeric_limits<double>::infinity();
}

int cCharController::NumChildren() const
{
	return 0;
}

const std::shared_ptr<cCharController>& cCharController::GetChild(int i) const
{
	return nullptr;
}