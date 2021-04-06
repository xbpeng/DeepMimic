#include <iostream>

#include "util/ArgParser.h"
#include "render/TextureDesc.h"
#include "scenes/Scene.h"
#include "scenes/RLScene.h"
#include "scenes/DrawScene.h"

class cDeepMimicCore
{
public:
	cDeepMimicCore(bool enable_draw);
	virtual ~cDeepMimicCore();

	virtual void SeedRand(int seed);
	virtual void ParseArgs(const std::vector<std::string>& args);
	virtual void Init();
	virtual void Update(double timestep);
	virtual void Reset();

	virtual double GetTime() const;
	virtual std::string GetName() const;
	virtual bool EnableDraw() const;

	virtual void Draw();
	virtual void Keyboard(int key, int x, int y);
	virtual void MouseClick(int button, int state, int x, int y);
	virtual void MouseMove(int x, int y);
	virtual void Reshape(int w, int h);

	virtual void Shutdown();
	virtual bool IsDone() const;
	virtual cDrawScene* GetDrawScene() const;

	virtual void SetPlaybackSpeed(double speed);
	virtual void SetUpdatesPerSec(double updates_per_sec);

	virtual int GetWinWidth() const;
	virtual int GetWinHeight() const;

	virtual int GetNumUpdateSubsteps() const;

	// RL interface
	virtual bool IsRLScene() const;
	virtual int GetNumAgents() const;
	virtual bool NeedNewAction(int agent_id) const;
	virtual std::vector<double> RecordState(int agent_id) const;
	virtual std::vector<double> RecordGoal(int agent_id) const;
	virtual void SetAction(int agent_id, const std::vector<double>& action);
	virtual void LogVal(int agent_id, double val);

	virtual int GetActionSpace(int agent_id) const;
	virtual int GetStateSize(int agent_id) const;
	virtual int GetGoalSize(int agent_id) const;
	virtual int GetActionSize(int agent_id) const;
	virtual int GetNumActions(int agent_id) const; // for discrete actions

	virtual std::vector<double> BuildStateOffset(int agent_id) const;
	virtual std::vector<double> BuildStateScale(int agent_id) const;
	virtual std::vector<double> BuildGoalOffset(int agent_id) const;
	virtual std::vector<double> BuildGoalScale(int agent_id) const;
	virtual std::vector<double> BuildActionOffset(int agent_id) const;
	virtual std::vector<double> BuildActionScale(int agent_id) const;
	virtual std::vector<double> BuildActionBoundMin(int agent_id) const;
	virtual std::vector<double> BuildActionBoundMax(int agent_id) const;

	virtual std::vector<int> BuildStateNormGroups(int agent_id) const;
	virtual std::vector<int> BuildGoalNormGroups(int agent_id) const;

	virtual double CalcReward(int agent_id) const;
	virtual double GetRewardMin(int agent_id) const;
	virtual double GetRewardMax(int agent_id) const;
	virtual double GetRewardFail(int agent_id);
	virtual double GetRewardSucc(int agent_id);

	virtual bool EnableAMPTaskReward() const;
	virtual int GetAMPObsSize() const;
	virtual std::vector<double> GetAMPObsOffset() const;
	virtual std::vector<double> GetAMPObsScale() const;
	virtual std::vector<int> GetAMPObsNormGroup() const;
	virtual std::vector<double> RecordAMPObsAgent(int agent_id);
	virtual std::vector<double> RecordAMPObsExpert(int agent_id);

	virtual bool IsEpisodeEnd() const;
	virtual bool CheckValidEpisode() const;
	virtual int CheckTerminate(int agent_id) const;
	virtual void SetMode(int mode);
	virtual void SetSampleCount(int count);

protected:
	unsigned long int mRandSeed;

	std::shared_ptr<cArgParser> mArgParser;
	std::shared_ptr<cScene> mScene;
	std::shared_ptr<cRLScene> mRLScene;

	std::shared_ptr<cTextureDesc> mDefaultFrameBuffer;

	int mNumUpdateSubsteps;

	// info for rendering
	double mPlaybackSpeed;
	double mUpdatesPerSec; // FPS counter
	
	virtual void SetupScene();
	virtual void ClearScene();

	virtual int GetCurrTime() const;
	virtual void InitFrameBuffer();
	virtual void CalcDeviceCoord(int pixel_x, int pixel_y, double& out_device_x, double& out_device_y) const;

	virtual double GetAspectRatio();
	virtual void CopyFrame(cTextureDesc& src) const;

	virtual const std::shared_ptr<cRLScene>& GetRLScene() const;
	virtual void ConvertVector(const Eigen::VectorXd& in_vec, std::vector<double>& out_vec) const;
	virtual void ConvertVector(const Eigen::VectorXi& in_vec, std::vector<int>& out_vec) const;
	virtual void ConvertVector(const std::vector<double>& in_vec, Eigen::VectorXd& out_vec) const;
	virtual void ConvertVector(const std::vector<int>& in_vec, Eigen::VectorXi& out_vec) const;
};