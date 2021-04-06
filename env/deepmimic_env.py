import numpy as np
from env.env import Env
from DeepMimicCore import DeepMimicCore
from env.action_space import ActionSpace

class DeepMimicEnv(Env):
    def __init__(self, args, enable_draw):
        super().__init__(args, enable_draw)

        self._core = DeepMimicCore.cDeepMimicCore(enable_draw)

        rand_seed = np.random.randint(np.iinfo(np.int32).max)
        self._core.SeedRand(rand_seed)

        self._core.ParseArgs(args)
        self._core.Init()
        return

    def update(self, timestep):
        self._core.Update(timestep)

    def reset(self):
        self._core.Reset()

    def get_time(self):
        return self._core.GetTime()

    def get_name(self):
        return self._core.GetName()

    # rendering and UI interface
    def draw(self):
        self._core.Draw()

    def keyboard(self, key, x, y):
        self._core.Keyboard(key, x, y)

    def mouse_click(self, button, state, x, y):
        self._core.MouseClick(button, state, x, y)

    def mouse_move(self, x, y):
        self._core.MouseMove(x, y)

    def reshape(self, w, h):
        self._core.Reshape(w, h)

    def shutdown(self):
        self._core.Shutdown()

    def is_done(self):
        return self._core.IsDone()

    def set_playback_speed(self, speed):
        self._core.SetPlaybackSpeed(speed)

    def set_updates_per_sec(self, updates_per_sec):
        self._core.SetUpdatesPerSec(updates_per_sec)

    def get_win_width(self):
        return self._core.GetWinWidth()

    def get_win_height(self):
        return self._core.GetWinHeight()

    def get_num_update_substeps(self):
        return self._core.GetNumUpdateSubsteps()

    # rl interface
    def is_rl_scene(self):
        return self._core.IsRLScene()

    def get_num_agents(self):
        return self._core.GetNumAgents()

    def need_new_action(self, agent_id):
        return self._core.NeedNewAction(agent_id)

    def record_state(self, agent_id):
        return np.array(self._core.RecordState(agent_id))

    def record_goal(self, agent_id):
        return np.array(self._core.RecordGoal(agent_id))

    def get_action_space(self, agent_id):
        return ActionSpace(self._core.GetActionSpace(agent_id))
    
    def set_action(self, agent_id, action):
        return self._core.SetAction(agent_id, action.tolist())
    
    def get_state_size(self, agent_id):
        return self._core.GetStateSize(agent_id)

    def get_goal_size(self, agent_id):
        return self._core.GetGoalSize(agent_id)

    def get_action_size(self, agent_id):
        return self._core.GetActionSize(agent_id)

    def get_num_actions(self, agent_id):
        return self._core.GetNumActions(agent_id)

    def build_state_offset(self, agent_id):
        return np.array(self._core.BuildStateOffset(agent_id))

    def build_state_scale(self, agent_id):
        return np.array(self._core.BuildStateScale(agent_id))
    
    def build_goal_offset(self, agent_id):
        return np.array(self._core.BuildGoalOffset(agent_id))

    def build_goal_scale(self, agent_id):
        return np.array(self._core.BuildGoalScale(agent_id))
    
    def build_action_offset(self, agent_id):
        return np.array(self._core.BuildActionOffset(agent_id))

    def build_action_scale(self, agent_id):
        return np.array(self._core.BuildActionScale(agent_id))

    def build_action_bound_min(self, agent_id):
        return np.array(self._core.BuildActionBoundMin(agent_id))

    def build_action_bound_max(self, agent_id):
        return np.array(self._core.BuildActionBoundMax(agent_id))

    def build_state_norm_groups(self, agent_id):
        return np.array(self._core.BuildStateNormGroups(agent_id))

    def build_goal_norm_groups(self, agent_id):
        return np.array(self._core.BuildGoalNormGroups(agent_id))

    def calc_reward(self, agent_id):
        return self._core.CalcReward(agent_id)

    def get_reward_min(self, agent_id):
        return self._core.GetRewardMin(agent_id)

    def get_reward_max(self, agent_id):
        return self._core.GetRewardMax(agent_id)

    def get_reward_fail(self, agent_id):
        return self._core.GetRewardFail(agent_id)

    def get_reward_succ(self, agent_id):
        return self._core.GetRewardSucc(agent_id)
    
    def enable_amp_task_reward(self):
        return self._core.EnableAMPTaskReward()

    def get_amp_obs_size(self):
        return self._core.GetAMPObsSize()

    def get_amp_obs_offset(self):
        return np.array(self._core.GetAMPObsOffset())
    
    def get_amp_obs_scale(self):
        return np.array(self._core.GetAMPObsScale())
    
    def get_amp_obs_norm_group(self):
        return np.array(self._core.GetAMPObsNormGroup())
    
    def record_amp_obs_expert(self, agent_id):
        return np.array(self._core.RecordAMPObsExpert(agent_id))

    def record_amp_obs_agent(self, agent_id):
        return np.array(self._core.RecordAMPObsAgent(agent_id))
    
    def is_episode_end(self):
        return self._core.IsEpisodeEnd()

    def check_terminate(self, agent_id):
       return Env.Terminate(self._core.CheckTerminate(agent_id))

    def check_valid_episode(self):
        return self._core.CheckValidEpisode()

    def log_val(self, agent_id, val):
        self._core.LogVal(agent_id, float(val))
        return

    def set_sample_count(self, count):
        self._core.SetSampleCount(count)
        return

    def set_mode(self, mode):
        self._core.SetMode(mode.value)
        return