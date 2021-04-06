import numpy as np
import copy as copy
import tensorflow as tf

from learning.pg_agent import PGAgent
from learning.solvers.mpi_solver import MPISolver
import learning.tf_util as TFUtil
import learning.rl_util as RLUtil
from util.logger import Logger
import util.mpi_util as MPIUtil
import util.math_util as MathUtil
from env.env import Env

'''
Proximal Policy Optimization Agent
'''

class PPOAgent(PGAgent):
    NAME = "PPO"
    EPOCHS_KEY = "Epochs"
    BATCH_SIZE_KEY = "BatchSize"
    RATIO_CLIP_KEY = "RatioClip"
    NORM_ADV_CLIP_KEY = "NormAdvClip"
    TD_LAMBDA_KEY = "TDLambda"
    TAR_CLIP_FRAC = "TarClipFrac"
    
    ADV_EPS = 1e-5

    def __init__(self, world, id, json_data): 
        super().__init__(world, id, json_data)
        return

    def _load_params(self, json_data):
        super()._load_params(json_data)

        self.epochs = 1 if (self.EPOCHS_KEY not in json_data) else json_data[self.EPOCHS_KEY]
        self.batch_size = 1024 if (self.BATCH_SIZE_KEY not in json_data) else json_data[self.BATCH_SIZE_KEY]
        self.ratio_clip = 0.2 if (self.RATIO_CLIP_KEY not in json_data) else json_data[self.RATIO_CLIP_KEY]
        self.norm_adv_clip = 5 if (self.NORM_ADV_CLIP_KEY not in json_data) else json_data[self.NORM_ADV_CLIP_KEY]
        self.td_lambda = 0.95 if (self.TD_LAMBDA_KEY not in json_data) else json_data[self.TD_LAMBDA_KEY]
        self.tar_clip_frac = -1 if (self.TAR_CLIP_FRAC not in json_data) else json_data[self.TAR_CLIP_FRAC]

        num_procs = MPIUtil.get_num_procs()
        self._local_batch_size = int(np.ceil(self.batch_size / num_procs))
        min_replay_size = 2 * self._local_batch_size # needed to prevent buffer overflow
        assert(self.replay_buffer_size > min_replay_size)

        self.replay_buffer_size = np.maximum(min_replay_size, self.replay_buffer_size)

        return

    def _build_nets(self, json_data):
        assert self.ACTOR_NET_KEY in json_data
        assert self.CRITIC_NET_KEY in json_data

        actor_net_name = json_data[self.ACTOR_NET_KEY]
        critic_net_name = json_data[self.CRITIC_NET_KEY]
        actor_init_output_scale = 1 if (self.ACTOR_INIT_OUTPUT_SCALE_KEY not in json_data) else json_data[self.ACTOR_INIT_OUTPUT_SCALE_KEY]

        s_size = self.get_state_size()
        g_size = self.get_goal_size()
        a_size = self.get_action_size()

        # setup input tensors
        self._s_ph = tf.placeholder(tf.float32, shape=[None, s_size], name="s")
        self._g_ph = tf.placeholder(tf.float32, shape=([None, g_size] if self.has_goal() else None), name="g")
        self._a_ph = tf.placeholder(tf.float32, shape=[None, a_size], name="a")
        self._old_logp_ph = tf.placeholder(tf.float32, shape=[None], name="old_logp")
        self._tar_val_ph = tf.placeholder(tf.float32, shape=[None], name="tar_val")
        self._adv_ph = tf.placeholder(tf.float32, shape=[None], name="adv")

        with tf.variable_scope('main'):
            self._norm_a_pd_tf = self._build_net_actor(actor_net_name, self._get_actor_inputs(), actor_init_output_scale)
            self._critic_tf = self._build_net_critic(critic_net_name, self._get_critic_inputs())
                
        if (self._norm_a_pd_tf != None):
            Logger.print("Built actor net: " + actor_net_name)

        if (self._critic_tf != None):
            Logger.print("Built critic net: " + critic_net_name)
        
        sample_norm_a_tf = self._norm_a_pd_tf.sample()
        self._sample_a_tf = self._a_norm.unnormalize_tf(sample_norm_a_tf)
        self._sample_a_logp_tf = self._norm_a_pd_tf.logp(sample_norm_a_tf)
        
        mode_norm_a_tf = self._norm_a_pd_tf.get_mode()
        self._mode_a_tf = self._a_norm.unnormalize_tf(mode_norm_a_tf)
        self._mode_a_logp_tf = self._norm_a_pd_tf.logp(mode_norm_a_tf)
        
        norm_tar_a_tf = self._a_norm.normalize_tf(self._a_ph)
        self._a_logp_tf = self._norm_a_pd_tf.logp(norm_tar_a_tf)
        
        return

    def _build_losses(self, json_data):
        actor_bound_loss_weight = 10.0
        actor_weight_decay = 0 if (self.ACTOR_WEIGHT_DECAY_KEY not in json_data) else json_data[self.ACTOR_WEIGHT_DECAY_KEY]
        critic_weight_decay = 0 if (self.CRITIC_WEIGHT_DECAY_KEY not in json_data) else json_data[self.CRITIC_WEIGHT_DECAY_KEY]
        
        val_diff = self._tar_val_ph - self._critic_tf
        self._critic_loss_tf = 0.5 * tf.reduce_mean(tf.square(val_diff))

        if (critic_weight_decay != 0):
            self._critic_loss_tf += critic_weight_decay * self._weight_decay_loss(self.MAIN_SCOPE + '/critic')
        
        ratio_tf = tf.exp(self._a_logp_tf - self._old_logp_ph)
        actor_loss0 = self._adv_ph * ratio_tf
        actor_loss1 = self._adv_ph * tf.clip_by_value(ratio_tf, 1.0 - self.ratio_clip, 1 + self.ratio_clip)
        actor_loss_tf = tf.minimum(actor_loss0, actor_loss1)
        self._actor_loss_tf = -tf.reduce_mean(actor_loss_tf)
        
        # for debugging
        self._clip_frac_tf = tf.reduce_mean(tf.to_float(tf.greater(tf.abs(ratio_tf - 1.0), self.ratio_clip)))

        if (actor_bound_loss_weight != 0.0):
            self._actor_loss_tf += actor_bound_loss_weight * self._build_action_bound_loss(self._norm_a_pd_tf)

        if (actor_weight_decay != 0):
            self._actor_loss_tf += actor_weight_decay * self._weight_decay_loss(self.MAIN_SCOPE + '/actor')
         
        return

    def _build_solvers(self, json_data):
        actor_stepsize = 0.001 if (self.ACTOR_STEPSIZE_KEY not in json_data) else json_data[self.ACTOR_STEPSIZE_KEY]
        actor_momentum = 0.9 if (self.ACTOR_MOMENTUM_KEY not in json_data) else json_data[self.ACTOR_MOMENTUM_KEY]
        critic_stepsize = 0.01 if (self.CRITIC_STEPSIZE_KEY not in json_data) else json_data[self.CRITIC_STEPSIZE_KEY]
        critic_momentum = 0.9 if (self.CRITIC_MOMENTUM_KEY not in json_data) else json_data[self.CRITIC_MOMENTUM_KEY]
        
        critic_vars = self._tf_vars(self.MAIN_SCOPE + '/critic')
        critic_opt = tf.train.MomentumOptimizer(learning_rate=critic_stepsize, momentum=critic_momentum)
        self._critic_grad_tf = tf.gradients(self._critic_loss_tf, critic_vars)
        self._critic_solver = MPISolver(self.sess, critic_opt, critic_vars)

        actor_vars = self._tf_vars(self.MAIN_SCOPE + '/actor')
        actor_opt = tf.train.MomentumOptimizer(learning_rate=actor_stepsize, momentum=actor_momentum)
        self._actor_grad_tf = tf.gradients(self._actor_loss_tf, actor_vars)
        self._actor_solver = MPISolver(self.sess, actor_opt, actor_vars)
        
        return

    def _train_step(self):
        start_idx = self.replay_buffer.buffer_tail
        end_idx = self.replay_buffer.buffer_head
        assert(start_idx == 0)
        assert(self.replay_buffer.get_current_size() <= self.replay_buffer.buffer_size) # must avoid overflow
        assert(start_idx < end_idx)

        idx = np.array(list(range(start_idx, end_idx)))        
        end_mask = self.replay_buffer.is_path_end(idx)
        end_mask = np.logical_not(end_mask) 
        
        rewards = self._fetch_batch_rewards(start_idx, end_idx)
        vals = self._compute_batch_vals(start_idx, end_idx)
        new_vals = self._compute_batch_new_vals(start_idx, end_idx, rewards, vals)

        valid_idx = idx[end_mask]
        exp_idx = self.replay_buffer.get_idx_filtered(self.EXP_ACTION_FLAG).copy()
        num_valid_idx = valid_idx.shape[0]
        num_exp_idx = exp_idx.shape[0]
        exp_idx = np.column_stack([exp_idx, np.array(list(range(0, num_exp_idx)), dtype=np.int32)])
        
        local_sample_count = valid_idx.size
        global_sample_count = int(MPIUtil.reduce_sum(local_sample_count))
        mini_batches = int(np.ceil(global_sample_count / self.mini_batch_size))
        
        adv = new_vals[exp_idx[:,0]] - vals[exp_idx[:,0]]
        new_vals = np.clip(new_vals, self.val_min, self.val_max)

        adv_mean = np.mean(adv)
        adv_std = np.std(adv)
        adv = (adv - adv_mean) / (adv_std + self.ADV_EPS)
        adv = np.clip(adv, -self.norm_adv_clip, self.norm_adv_clip)

        critic_loss = 0
        actor_loss = 0
        actor_clip_frac = 0

        for e in range(self.epochs):
            np.random.shuffle(valid_idx)
            np.random.shuffle(exp_idx)

            for b in range(mini_batches):
                batch_idx_beg = b * self._local_mini_batch_size
                batch_idx_end = batch_idx_beg + self._local_mini_batch_size

                critic_batch = np.array(range(batch_idx_beg, batch_idx_end), dtype=np.int32)
                actor_batch = critic_batch.copy()
                critic_batch = np.mod(critic_batch, num_valid_idx)
                actor_batch = np.mod(actor_batch, num_exp_idx)
                shuffle_actor = (actor_batch[-1] < actor_batch[0]) or (actor_batch[-1] == num_exp_idx - 1)

                critic_batch = valid_idx[critic_batch]
                actor_batch = exp_idx[actor_batch]
                critic_batch_vals = new_vals[critic_batch]
                actor_batch_adv = adv[actor_batch[:,1]]

                critic_s = self.replay_buffer.get('states', critic_batch)
                critic_g = self.replay_buffer.get('goals', critic_batch) if self.has_goal() else None
                curr_critic_loss = self._update_critic(critic_s, critic_g, critic_batch_vals)

                actor_s = self.replay_buffer.get("states", actor_batch[:,0])
                actor_g = self.replay_buffer.get("goals", actor_batch[:,0]) if self.has_goal() else None
                actor_a = self.replay_buffer.get("actions", actor_batch[:,0])
                actor_logp = self.replay_buffer.get("logps", actor_batch[:,0])
                curr_actor_loss, curr_actor_clip_frac = self._update_actor(actor_s, actor_g, actor_a, actor_logp, actor_batch_adv)
                
                critic_loss += curr_critic_loss
                actor_loss += np.abs(curr_actor_loss)
                actor_clip_frac += curr_actor_clip_frac

                if (shuffle_actor):
                    np.random.shuffle(exp_idx)

        total_batches = mini_batches * self.epochs
        critic_loss /= total_batches
        actor_loss /= total_batches
        actor_clip_frac /= total_batches

        critic_loss = MPIUtil.reduce_avg(critic_loss)
        actor_loss = MPIUtil.reduce_avg(actor_loss)
        actor_clip_frac = MPIUtil.reduce_avg(actor_clip_frac)

        critic_stepsize = self._critic_solver.get_stepsize()
        actor_stepsize = self._actor_solver.get_stepsize()

        self.logger.log_tabular('Critic_Loss', critic_loss)
        self.logger.log_tabular('Critic_Stepsize', critic_stepsize)
        self.logger.log_tabular('Actor_Loss', actor_loss) 
        self.logger.log_tabular('Actor_Stepsize', actor_stepsize)
        self.logger.log_tabular('Clip_Frac', actor_clip_frac)
        self.logger.log_tabular('Adv_Mean', adv_mean)
        self.logger.log_tabular('Adv_Std', adv_std)

    def _train(self):
        super()._train()
        self.replay_buffer.clear()
        return
    
    def _fetch_batch_rewards(self, start_idx, end_idx):
        rewards = self.replay_buffer.get_all("rewards")[start_idx:end_idx]
        return rewards

    def _get_iters_per_update(self):
        return 1

    def _valid_train_step(self):
        samples = self.replay_buffer.get_current_size()
        exp_samples = self.replay_buffer.count_filtered(self.EXP_ACTION_FLAG)
        return (samples >= self._local_batch_size) and (exp_samples >= self._local_mini_batch_size)

    def _compute_batch_vals(self, start_idx, end_idx):
        states = self.replay_buffer.get_all("states")[start_idx:end_idx]
        goals = self.replay_buffer.get_all("goals")[start_idx:end_idx] if self.has_goal() else None
        
        idx = np.array(list(range(start_idx, end_idx)))        
        is_end = self.replay_buffer.is_path_end(idx)
        is_fail = self.replay_buffer.check_terminal_flag(idx, Env.Terminate.Fail)
        is_succ = self.replay_buffer.check_terminal_flag(idx, Env.Terminate.Succ)
        is_fail = np.logical_and(is_end, is_fail) 
        is_succ = np.logical_and(is_end, is_succ) 

        vals = self._eval_critic(states, goals)
        vals[is_fail] = self.val_fail
        vals[is_succ] = self.val_succ

        return vals

    def _compute_batch_new_vals(self, start_idx, end_idx, rewards, val_buffer):
        if self.discount == 0:
            new_vals = rewards.copy()
        else:
            new_vals = np.zeros_like(val_buffer)

            curr_idx = start_idx
            while curr_idx < end_idx:
                idx0 = curr_idx - start_idx
                idx1 = self.replay_buffer.get_path_end(curr_idx) - start_idx
                r = rewards[idx0:idx1]
                v = val_buffer[idx0:(idx1 + 1)]

                new_vals[idx0:idx1] = RLUtil.compute_return(r, self.discount, self.td_lambda, v)
                curr_idx = idx1 + start_idx + 1
        
        return new_vals

    def _update_critic(self, s, g, tar_vals):
        feed = {
            self._s_ph: s,
            self._g_ph: g,
            self._tar_val_ph: tar_vals
        }

        loss, grads = self.sess.run([self._critic_loss_tf, self._critic_grad_tf], feed)
        self._critic_solver.update(grads)
        return loss
    
    def _update_actor(self, s, g, a, logp, adv):
        feed = {
            self._s_ph: s,
            self._g_ph: g,
            self._a_ph: a,
            self._adv_ph: adv,
            self._old_logp_ph: logp
        }

        loss, grads, clip_frac = self.sess.run([self._actor_loss_tf, self._actor_grad_tf,
                                                self._clip_frac_tf], feed)
        self._actor_solver.update(grads)

        return loss, clip_frac