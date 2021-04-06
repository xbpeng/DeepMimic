import numpy as np
import tensorflow as tf
import copy

from learning.tf_agent import TFAgent
from learning.solvers.mpi_solver import MPISolver
import learning.tf_util as TFUtil
import learning.nets.net_builder as NetBuilder
from learning.tf_distribution_gaussian_diag import TFDistributionGaussianDiag
import learning.rl_util as RLUtil
from util.logger import Logger
import util.mpi_util as MPIUtil
import util.math_util as MathUtil
from env.action_space import ActionSpace
from env.env import Env

'''
Policy Gradient Agent
'''

class PGAgent(TFAgent):
    NAME = 'PG'

    ACTOR_NET_KEY = 'ActorNet'
    ACTOR_STEPSIZE_KEY = 'ActorStepsize'
    ACTOR_MOMENTUM_KEY = 'ActorMomentum'
    ACTOR_WEIGHT_DECAY_KEY = 'ActorWeightDecay'
    ACTOR_INIT_OUTPUT_SCALE_KEY = 'ActorInitOutputScale'

    CRITIC_NET_KEY = 'CriticNet'
    CRITIC_STEPSIZE_KEY = 'CriticStepsize'
    CRITIC_MOMENTUM_KEY = 'CriticMomentum'
    CRITIC_WEIGHT_DECAY_KEY = 'CriticWeightDecay'

    MAIN_SCOPE = "main"
    
    EXP_ACTION_FLAG = 1 << 0

    def __init__(self, world, id, json_data): 
        self._exp_action = False
        super().__init__(world, id, json_data)
        return

    def reset(self):
        super().reset()
        self._exp_action = False
        return

    def _check_action_space(self):
        action_space = self.get_action_space()
        return action_space == ActionSpace.Continuous

    def _load_params(self, json_data):
        super()._load_params(json_data)
        self.val_min, self.val_max = self._calc_val_bounds(self.discount)
        self.val_fail, self.val_succ = self._calc_term_vals(self.discount)
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
        self._s_ph = tf.placeholder(tf.float32, shape=[None, s_size], name="s") # observations
        self._tar_val_ph = tf.placeholder(tf.float32, shape=[None], name="tar_val") # target value s
        self._adv_ph = tf.placeholder(tf.float32, shape=[None], name="adv") # advantage
        self._a_ph = tf.placeholder(tf.float32, shape=[None, a_size], name="a") # target actions
        self._g_ph = tf.placeholder(tf.float32, shape=([None, g_size] if self.has_goal() else None), name="g") # goals

        with tf.variable_scope(self.MAIN_SCOPE):
            self._norm_a_pd_tf = self._build_net_actor(actor_net_name, self._get_actor_inputs(), actor_init_output_scale)
            self._critic_tf = self._build_net_critic(critic_net_name, self._get_critic_inputs())

        if (self.actor_tf != None):
            Logger.print('Built actor net: ' + actor_net_name)

        if (self.critic_tf != None):
            Logger.print('Built critic net: ' + critic_net_name)
            
        sample_norm_a_tf = self._norm_a_pd_tf.sample()
        self._sample_a_tf = self._a_norm.unnormalize_tf(sample_norm_a_tf)
        self._sample_a_logp_tf = self._norm_a_pd_tf.logp(sample_norm_a_tf)
        
        mode_norm_a_tf = self._norm_a_pd_tf.get_mode()
        self._mode_a_tf = self._a_norm.unnormalize_tf(mode_norm_a_tf)
        self._mode_a_logp_tf = self._norm_a_pd_tf.logp(mode_norm_a_tf)
        
        norm_tar_a_tf = self._a_norm.normalize_tf(self._a_tf)
        self._a_logp_tf = self._norm_a_pd_tf.logp(norm_tar_a_tf)
        
        return

    def _build_losses(self, json_data):
        actor_bound_loss_weight = 10.0
        actor_weight_decay = 0 if (self.ACTOR_WEIGHT_DECAY_KEY not in json_data) else json_data[self.ACTOR_WEIGHT_DECAY_KEY]
        critic_weight_decay = 0 if (self.CRITIC_WEIGHT_DECAY_KEY not in json_data) else json_data[self.CRITIC_WEIGHT_DECAY_KEY]

        val_diff = self._tar_val_tf - self._critic_tf
        self._critic_loss_tf = 0.5 * tf.reduce_mean(tf.square(val_diff))

        if (critic_weight_decay != 0):
            self._critic_loss_tf += critic_weight_decay * self._weight_decay_loss(self.MAIN_SCOPE + '/critic')
        
        self._actor_loss_tf = self._adv_ph * self._a_logp_tf
        self._actor_loss_tf = -tf.reduce_mean(self._actor_loss_tf)
        
        if (actor_bound_loss_weight != 0.0):
            self._actor_loss_tf += actor_bound_loss_weight * self._build_action_bound_loss(self._norm_a_pd_tf)
        
        if (actor_weight_decay != 0):
            self.actor_loss_tf += actor_weight_decay * self._weight_decay_loss(self.MAIN_SCOPE + '/actor')
        
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

    def _build_net_actor(self, net_name, input_tfs, init_output_scale, reuse=False):
        with tf.variable_scope('actor', reuse=reuse):
            h = NetBuilder.build_net(net_name, input_tfs, reuse)
            
            std_type = TFDistributionGaussianDiag.StdType.Default
            a_size = self.get_action_size()

            mean_kernel_init = tf.random_uniform_initializer(minval=-init_output_scale, maxval=init_output_scale)
            mean_bias_init = tf.zeros_initializer()
            logstd_kernel_init = tf.random_uniform_initializer(minval=-init_output_scale, maxval=init_output_scale)
            logstd_bias_init = np.log(self.exp_params_curr.noise) * np.ones(a_size)
            logstd_bias_init = logstd_bias_init.astype(np.float32)
            
            norm_a_pd_tf = TFDistributionGaussianDiag(input=h, dim=a_size, std_type=std_type,
                                 mean_kernel_init=mean_kernel_init, mean_bias_init=mean_bias_init, 
                                 logstd_kernel_init=logstd_kernel_init, logstd_bias_init=logstd_bias_init,
                                 reuse=reuse)

        return norm_a_pd_tf
    
    def _build_net_critic(self, net_name, input_tfs, reuse=False):
        out_size = 1

        with tf.variable_scope('critic', reuse=reuse):
            h = NetBuilder.build_net(net_name, input_tfs, reuse)
            val_tf = tf.layers.dense(inputs=h, units=out_size, activation=None,
                                    kernel_initializer=tf.contrib.layers.xavier_initializer(),
                                    reuse=reuse)
            val_tf = tf.squeeze(val_tf, axis=-1)

        return val_tf
    
    def _get_actor_inputs(self):
        norm_s_tf = self._s_norm.normalize_tf(self._s_ph)
        input_tfs = [norm_s_tf]
        if (self.has_goal()):
            norm_g_tf = self._g_norm.normalize_tf(self._g_ph)
            input_tfs += [norm_g_tf]
        return input_tfs
    
    def _get_critic_inputs(self):
        norm_s_tf = self._s_norm.normalize_tf(self._s_ph)
        input_tfs = [norm_s_tf]
        if (self.has_goal()):
            norm_g_tf = self._g_norm.normalize_tf(self._g_ph)
            input_tfs += [norm_g_tf]
        return input_tfs

    def _build_action_bound_loss(self, norm_a_pd_tf):
        norm_a_bound_min = self._a_norm.normalize(self._a_bound_min)
        norm_a_bound_max = self._a_norm.normalize(self._a_bound_max)
        
        if (isinstance(norm_a_pd_tf, TFDistributionGaussianDiag)):
            logstd_min = -np.inf
            logstd_max = np.inf
            norm_a_logstd_min = logstd_min * np.ones_like(norm_a_bound_min)
            norm_a_logstd_max = logstd_max * np.ones_like(norm_a_bound_max)
            norm_a_bound_min = np.concatenate([norm_a_bound_min, norm_a_logstd_min], axis=-1)
            norm_a_bound_max = np.concatenate([norm_a_bound_max, norm_a_logstd_max], axis=-1)
        
        a_bound_loss = norm_a_pd_tf.param_bound_loss(norm_a_bound_min, norm_a_bound_max)
        return a_bound_loss
    
    def _initialize_vars(self):
        super()._initialize_vars()
        self._sync_solvers()
        return

    def _sync_solvers(self):
        self._actor_solver.sync()
        self._critic_solver.sync()
        return

    def _decide_action(self, s, g):
        with self.sess.as_default(), self.graph.as_default():
            self._exp_action = self._enable_stoch_policy() and MathUtil.flip_coin(self.exp_params_curr.rate)

            a, logp = self._eval_actor(s, g, self._exp_action)
            a = a[0]
            logp = logp[0]

        return a, logp

    def _enable_stoch_policy(self):
        return self.enable_training and (self._mode == self.Mode.TRAIN or self._mode == self.Mode.TRAIN_END)

    def _eval_actor(self, s, g, exp_action):
        s = np.reshape(s, [-1, self.get_state_size()])
        g = np.reshape(g, [-1, self.get_goal_size()]) if self.has_goal() else None
        
        feed = {
            self._s_ph : s,
            self._g_ph : g
        }
        
        if (exp_action):
            run_tfs = [self._sample_a_tf, self._sample_a_logp_tf]
        else:
            run_tfs = [self._mode_a_tf, self._mode_a_logp_tf]
        
        a, logp = self.sess.run(run_tfs, feed_dict=feed)

        return a, logp
    
    def _eval_critic(self, s, g):
        s = np.reshape(s, [-1, self.get_state_size()])
        g = np.reshape(g, [-1, self.get_goal_size()]) if self.has_goal() else None

        feed = {
            self._s_ph : s,
            self._g_ph : g
        }

        val = self.sess.run(self._critic_tf, feed_dict=feed)

        return val

    def _record_flags(self):
        flags = int(0)
        if (self._exp_action):
            flags = flags | self.EXP_ACTION_FLAG
        return flags

    def _train_step(self):
        super()._train_step()

        critic_loss = self._update_critic()
        actor_loss = self._update_actor()
        critic_loss = MPIUtil.reduce_avg(critic_loss)
        actor_loss = MPIUtil.reduce_avg(actor_loss)

        critic_stepsize = self.critic_solver.get_stepsize()
        actor_stepsize = self.actor_solver.get_stepsize()
        
        self.logger.log_tabular('Critic_Loss', critic_loss)
        self.logger.log_tabular('Critic_Stepsize', critic_stepsize)
        self.logger.log_tabular('Actor_Loss', actor_loss) 
        self.logger.log_tabular('Actor_Stepsize', actor_stepsize)

        return

    def _update_critic(self):
        idx = self.replay_buffer.sample(self._local_mini_batch_size)
        s = self.replay_buffer.get('states', idx)
        g = self.replay_buffer.get('goals', idx) if self.has_goal() else None
        
        tar_vals = self._calc_updated_vals(idx)
        tar_vals = np.clip(tar_vals, self.val_min, self.val_max)
        
        feed = {
            self._s_ph: s,
            self._g_ph: g,
            self._tar_val_ph: tar_vals
        }

        loss, grads = self.sess.run([self.critic_loss_tf, self.critic_grad_tf], feed)
        self.critic_solver.update(grads)
        return loss
    
    def _update_actor(self):
        key = self.EXP_ACTION_FLAG
        idx = self.replay_buffer.sample_filtered(self._local_mini_batch_size, key)
        has_goal = self.has_goal()

        s = self.replay_buffer.get('states', idx)
        g = self.replay_buffer.get('goals', idx) if has_goal else None
        a = self.replay_buffer.get('actions', idx)

        V_new = self._calc_updated_vals(idx)
        V_old = self._eval_critic(s, g)
        adv = V_new - V_old

        feed = {
            self._s_ph: s,
            self._g_ph: g,
            self._a_ph: a,
            self._adv_ph: adv
        }

        loss, grads = self.sess.run([self._actor_loss_tf, self._actor_grad_tf], feed)
        self._actor_solver.update(grads)

        return loss

    def _calc_updated_vals(self, idx):
        r = self.replay_buffer.get('rewards', idx)

        if self.discount == 0:
            new_V = r
        else:
            next_idx = self.replay_buffer.get_next_idx(idx)
            s_next = self.replay_buffer.get('states', next_idx)
            g_next = self.replay_buffer.get('goals', next_idx) if self.has_goal() else None

            is_end = self.replay_buffer.is_path_end(idx)
            is_fail = self.replay_buffer.check_terminal_flag(idx, Env.Terminate.Fail)
            is_succ = self.replay_buffer.check_terminal_flag(idx, Env.Terminate.Succ)
            is_fail = np.logical_and(is_end, is_fail) 
            is_succ = np.logical_and(is_end, is_succ) 

            V_next = self._eval_critic(s_next, g_next)
            V_next[is_fail] = self.val_fail
            V_next[is_succ] = self.val_succ

            new_V = r + self.discount * V_next
        return new_V

    def _log_val(self, s, g):
        val = self._eval_critic(s, g)
        norm_val = (1.0 - self.discount) * val
        self.world.env.log_val(self.id, norm_val[0])
        return

    def _build_replay_buffer(self, buffer_size):
        super()._build_replay_buffer(buffer_size)
        self.replay_buffer.add_filter_key(self.EXP_ACTION_FLAG)
        return