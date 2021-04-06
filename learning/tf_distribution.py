from abc import ABC, abstractmethod
import tensorflow as tf
import learning.tf_util as TFUtil

'''
Probability Distribution
'''
class TFDistribution(ABC):
    def __init__(self, input): 
        self._input = input
        return

    @abstractmethod
    def flat_params(self):
        pass
    
    @abstractmethod
    def logp(self, x):
        pass

    def p(self, x):
        logp_tf = self.logp(x)
        p_tf = tf.exp(logp_tf)

        return p_tf
    
    @abstractmethod
    def kl(self, other):
        pass
    
    @abstractmethod
    def entropy(self):
        pass
    
    @abstractmethod
    def sample(self):
        pass
    
    @abstractmethod
    def sample_cond(self, cond):
        pass
    
    @abstractmethod
    def get_mode(self):
        pass
    
    def param_bound_loss(self, bound_min, bound_max):
        flat_params = self.flat_params()
        num_params = flat_params.get_shape().as_list()[-1]
        assert(bound_min.shape[-1] == num_params)
        assert(bound_max.shape[-1] == num_params)

        loss = TFUtil.bound_loss(flat_params, bound_min, bound_max, axis=-1)

        return loss

    def param_reg_loss(self):
        params_tf = self.flat_params()
        err = tf.reduce_sum(tf.square(params_tf), axis=-1)
        loss = 0.5 * tf.reduce_mean(err)
        return loss