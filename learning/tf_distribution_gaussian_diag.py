from enum import Enum
import numpy as np
import tensorflow as tf
import learning.tf_util as TFUtil
from learning.tf_distribution import TFDistribution

'''
Gaussian Distribution - Diagonal Covariance
'''
class TFDistributionGaussianDiag(TFDistribution):
    class StdType(Enum):
        Default = 0
        Constant = 1
        Variable = 2

    def identity(dim, name="identity"):
        mean = np.zeros(dim)
        logstd = np.zeros(dim)
        dist = TFDistributionGaussianDiag(input=None, dim=dim, std_type=TFDistributionGaussianDiag.StdType.Default,
                                          mean_kernel_init=None, mean_bias_init=None,
                                          logstd_kernel_init=None, logstd_bias_init=None,
                                          name=name, direct_mean=mean, direct_logstd=logstd)
        return dist

    def from_params(mean, logstd, name="dist_gauss_diag"):
        dim = int(mean.shape[-1])
        assert(dim == logstd.shape[-1])
        dist = TFDistributionGaussianDiag(input=None, dim=dim, std_type=TFDistributionGaussianDiag.StdType.Default,
                                          mean_kernel_init=None, mean_bias_init=None,
                                          logstd_kernel_init=None, logstd_bias_init=None,
                                          name=name, direct_mean=mean, direct_logstd=logstd)
        return dist

    def __init__(self, input, dim, std_type,
                 mean_kernel_init=tf.contrib.layers.xavier_initializer(),
                 mean_bias_init=tf.zeros_initializer(), 
                 logstd_kernel_init=tf.contrib.layers.xavier_initializer(),
                 logstd_bias_init=tf.zeros_initializer(), 
                 name="dist_gauss_diag", direct_mean=None, direct_logstd=None,
                 reuse=False): 

        super().__init__(input)

        isinstance(logstd_bias_init, np.ndarray)

        self._dim = dim
        self._std_type = std_type
        self._mean_kernel_init = mean_kernel_init
        self._mean_bias_init = mean_bias_init
        self._logstd_kernel_init = logstd_kernel_init
        self._logstd_bias_init = logstd_bias_init

        self._mean = direct_mean
        self._logstd = direct_logstd

        with tf.variable_scope(name, reuse=reuse):
            self._build_params(reuse)
        
        mean = self.get_mean()
        logstd = self.get_logstd()
        std = self.get_std()
        
        mean_shape = mean.get_shape().as_list()
        logstd_shape = logstd.get_shape().as_list()
        std_shape = std.get_shape().as_list()
        assert(mean_shape[-1] == self._dim)
        assert(logstd_shape[-1] == self._dim)
        assert(std_shape[-1] == self._dim)

        return

    def get_dim(self):
        return self._dim

    def get_mean(self):
        return self._mean

    def set_mean(self, mean_tf):
        self._mean = mean_tf
        return

    def get_logstd(self):
        return self._logstd

    def set_logstd(self, logstd_tf):
        self._logstd = logstd_tf
        return

    def get_std(self):
        return self._std
    
    def flat_params(self):
        mean_tf = self._mean
        logstd_tf = self._logstd
        mean_shape = mean_tf.get_shape().as_list()
        logstd_shape = logstd_tf.get_shape().as_list()
        
        if (len(mean_shape) == 2 and len(logstd_shape) == 1):
            mean_rows = tf.shape(mean_tf)[0]
            logstd_tf = tf.reshape(logstd_tf, [1, logstd_shape[-1]])
            logstd_tf = tf.tile(logstd_tf, [mean_rows, 1])
        else:
            assert (len(mean_shape) == len(logstd_shape))

        params = tf.concat([mean_tf, logstd_tf], axis=-1)
        return params

    def logp(self, x):
        diff_tf = x - self._mean
        logp_tf = -0.5 * tf.reduce_sum(tf.square(diff_tf / self._std), axis=-1)
        logp_tf += -0.5 * self._dim * np.log(2.0 * np.pi) - tf.reduce_sum(self._logstd, axis=-1)
        return logp_tf
    
    def kl(self, other, eps=0):
        assert isinstance(other, TFDistributionGaussianDiag)
        other_var = tf.square(other.get_std())
        if (eps > 0):
            other_var = tf.maximum(other_var, eps * eps)

        kl_tf = tf.reduce_sum(other.get_logstd() - self._logstd + (tf.square(self._std) + tf.square(self._mean - other.get_mean())) / (2.0 * other_var), axis=-1)
        kl_tf += -0.5 * self._dim
        return kl_tf

    def kl_reg(self):
        kl_tf = tf.reduce_sum(-self._logstd + 0.5 * (tf.square(self._std) + tf.square(self._mean)), axis=-1)
        kl_tf += -0.5 * self._dim
        return kl_tf

    def entropy(self):
        ent_tf = self._calc_entropy(self._logstd)
        return ent_tf
    
    def sample(self):
        shape_tf = tf.shape(self._mean)
        noise = tf.random_normal(shape_tf)
        samples_tf = self.sample_noise(noise)

        return samples_tf

    def sample_noise(self, noise):
        samples_tf = self._std * noise
        samples_tf += self._mean
        return samples_tf

    def sample_clip(self, noise_clip):
        assert(noise_clip >= 0.0)

        shape_tf = tf.shape(self._mean)
        noise = tf.random_normal(shape_tf)
        noise = tf.clip_by_value(noise, -noise_clip, noise_clip)
        samples_tf = self.sample_noise(noise)

        return samples_tf

    def sample_cond(self, cond):
        cond_tf = cond
        cond_shape = cond_tf.get_shape().as_list()
        shape_tf = tf.shape(self._mean)
        
        sample_tf = self._std * tf.random_normal(shape_tf)
        
        sample_shape = sample_tf.get_shape().as_list()
        assert (len(sample_shape) == len(cond_shape) + 1)
        cond_tf = tf.expand_dims(cond_tf, axis=-1)
        
        sample_tf = cond_tf * sample_tf
        sample_tf += self._mean

        return sample_tf

    def get_mode(self):
        return self.get_mean()

    def param_reg_loss(self):
        # only regularize mean, covariance is regularized via entropy reg
        params_tf = self._mean
        err = tf.reduce_sum(tf.square(params_tf), axis=-1)
        loss = 0.5 * tf.reduce_mean(err)

        return loss

    def _build_params(self, reuse):
        if (self._mean is None):
            self._mean = self._build_params_mean(self._input, "mean", reuse)
        elif (isinstance(self._mean, np.ndarray)):
            self._mean = tf.convert_to_tensor(self._mean, dtype=tf.float32)

        if (self._logstd is None):
            self._logstd =  self._build_params_logstd(self._input, tf.shape(self._mean), "logstd", reuse)
        elif (isinstance(self._logstd, np.ndarray)):
            self._logstd = tf.convert_to_tensor(self._logstd, dtype=tf.float32)

        self._std = tf.exp(self._logstd)

        return

    def _build_params_mean(self, input, name, reuse):
        mean = tf.layers.dense(inputs=input, units=self._dim, activation=None,
                                kernel_initializer=self._mean_kernel_init,
                                bias_initializer=self._mean_bias_init, 
                                name=name, reuse=reuse)
        return mean

    def _build_params_logstd(self, input, mean_shape, name, reuse):
        if ((self._std_type == self.StdType.Default) or (self._std_type == self.StdType.Constant)):
            with tf.variable_scope(name, reuse=reuse):
                trainable = (self._std_type == self.StdType.Constant)
                logstd = tf.get_variable(dtype=tf.float32, name="bias", initializer=self._logstd_bias_init,
                                          trainable=trainable)
                logstd = tf.broadcast_to(logstd, mean_shape)

        elif (self._std_type == self.StdType.Variable):
            logstd = tf.layers.dense(inputs=input, units=self._dim, activation=None,
                                kernel_initializer=self._logstd_kernel_init, 
                                bias_initializer=self._logstd_bias_init, 
                                name=name, reuse=reuse)

        else:
            assert(False), "Unsupported standard deviation type"

        return logstd

    def _calc_entropy(self, logstd):
        ent_tf = tf.reduce_sum(logstd, axis=-1)
        ent_tf += 0.5 * self._dim * np.log(2.0 * np.pi * np.e)
        return ent_tf