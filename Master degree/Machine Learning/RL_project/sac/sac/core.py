import numpy as np
import tensorflow as tf
import tensorflow_probability as tfp

LOG_STD_MIN = -20
LOG_STD_MAX = 2
EPSILON = 1e-6
    
# Actor NN
class Actor(tf.keras.Model):
    # init with super
    def __init__(self, num_hidden_layers, hidden_units, num_actions):
        super(Actor, self).__init__()
        self.hidden_layers = tf.keras.Sequential()
        for _ in range(num_hidden_layers):
            self.hidden_layers.add(tf.keras.layers.Dense(hidden_units, activation='relu'))
        
        self.mean_layer = tf.keras.layers.Dense(
            num_actions, activation='linear', kernel_initializer='RandomNormal')
        self.std_layer = tf.keras.layers.Dense(
            num_actions, activation='linear', kernel_initializer='RandomNormal')
        
    # calling function as seen in class
    # here I compute an action and its log probability
    @tf.function
    def call(self, inputs):
        # fist getting mean and standard deviation of the model
        z = self.hidden_layers(inputs)
        mu = self.mean_layer(z)
        log_std = self.std_layer(z)
        log_std = tf.clip_by_value(log_std, LOG_STD_MIN, LOG_STD_MAX)
        std = tf.exp(log_std)
        
        # sampling in a Gaussian distribution...
        dist = tfp.distributions.Normal(mu, std)
        pi = tf.stop_gradient(dist.sample())
        
        # ... and remember to squash everything
        pi_s = tf.tanh(pi)
        # this formula comes from Equation 21 of Appendix C of the paper
        logp_pi = dist.log_prob(pi) - tf.math.log(1.0 - tf.pow(pi_s, 2) + EPSILON)
        logp_pi = tf.reduce_sum(logp_pi, axis=-1, keepdims=True)
        
        return pi_s, logp_pi
    
    
class Critic(tf.keras.Model):
    # init with super
    def __init__(self, num_hidden_layers, hidden_units):
        super(Critic, self).__init__()
        self.hidden_layers = tf.keras.Sequential()
        for _ in range(num_hidden_layers):
            self.hidden_layers.add(tf.keras.layers.Dense(hidden_units, activation='relu'))
        
        self.hidden_layers.add(tf.keras.layers.Dense(
            1, activation='linear', kernel_initializer='RandomNormal'))
        
    # calling function as seen in class
    # here I compute a Q-value GIVEN STATE AND ACTION (to be concatenated)
    @tf.function
    def call(self, states, actions):
        z = tf.concat([states, actions], axis=1)
        return self.hidden_layers(z)
    
    