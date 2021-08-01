import tensorflow as tf

from sac import replay_buffer
from sac import core

EPSILON = 1e-6

# updating weights using polyak averaging
# polyak averaging is an optimization technique that sets final parameters
# to an average of (recent) parameters visited in the optimization trajectory
# traced by the gradient descent algorithm
def weights_update(weights_target, weights, polyak):
    for weight_target, weight in zip(weights_target, weights):
        # theta_target = polyak * theta_target + (1 - polyak) * theta_current
        polyak_step = polyak * weight_target + (1 - polyak) * weight
        weight_target.assign(polyak_step)


class SoftActorCritic:
    def __init__(self, state_dim, num_actions,
                 gamma=0.99, alpha=0.3, learning_rate=1e-3,
                 polyak=0.99, num_hidden_layers=2, hidden_units=256):
        
        # arguments of SAC:
        # - state_dim : dimension of the observation space of the environment
        #               (in Ant-v2 it is 111)
        # - num_actions: dimension of the action space of the environment
        #               (in Ant-v2 it is 8)
        # - gamma: 0.99, discount factor
        # - alpha: 0.3, temperature parameter that determines the contribution of the
        #               Entropy Maximization term in the soft state value function
        # - learning_rate: 1e-3
        # - polyak: 0.99, Polyak coefficient for the weights updating in a NN
        # - num_hidden_layers: 2, number of hidden layers for the Actor and Critic networks
        # - hidden_units: 256, number of units for each hidden layer
        
        # setting some params
        self.state_dim = state_dim
        self.num_actions = num_actions
        self.gamma = gamma
        self.alpha = alpha
        self.polyak = polyak
        self.learning_rate = learning_rate
        
        # creating all the networks I need: 1 Actor + 4 Critics (= 2 + 2 targets)
        self.actor = core.Actor(num_hidden_layers, hidden_units, num_actions)
        self.critic1 = core.Critic(num_hidden_layers, hidden_units)
        self.critic2 = core.Critic(num_hidden_layers, hidden_units)
        self.critic1_target = core.Critic(num_hidden_layers, hidden_units)
        self.critic2_target = core.Critic(num_hidden_layers, hidden_units)
        
        # old version of spinning-up with tf1 uses placeholder
        # migrating to tf2 -> tf.keras.Input
        # initialize Critics with these Inputs to build input layers
        input1 = tf.keras.Input(shape=(state_dim), dtype=tf.float32)
        input2 = tf.keras.Input(shape=(num_actions), dtype=tf.float32)
        self.critic1(input1, input2)
        self.critic2(input1, input2)
        self.critic1_target(input1, input2)
        self.critic2_target(input1, input2)
        
        # updating weights with a zero-coefficient (not with polyak)
        weights_update(self.critic1_target.variables, self.critic1.variables, 0.0)
        weights_update(self.critic2_target.variables, self.critic2.variables, 0.0)
        
        # we use Adam optimizers with the given learning rate
        self.actor_optimizer = tf.optimizers.Adam(self.learning_rate)
        self.critic1_optimizer = tf.optimizers.Adam(self.learning_rate)
        self.critic2_optimizer = tf.optimizers.Adam(self.learning_rate)
        
    # off-line training -> these arguments come from the replay buffer
    def train(self, states, actions, next_states, rewards, dones, batch_size):
        # getting next policy ...
        next_pi, next_logp_pi = self.actor(next_states)
        
        # ... and using it to compute the Q-targets
        q1_target = self.critic1_target(next_states, next_pi)
        q2_target = self.critic2_target(next_states, next_pi)
        
        # in particular, the Min-Q-target
        min_q_target = tf.minimum(q1_target, q2_target)
        
        # Soft State Value function:
        # V(s_t) = Q(s_t, a_t) - alpha * log(pi(a_t|s_t))
        soft_state_value = min_q_target - self.alpha * next_logp_pi
        soft_state_value = tf.reshape(soft_state_value, [-1])
        
        # Soft Q-value function:
        # Q(s_t, a_t) = r(s_t, a_t) + gamma * (1 - done) * V(s_t)
        soft_q_value = rewards + self.gamma * (1 - dones) * soft_state_value
        soft_q_value = tf.reshape(soft_q_value, [batch_size, 1])
        
        # now compute the loss for the:
        # - first critic
        with tf.GradientTape() as tape1:
            q1 = self.critic1(states, actions)
            critic1_loss = tf.reduce_mean((q1 - soft_q_value)**2)
        
        # - second critic
        with tf.GradientTape() as tape2:
            q2 = self.critic2(states, actions)
            critic2_loss = tf.reduce_mean((q2 - soft_q_value)**2)
        
        # - actor
        with tf.GradientTape() as tape3:
            pi, logp_pi = self.actor(states)
            q1 = self.critic1(states, pi)
            q2 = self.critic2(states, pi)
            min_q = tf.minimum(q1, q2)
            
            actor_loss = tf.reduce_mean(logp_pi * tf.stop_gradient(logp_pi - min_q))
            
        # computing gradients as shown in class...
        actor_gradients = tape3.gradient(actor_loss, self.actor.trainable_variables)
        critic1_gradients = tape1.gradient(critic1_loss, self.critic1.trainable_variables)
        critic2_gradients = tape2.gradient(critic2_loss, self.critic2.trainable_variables)
        
        # ... and applying gradient descent step for each gradient
        self.actor_optimizer.apply_gradients(zip(actor_gradients, self.actor.trainable_variables))
        self.critic1_optimizer.apply_gradients(zip(critic1_gradients, self.critic1.trainable_variables))
        self.critic2_optimizer.apply_gradients(zip(critic2_gradients, self.critic2.trainable_variables))
        
        # updating weights with polyak coefficient
        weights_update(self.critic1_target.variables, self.critic1.variables, self.polyak)
        weights_update(self.critic2_target.variables, self.critic2.variables, self.polyak)
        
        # returning the losses
        return critic1_loss, critic2_loss, actor_loss
        
    # the actor takes an action given a state
    def take_action(self, state):
        action = self.actor(state[None, :])[0][0]
        return action
        
        
        
        
