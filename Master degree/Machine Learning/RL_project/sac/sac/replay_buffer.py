import numpy as np

# defining the class of the replay buffer used to store the distribution
# of the previously sampled states, actions, rewards and dones
# N.B. Implemented as a FIFO
class ReplayBuffer:
    # init
    def __init__(self, state_dim, action_dim, size):
        # class as a set of buffers that at the beginning are all empty
        # buffers for states and actions
        self.states = np.zeros([size, state_dim], dtype=np.float32)
        self.next_states = np.zeros([size, state_dim], dtype=np.float32)
        self.actions = np.zeros([size, action_dim], dtype=np.float32)

        # buffers for rewards and dones (if task completed)
        self.rewards = np.zeros(size, dtype=np.float32)
        self.dones = np.zeros(size, dtype=np.float32)

        # remaining fields
        self.ptr = 0
        self.size = 0
        self.max_size = size

    # saving data in the form (state, action, reward, next_state, done)
    def save(self, state, action, reward, next_state, done):
        # saving data...
        self.states[self.ptr] = state
        self.next_states[self.ptr] = next_state
        self.actions[self.ptr] = action
        self.rewards[self.ptr] = reward
        self.dones[self.ptr] = done
        # ... and uploading the "index" and the current size
        self.ptr = (self.ptr + 1) % self.max_size
        new_size = self.size + 1
        if new_size < self.max_size:
            self.size = new_size
        else:
            self.size = self.max_size

    # returning a sampled batch of given size
    def sample_batch(self, size=32):
        # array of size indices in [0, ReplayBuffer.size]
        rand_idx = np.random.randint(0, self.size, size=size)
        return self.states[rand_idx], self.next_states[rand_idx], self.actions[rand_idx], self.rewards[rand_idx], self.dones[rand_idx]
    
    