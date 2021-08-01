from typing import Dict, List, Optional, Tuple

import dataclasses
import numpy as np
import os
import gym
from gym import error, spaces
from gym import utils
from gym.utils import seeding
try:
    import atari_py
except ImportError as e:
    raise error.DependencyNotInstalled(
            "{}. (HINT: you can install Atari dependencies by running "
            "'pip install gym[atari]'.)".format(e))
            
@dataclasses.dataclass
class Transition:
    state: Tuple[int, int]
    action: str
    next_state: Tuple[int, int]
    reward: float
    termination: bool
        
class SpaceEnv(gym.Env, utils.EzPickle):
    _states: np.array
    _rewards: np.array
    _action_semantics: List[str]
    _actions: Dict[str, np.array]
    _init_state: Tuple[int, int]
    _current_state: Tuple[int, int]
    _obstacles: List[Tuple[int, int]]
    _aliens: List[Tuple[int, int]]
    _transition_probabilities: np.array
    


    def __init__(self,
                 rows = 8,
                 cols = 7,
                 aliens = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6)],
                 obstacles = [(4, 1), (5, 1), (4, 3), (5, 3), (4, 5), (5, 5)]) -> None:

        
        self._states = np.zeros((rows, cols))
        self._rewards = np.zeros((rows, cols))
        for i in range(7):
          self._rewards[7][i] = 3
        self._init_state = (7, 1)
        self._current_state = self._init_state
        self._obstacles = [(4, 1), (5, 1), (4, 3), (5, 3), (4, 5), (5, 5)]
        self._aliens = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6)]
        self._action_semantics = ['left', 'right', 'fire']
        self._actions = np.array([[0, -1], [0, 1], self._current_state])
        self.action_space = spaces.Discrete(len(self._action_semantics))
        
        # going right, straight, left wrt chosen action
        self._transition_probabilities = np.array([0.00, 1.00, 0.00])
        
    @property
    def actions(self) -> List[str]:
        return self._action_semantics
    
    @property
    def current_state(self) -> Tuple[int, int]:
        return self._current_state
    
    @property
    def reward(self) -> float:
        r, c = self._current_state
        return self._rewards[r, c]
    
    @property
    def termination(self) -> bool:
        the_end = len(self._aliens) == 0
        return the_end
    
    def render(self) -> None:
        for r, c in self._obstacles:
            self._states[r, c] = 1
        grid = np.array(self._states, dtype=str)
        r, c = self._current_state
        grid[r, c] = ' X '
        for r, c in self._aliens:
          grid[r, c] = ' A '
        print(grid)
        
    def _transition(self, state: Tuple[int, int], a: np.array) -> Tuple[int, int]:
        n_actions = len(self._actions)
        ind = a + n_actions if a < 0 else a % n_actions
        a = self._actions[ind]
        if (ind == 0 or ind == 1):
          new_r = max(0, min(self._states.shape[0] - 1, state[0] + a[0]))
          new_c = max(0, min(self._states.shape[1] - 1, state[1] + a[1]))
          return (new_r, new_c) if self._states[new_r, new_c] == 0. else state
        elif (ind == 2):
          for i in range(-7, 1):
            i = -i
            if((i, state[1]) in self._obstacles):
              self._obstacles.remove((i, state[1]))
              self._states[i, state[1]] = 0
              break
            elif((i, state[1]) in self._aliens):
              self._aliens.remove((i, state[1]))
              self._rewards[7][state[1]]-=1
              break
          return state 
        
    def step(self, a_idx: int) -> Transition:
        action = self._action_semantics[a_idx]
        print('\n')
        print(action)
        rnd = np.random.rand()
        chosen_action = a_idx + np.random.choice([1, 0, -1], p=self._transition_probabilities)
        prev_state = self._current_state
        self._current_state = self._transition(self._current_state, chosen_action)
        return Transition(state=prev_state,
                          action=action,
                          next_state=self._current_state,
                          reward=self.reward,
                          termination=self.termination)
    
    def reset(self) -> None:
        self._current_state = self._init_state
        for i in range(7):
          self._rewards[7][6] = 3
        self._obstacles = [(4, 1), (5, 1), (4, 3), (5, 3), (4, 5), (5, 5)]
        self._aliens = [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6), (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), (1, 6), (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), (2, 6)]
        
    def state_space_size(self) -> Tuple[int, int]:
        return self._states.shape
    
    def action_space_size(self) -> int:
        return len(self._actions)
