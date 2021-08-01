from gym.envs.registration import register

register(
    id='MySpaceInvaders-v0',
    entry_point='gym_space.envs:SpaceEnv',
)
