from gym.envs.registration import register

register(
    id='Stage-v0',
    entry_point='gym_stage.envs:StageEnv',
)
