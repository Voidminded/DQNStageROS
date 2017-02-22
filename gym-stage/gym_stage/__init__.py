from gym.envs.registration import register

register(
    id='stage-v0',
    entry_point='gym_stage.envs:StageEnv',
)
