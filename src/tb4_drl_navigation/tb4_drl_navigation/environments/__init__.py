from gymnasium.envs.registration import register

register(
    id='Turtlebot4Env-v0',
    entry_point='tb4_drl_navigation.environments.envs.turtlebot4_env:Turtlebot4Env',
    max_episode_steps=500,
)
