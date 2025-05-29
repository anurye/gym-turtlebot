from gymnasium.envs.registration import register

register(
    id='Turtlebot4Env-v0',
    entry_point='tb4_drl_navigation.envs.diffdrive.turtlebot4:Turtlebot4Env',
    max_episode_steps=500,
)
