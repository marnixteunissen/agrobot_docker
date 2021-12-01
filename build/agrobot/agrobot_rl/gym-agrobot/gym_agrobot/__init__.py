from gym.envs.registration import register

# Register the agrobot environment(s) to OpenAI Gym:
register(id='agrobot-explore-v0',
         entry_point='gym_agrobot.envs:agrobot-explore-v0',
         max_episode_steps=1000)

register(id='Agrobot-Pick-v0',
         entry_point='gym_agrobot.envs:AgrobotPickEnv',
         max_episode_steps=1000)

# Don't forget to install the environments by running:
# pip install -e gym-agrobot
# in the terminal from the "agrobot_rl" directory
