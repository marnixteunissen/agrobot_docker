# Notes:

In order to avoid errors with tf2, the agrobot rl environment should be run WITHOUT rviz, 
this would give tf_old_data errors since the clock in rviz is not reset once the simulatiojn resets.

run:
cd ~/agrobot_ws/src/agrobot/agrobot_rl/ && pip install -e gym-agrobot
to install the agrubot gym environment