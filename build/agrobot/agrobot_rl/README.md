#Reinforement learning bringup for the Agrobot:
- bringup robot (in simulation)
- bringup environment
- bringup agent
- start training?

# RL-environment:
## Observation space:
- Occupancy gridmap or ...
- Add in fruit locations as coloured boxes
- Fixed maximum map size

## Action space:
- Navigation goal

## Reward strategy:
- High reward for setting navigation goal on fruit location
- High reward for setting navigation goal close to fruit location
- Small reward for setting goal in general direction of fruit
- Reward for increasing knowledge of the map (occupancy grid)
- Negative reward for every navigation goal set

## Find out:
- how are navigation goals set
- what should the output of the network be (continuous?)
- how to get map from map service to agent
- 