# Instructions to setup the docker simulation

1. Clone this folder to your local PC
2. Make sure that docker is installed (https://docs.docker.com/engine/install/ubuntu/), and you are able to run docker without sudo priveleges (https://docs.docker.com/engine/install/linux-postinstall/).

# To run
## from the dockerhub image
1. Run the docker container `docker run -it --rm -p 6080:80 vinsento/agrobot_docker`
2. Access the environment via your browser on `http://localhost:6080/`

## Building from source (only if you want to make changes to the docker build)
1. go to the docker folder in your terminal `cd docker`
2. build the docker image `docker build . -t agrobot_docker`
3. Run the docker container `docker run -it --rm -p 6080:80 agrobot_docker`
4. Access the environment via your browser on `http://localhost:6080/`

# Run the gazebo simulation
1. open a terminal (terminator) and cd into the catkin_ws folder `cd catkin_ws`
2. launch gazebo: `./launch_turtlbot.sh`
3. launch rviz: `./launch_rviz.sh`
4. launch moveit: `./launch_moveit.sh`
5. launch GUI controller: `./launch_controller_gui.sh`

# Working with single command line
If you want to run all in one terminal, make sure you type a `&` after each command, so for example: `roscore &`
