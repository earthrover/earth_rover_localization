# Earth Rover Localization with Docker

This project and all its dependencies are available as a Docker image from: [earthrover/earth_rover_localization](https://cloud.docker.com/u/earthrover/repository/docker/earthrover/earth_rover_localization)

If you want to build this images yourself the instructions are below:

## install docker

    # install docker
    sudo curl -sSL get.docker.com | sh

    # sets up service
    sudo systemctl enable docker
    sudo groupadd docker

    # allows users to use to use docker
    sudo usermod -aG docker earth

    # installs docker compose
    sudo pip install docker-compose

## to clone this repo

    git clone --recursive --depth=1 https://github.com/earthrover/er_localisation.git


## to build this dockerfile (change the version number)

    docker build --no-cache=true -t="earthrover/earth_rover_localisation:0.0.1" .


## to push new build to dockerhub

    docker login -u <username> -p <password>
    docker push earthrover/earth_rover_localization:0.0.1

## to run using docker

    docker-compose up
