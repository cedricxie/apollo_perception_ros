#!/bin/sh

DOCKER_NAME=cedricxie/apollo-perception-ros
DOCKERFILE_NAME=Dockerfile_apollo_perception_ros.gpu

nvidia-docker build -t $DOCKER_NAME -f $DOCKERFILE_NAME . --no-cache
