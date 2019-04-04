#!/bin/sh

if [ "$1" = "" ]
then
    DOCKER_NAME=cedricxie/apollo-perception-ros:latest
else
    DOCKER_NAME=cedricxie/apollo-perception:$1
fi

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
SHARED_DIR=/home/yx/shared_dir
HOST_DIR=/home/$USER/shared_dir

mkdir -p $HOST_DIR
echo "Shared directory: ${HOST_DIR}"

docker run \
    -it --rm \
    --volume=$HOST_DIR:$SHARED_DIR:rw \
    --volume=$XSOCK:$XSOCK:rw \
    --volume=$XAUTH:$XAUTH:rw \
    --env="XAUTHORITY=${XAUTH}" \
    --env="DISPLAY=${DISPLAY}" \
    --env="QT_X11_NO_MITSHM=1" \
    -u yx \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /dev/video0:/dev/video0 \
    --net=host \
    --runtime=nvidia \
    $DOCKER_NAME

