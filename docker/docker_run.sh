#! /bin/bash
# enable access to xhost from the container

if [[ $1 = "bash" ]]; then
	option="--entrypoint=/bin/bash"
fi

xhost +
docker run -it --rm --privileged \
    -v `pwd`:/home/root/src:rw \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -e DISPLAY=:0 \
    --name=openrealm_docker $option openrealm:latest