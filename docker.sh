#!/bin/bash

IMAGE_TAG=dense_sptam:latest

case "$1" in
  run)
    docker run -it \
    --net=host \
    -v /etc/group:/etc/group:ro \
    -v /etc/passwd:/etc/passwd:ro \
    -v $PWD:/usr/src/dense_sptam \
    -v $HOME/.ssh:/home/$( id -u -n)/.ssh:ro \
    -v $SSH_AUTH_SOCK:/ssh-agent \
    -u $( id -u ):$( id -g ) \
    --mount type=tmpfs,destination=/home/$( id -u -n) \
    -e "SSH_AUTH_SOCK=/ssh-agent" \
    $IMAGE_TAG \
    /bin/bash
    ;;
  build)
    docker build -t $IMAGE_TAG .
    ;;
  *)
    echo "Usage: $0 {run|build}"
    exit 1
esac
