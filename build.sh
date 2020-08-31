#!/bin/sh

set -eu

PROJECT=ch55xjtag

docker build -t ${PROJECT} -f Dockerfile .
docker run -v "${PWD}:/mnt" ${PROJECT} bash -c "cp -v /code/src/*.hex /mnt"
