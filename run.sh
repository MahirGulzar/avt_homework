#!/bin/bash

cd docker/
docker build -t avt_hw .
docker run --gpus all --network=host --rm -it avt_hw
