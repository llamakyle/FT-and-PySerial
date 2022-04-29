#!/bin/sh
# Build the docker image.

tag="test"
docker build --platform linux/amd64 -t "$tag" -f Dockerfile .
