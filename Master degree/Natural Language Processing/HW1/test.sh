#!/bin/bash

# initial check

if [ "$#" != 1 ]; then
    echo "$# parameters given. Only 1 expected. Use -h to view command format"
    exit 1
fi

if [ "$1" == "-h" ]; then
  echo "Usage: `basename $0` [file to evaluate upon]"
  exit 1
fi

test_path=$1

# delete old docker if exists
docker ps -q --filter "name=nlp2021-hw1" | grep -q . && docker stop nlp2021-hw1
docker ps -aq --filter "name=nlp2021-hw1" | grep -q . && docker rm nlp2021-hw1

# build docker file
docker build . -f Dockerfile -t nlp2021-hw1

# bring model up
docker run -d -p 12345:12345 --name nlp2021-hw1 nlp2021-hw1

# perform evaluation
/usr/bin/env python hw1/evaluate.py $test_path

# stop container
docker stop nlp2021-hw1

# dump container logs
docker logs -t nlp2021-hw1 > logs/server.stdout 2> logs/server.stderr

# remove container
docker rm nlp2021-hw1