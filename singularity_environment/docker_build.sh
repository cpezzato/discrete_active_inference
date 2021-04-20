#!/bin/bash

BASENAME="ro47014"
VERSION="20-10-3"

#echo $VERSION > hrwros_image_id

docker build --build-arg TARGET_ROS_VERSION=melodic -t $BASENAME:$VERSION .

#rm hrwros_image_id

