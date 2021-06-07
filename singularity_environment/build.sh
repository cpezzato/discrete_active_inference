#!/bin/bash
set -e


BASENAME="ro47014"
VERSION="20-10-3"


docker tag ${BASENAME}:${VERSION} localhost:5000/${BASENAME}:${VERSION}
docker push localhost:5000/${BASENAME}:${VERSION}


echo $VERSION > hrwros_image_id
sudo SINGULARITY_NOHTTPS=1 singularity build ${BASENAME}-${VERSION}.simg Singularity
rm hrwros_image_id
