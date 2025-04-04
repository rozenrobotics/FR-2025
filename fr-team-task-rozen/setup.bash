#!/bin/bash -i
set -o pipefail

echo 'Edit ~/.zshrc'
rospd world_generator > /dev/null
WORLD_GENERATOR_MODEL_PATH='export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:'$(pwd)'/models/dronepoint'
grep -qxF "${WORLD_GENERATOR_MODEL_PATH}" ~/.zshrc || echo ${WORLD_GENERATOR_MODEL_PATH} >> ~/.zshrc
popd > /dev/null

echo 'Edit clover.launch'
rospd clover/ > /dev/null
xmlstarlet edit -P -O --inplace --update '/launch/arg[@name="aruco"]/@default' -v true launch/clover.launch
xmlstarlet edit -P -O --inplace --update '/launch/arg[@name="rosbridge"]/@default' -v true launch/clover.launch

echo 'Edit aruco.launch'
xmlstarlet edit -P -O --inplace --update '/launch/arg[@name="aruco_detect"]/@default' -v true launch/aruco.launch
xmlstarlet edit -P -O --inplace --update '/launch/arg[@name="aruco_map"]/@default' -v true launch/aruco.launch
xmlstarlet edit -P -O --inplace --update '/launch/arg[@name="map"]/@default' -v cmit.txt launch/aruco.launch
popd > /dev/null

echo 'Done!'
