#!/usr/bin/env bash

SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

set -e

px4_firmware_path="$1"
program="gazebo"
model_name="$2"
model="ssrc_fog_x"
world="forest.world"
worlds_path="$SCRIPTPATH/../worlds"
verbose="--verbose"

src_path="$px4_firmware_path"
sitl_bin="$px4_firmware_path/build/px4_sitl_rtps/bin/px4"
build_path="$px4_firmware_path/build/px4_sitl_rtps"
models_path="$px4_firmware_path/Tools/sitl_gazebo/models"

# The rest of the arguments are files to copy into the working dir.

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo debugger: $debugger
echo program: $program
echo model: $model
echo world: $world
echo src_path: $src_path
echo build_path: $build_path

rootfs="$build_path/tmp/rootfs" # this is the working directory
mkdir -p "$rootfs"

modelpath=$models_path

echo "Recompiling model using Jinja"
python3 ${src_path}/Tools/sitl_gazebo/scripts/jinja_gen.py ${src_path}/Tools/sitl_gazebo/models/${model}/${model}.sdf.jinja ${src_path}/Tools/sitl_gazebo --mavlink_tcp_port 4560 --mavlink_udp_port 14560 --mavlink_id 1 --gst_udp_port 5600 --video_uri 5600 --mavlink_cam_udp_port 14530 --output-file /tmp/${model_name}.sdf --vehicle_name ${model_name}

echo "Spawning model: /tmp/${model_name}.sdf"
while gz model --verbose --spawn-file="/tmp/${model_name}.sdf" -m ${model_name} -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -q "An instance of Gazebo is not running."; do
  echo "gzserver not ready yet, trying again!"
  sleep 1
done

echo "Model spawned!"

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="\"$sitl_bin\" $no_pxh \"$build_path\"/etc -s etc/init.d-posix/rcS -t \"$src_path\"/test_data"

echo SITL COMMAND: $sitl_command

export PX4_SIM_MODEL=${model}

eval $sitl_command

popd >/dev/null
