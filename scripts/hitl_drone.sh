#!/usr/bin/env bash

SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

if [[ $# -le 2 ]]; then 
  uav_id=0 
else
  uav_id=$3
fi
export MAV_SYS_ID="${uav_id}"

set -e

px4_firmware_path="$1"
model_name="$2"
model="ssrc_fog_x"
verbose="--verbose"

src_path="$px4_firmware_path"
sitl_bin="$px4_firmware_path/build/px4_sitl_rtps/bin/px4"
build_path="$px4_firmware_path/build/px4_sitl_rtps"
models_path="$px4_firmware_path/Tools/sitl_gazebo/models"

# The rest of the arguments are files to copy into the working dir.

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo model: $model
echo src_path: $src_path
echo build_path: $build_path

rootfs="$build_path/tmp/rootfs" # this is the working directory
mkdir -p "$rootfs"

modelpath=$models_path

echo "Recompiling model using Jinja"
python3 ${src_path}/Tools/sitl_gazebo/scripts/jinja_gen.py ${src_path}/Tools/sitl_gazebo/models/${model}/${model}.sdf.jinja ${src_path}/Tools/sitl_gazebo --mavlink_tcp_port $((4560+${uav_id})) --mavlink_udp_port $((14560+${uav_id})) --mavlink_id $((1+${uav_id})) --gstudpport $((5600+${uav_id})) --video_uri $((5600+${uav_id})) --mavlink_cam_udp_port $((14530+${uav_id})) --output-file /tmp/${model_name}.sdf --vehicle_name ${model_name} --hil_mode 1 --serial_enabled 1 --serial_device /dev/ttyUSB0 --serial_baudrate 2000000 --lockstep 0 \
 # --tcp_client_mode 0 --use_tcp 0

echo "Spawning model: /tmp/${model_name}.sdf"
while gz model --verbose --spawn-file="/tmp/${model_name}.sdf" -m ${model_name} -x $((1+${uav_id}*2)) -y $((1+${uav_id}*2)) -z 0.5 2>&1 | grep -q "An instance of Gazebo is not running."; do
  echo "gzserver not ready yet, trying again!"
  sleep 1
done

echo "Model spawned!"

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

sitl_command="\"$sitl_bin\" -i ${uav_id} -d \"$build_path/etc\" -w sitl_${model_name} -s etc/init.d-posix/rcS -t \"$src_path/test_data\""

echo SITL COMMAND: $sitl_command

export PX4_SIM_MODEL=${model}

# eval $sitl_command

popd >/dev/null
