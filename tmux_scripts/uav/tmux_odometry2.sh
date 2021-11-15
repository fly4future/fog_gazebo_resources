#!/bin/bash
### BEGIN INIT INFO
# Provides: tmux
# Required-Start:    $local_fs $network dbus
# Required-Stop:     $local_fs $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start the uav
### END INIT INFO
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

# change this to your liking
PROJECT_NAME=ros2_integration

# do not change this
MAIN_DIR=~/"bag_files"

# following commands will be executed first in each window
pre_input="mkdir -p $MAIN_DIR/$PROJECT_NAME"

SESSION_NAME=mav

# prefere the user-compiled tmux
if [ -f /usr/local/bin/tmux ]; then
  export TMUX_BIN=/usr/local/bin/tmux
else
  export TMUX_BIN=/usr/bin/tmux
fi

# find the session
FOUND=$( $TMUX_BIN ls | grep $SESSION_NAME )

if [ $? == "0" ]; then

  echo "The session already exists"
  exit
fi

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  touch "$ITERATOR_FILE"
  ITERATOR="1"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d__%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# define commands
# 'name' 'command'
# DO NOT PUT SPACES IN THE NAMES
input=(
  'Rosbag' 'ros2 bag record -a -o '"$SUBLOG_DIR"'/bag
'
  # 'Rosbag' 'ros2 bag record -a -o '"$MAIN_DIR"'/'"$PROJECT_NAME"'/test.bag
# '
  'ProtocolSplitter' 'protocol_splitter -b 1000000 -d /dev/ttyS7 -x 14540 -w 14580 -y 2019 -z 2020 -v
'
  'MicroRTPSagent' 'micrortps_agent -t UDP -r 2020 -s 2019 -n '"$UAV_NAME"'
'
  'Control' 'ros2 launch control_interface control_interface.py
'
  'Bumper' 'ros2 launch fog_bumper bumper_launch.py
'
  'Odometry' 'ros2 launch odometry2 odometry2_real.py
'
  'Change Odometry' 'ros2 service call /'"$UAV_NAME"'/odometry2/change_odometry_source fog_msgs/srv/ChangeOdometry "{odometry_type:{type: 0}}"'

  'Hector' 'ros2 launch hector_mapping hector_mapping_real.py
'
  'Hector reset' 'ros2 service call /'"$UAV_NAME"'/odometry2/reset_hector_service std_srvs/srv/Trigger'
  'Change param' 'ros2 service call /'"$UAV_NAME"'/control_interface/set_px4_param_float fog_msgs/srv/SetPx4ParamFloat "{param_name: EKF2_EVP_NOISE , value: 0.1}"'
  'VisualOdom' 'ros2 topic echo /tii/VehicleVisualOdometry_PubSubTopic | grep -A4 "local_frame"'
  'VehicleOdom' 'ros2 topic echo /tii/VehicleOdometry_PubSubTopic | grep -A4 "local_frame"'

  'Navigation' 'ros2 launch navigation navigation.py
'
  'Octomap' 'ros2 launch octomap_server2 octomap_server.py
'
  'LocalOdom' 'ros2 topic echo /'"$UAV_NAME"'/odometry2/local_odom'
  'Rplidar' ' ros2 launch rplidar_ros2 sensors_launch.py
'
  'Takeoff' 'ros2 service call /'"$UAV_NAME"'/control_interface/takeoff std_srvs/srv/Trigger {}'
  'Raw goto' 'ros2 service call /'"$UAV_NAME"'/control_interface/local_waypoint fog_msgs/srv/Vec4 "goal: [0,0,2,0]"'
  'Planning goto' 'ros2 service call /'"$UAV_NAME"'/navigation/local_waypoint fog_msgs/srv/Vec4 "goal: [0,0,2,0]"'
  'Land' 'ros2 service call /'"$UAV_NAME"'/control_interface/land std_srvs/srv/Trigger {}'
)

init_window="Status"

###########################
### DO NOT MODIFY BELOW ###
###########################

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

if [ -z ${TMUX} ];
then
  TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
  echo "Starting new session."
else
  echo "Already in tmux, leave it first."
  exit
fi

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest"
rm "$MAIN_DIR/latest"
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
  ((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  $TMUX_BIN send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;${pre_input};${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

$TMUX_BIN select-window -t $SESSION_NAME:$init_index

$TMUX_BIN -2 attach-session -t $SESSION_NAME

clear
