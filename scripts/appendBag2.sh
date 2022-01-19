# #{ appendBag2()

appendBag2() {

  if [ "$#" -ne 1 ]; then
    echo ERROR: please supply one argument: the text that should be appended to the name of the folder with the latest rosbag file and logs
  else

    bag_adress=`readlink ~/bag_files/latest`

    if test -d "$bag_adress"; then

      echo $bag_adress
      appended_adress=$bag_adress\_$1
      echo $appended_adress 
      mv $bag_adress $appended_adress
      ln -sf $appended_adress ~/bag_files/latest

      echo Rosbag name appended: $appended_adress

      # echo $appended_adress/tmux/5_Control.log
      journalctl -u agent_protocol_splitter -b > $appended_adress/tmux/2_ProtocolSplitter.log
      journalctl -u mavlink-router -b > $appended_adress/tmux/3_MavlinkRouter.log
      journalctl -u micrortps-agent -b > $appended_adress/tmux/4_MicroRtpsAgent.log
      journalctl -u control_interface -b > $appended_adress/tmux/5_Control.log
      journalctl -u navigation -b > $appended_adress/tmux/6_Navigation.log
      journalctl -u odometry -b > $appended_adress/tmux/7_Odometry.log
      journalctl -u bumper -b > $appended_adress/tmux/8_Bumper.log

    else
      echo ERROR: symlink ~/bag_files/latest does not point to a file! - $bag_adress
    fi
  fi

}

# #}
