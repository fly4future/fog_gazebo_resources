# Gazebo resources

### Using more than 1 drone

-Modify `~git/px4_firmware/ROMFS/px4fmu_common/init.d-posix/airframes/4400_ssrc_fox_rtps_x.post` to contain
```
micrortps_client start -t UDP -r $((2019+2*px4_instance)) -s $((2020+2*px4_instance))
```
and rebuild. (Or modify same file inside build)


