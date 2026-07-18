# FAST-LIVO2 software arrival-time synchronization

This branch adds `fast_livo_soft_time_sync` as a preprocessing node. The
FAST-LIVO2 estimator and its internal `sync_packages()` implementation remain
unchanged.

## Data flow

Raw topics:

```text
/livox/lidar
/livox/imu
/hikrobot_camera/rgb
```

Synchronized topics consumed by FAST-LIVO2:

```text
/soft_sync/livox/lidar
/soft_sync/livox/imu
/soft_sync/camera/image
```

The node uses Ubuntu wall-clock arrival time:

1. IMU messages are stamped with their host arrival time and forwarded.
2. Images are stamped with their host arrival time and buffered.
3. Each complete Livox frame is assigned a host frame-end arrival time. Its
   frame-start stamp is recovered from the maximum Livox `offset_time`, so
   per-point timing remains usable for undistortion.
4. After a short wait, each LiDAR frame consumes the image with the smallest
   host arrival-time difference. The output image is stamped at the matched
   LiDAR frame-end time.
5. A LiDAR frame is dropped when no image falls within `max_time_diff_sec`.
   A matched image is removed, providing one-to-one pairing.

## Running

Build with the Livox workspace sourced:

```bash
source /opt/ros/noetic/setup.bash
source ~/ws_livox/devel/setup.bash
cd ~/pc_pose
catkin_make
source devel/setup.bash
roslaunch fast_livo mapping_avia.launch
```

The launch arguments can be changed for a particular sensor setup:

```bash
roslaunch fast_livo mapping_avia.launch \
  raw_lidar_topic:=/livox/lidar \
  raw_imu_topic:=/livox/imu \
  raw_image_topic:=/hikrobot_camera/rgb \
  match_wait_sec:=0.10 \
  max_time_diff_sec:=0.05
```

Check synchronization:

```bash
rostopic hz /soft_sync/livox/lidar
rostopic hz /soft_sync/livox/imu
rostopic hz /soft_sync/camera/image
```

The node prints the latest LiDAR/image arrival-time difference and drop count.
If the LiDAR and camera rates differ substantially, strict one-to-one matching
necessarily drops messages from the faster stream.

## Scope and limitations

This is host-arrival-time synchronization. It removes incompatible device
clock epochs and gives FAST-LIVO2 a common monotonic time domain, but it cannot
recover the true optical exposure time or remove variable USB/network latency.
For fast motion and high-accuracy calibration, hardware triggering or PTP/PPS
remains preferable.



export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
