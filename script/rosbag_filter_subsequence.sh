INBAG=$1
OUTBAG=$2
STARTTIME=$3
DURATION=$4
ENDTIME="($STARTTIME + $DURATION)"

rosbag filter $INBAG $OUTBAG \
"((
( (topic == '/camera/image_mono') or (topic == 'camera/image_mono') or
  (topic == '/camera/camera_info') or (topic == 'camera/camera_info') or
  (topic == '/bosch_imu/imu0/data_raw') or (topic == 'bosch_imu/imu0/data_raw') or
  (topic == '/ps') or (topic == 'ps') or
  (topic == '/xsens_imu/imu/data') or (topic == 'xsens_imu/imu/data') or
  (topic == '/imu/data') or (topic == 'imu/data') or
  (topic == '/vps_pose_covariance') or (topic == 'vps_pose_covariance') or
  (topic == '/vps_pose_covariance_camera') or (topic == 'vps_pose_covariance_camera') or
  (topic == '/vps_pose_covariance_imu') or (topic == 'vps_pose_covariance_imu') )
and
  (m.header.stamp.to_sec() >= $STARTTIME and m.header.stamp.to_sec() < $ENDTIME)
) or
(
  ( (topic == '/richimage') or (topic == 'richimage') )
and
  (m.image.header.stamp.to_sec() >= $STARTTIME and m.image.header.stamp.to_sec() < $ENDTIME)
) or
(
  ( (topic == '/camera_measurement_info') or (topic == 'camera_measurement_info') )
and
  (m.measurement.richimage.image.header.stamp.to_sec() >= $STARTTIME and m.measurement.richimage.image.header.stamp.to_sec() < $ENDTIME)
) or
(
  ( (topic == '/tf') or (topic == '') )
and
  (m.transforms[0].header.stamp.to_sec() >= $STARTTIME and m.transforms[0].header.stamp.to_sec() < $ENDTIME)
))"