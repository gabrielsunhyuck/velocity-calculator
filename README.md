# velocity_calculator

## 1. utm_converter.launch
You can convert GPS raw Data to utm position.

## 2. velocity_gps.launch
You can calculate velocity of the object which is detected by GPS.

Object's Position > **DIFFERENTIAL** > Object's Velocity

## 3. velocity_imu.launch
You can calculate velocity of the object which is detected by IMU.

Object's Acceleration > **INTEGRAL** > Object's Velocity

- errors accumulate soon after

- you have to utilize appropriate filter in neccessarily

- This launch file didn't use some filters, so There will be some errors.

## 4. velocity_ouster.launch
You can calculate velocity of the object which is detected by OUSTER(3D-LiDAR)

Object's Position > **DEFFERENTIAL** > Object's Velocity
