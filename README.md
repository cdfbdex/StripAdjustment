# Airborne LiDAR Strip Adjustment by Boresight Calibration

### Introduction
<p align="justify">
LiDAR (Light Detection and Ranging) systems are widely used in mobile and airborne mapping at various scales. Airborne surveys enable to map wide areas at high speed and relatively low resolution, while unmanned aerial vehicles (UAV) are used to get a detailed description of local areas. LiDAR systems are composed of a positioning system (GNSS receiver), an inertial measurement unit (IMU) and a LiDAR measuring relative distances to the terrain.
</p>
<p align="justify">
Georeferencing is performed by combining LiDAR angles and distance measurements, IMU attitude angles and GNSS positions with a point positioning mathematical model. This model depends on several parameters which are sources of systematic errors that can be observed by comparing overlapping survey strips [1].
</p>

### Problem
<p align="justify">
While a point cloud  may seem clear at high-scale, most of the time two or more structures appear in point clouds where there should only be one. This is a problem with the consistency of the point cloud due to the alignment of the LiDAR frame and the IMU frame. In the literature this problem has been addressed and it is called boresight calibration. See <b>Figure 1</b> for an example of boresight calibration. 
</p>

![Screenshot](resources/images/BeforeAfterBoresightCalibration.png)
<p align="center"><b>Figure 1.</b> Boresight calibration of scans: before (left) and after (right).</p>

### References
<b>[1]</b> Filin, S. Recovery of systematic biases in laser altimetry data using natural surfaces. Photogramm. Eng. Remote Sens.
2003, 69, 1235–1242.
