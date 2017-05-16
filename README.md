
# M2SLAM

**Authors:** Fu Li, Shaowu Yang

**Current version:** 1.0.0

### introduction

**M2SLAM** is a novel visual SLAM system with **memory management** to overcome two major challenges in reducing memory con- sumption of visual SLAM: efficient map data scheduling between the memory and the external storage, and map data persistence method (i.e., the data out- lives the process that created it). We redesign the framework of a visual SLAM system to contain a SLAM front end and a database server end. The front end maintains a localization and a mapping process, as well as map data scheduling between the memory and the database. We propose a map data scheduling method to organize map data considering the local spatial character among keyframes. The server end provides data services from a spatial database for global map data storage and accessing.

<img src="https://github.com/lifunudt/M2SLAM/blob/master/image/framework.png" alt="M2SLAM" height="180" />

The framework of **M2SLAM** is shown above, which includes the front end and the server end. We implement the front end based on **[ORB_SLAM2[1-2]](https://github.com/raulmur/ORB_SLAM2)** by extending its tracking, local mapping, and loop closing module, and adding one novel memory managing module. The server end publishes data services for map data storage and accessing.
### 0. Related Publications

### 1. License

### 2. Prerequisites

### 3. Building and Run M2SLAM

### Reference
[1] Mur-Artal R, Montiel J M M, Tardos J D. ORB-SLAM: a versatile and accurate monocular SLAM system[J]. IEEE Transactions on Robotics, 2015, 31(5): 1147-1163.

[2] Mur-Artal R, Tardos J D. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras[J]. arXiv preprint arXiv:1610.06475, 2016.
