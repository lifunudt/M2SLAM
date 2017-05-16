
# M2SLAM

**Authors:** Fu Li, Shaowu Yang

**Current version:** 1.0.0

## introduction

**M2SLAM** is a novel visual SLAM system with **memory management** to overcome two major challenges in reducing memory con- sumption of visual SLAM: efficient map data scheduling between the memory and the external storage, and map data persistence method (i.e., the data out- lives the process that created it). We redesign the framework of a visual SLAM system to contain a SLAM front end and a database server end. The front end maintains a localization and a mapping process, as well as map data scheduling between the memory and the database. We propose a map data scheduling method to organize map data considering the local spatial character among keyframes. The server end provides data services from a spatial database for global map data storage and accessing.

<img src="https://github.com/lifunudt/M2SLAM/blob/master/image/framework.png" alt="M2SLAM" height="180" />

The framework of **M2SLAM** is shown above, which includes the front end and the server end. We implement the front end based on **[ORB_SLAM2[1-2]](https://github.com/raulmur/ORB_SLAM2)** by extending its tracking, local mapping, and loop closing module, and adding one novel memory managing module. The server end publishes data services for map data storage and accessing.
## 0. Related Publications

## 1. Prerequisites

## 2. Building and Run M2SLAM

### 2.1 ORBSLAM2 and its dependencies

Our M2SLAM system is build on the foundation of [ORB_SLAM2(https://github.com/raulmur/ORB_SLAM2). You should follow the instructions provided by ORB_SLAM2 build its dependencies. We do not list here.

### 2.2 Boost library install
We use boost library to serialize and deserialize the data.
We can install boost library using the following instruction in terminal.
```bash
sudo apt-get instal libboost-dev
```
### 2.2 ODB environment build

We use [ODB](http://www.codesynthesis.com/products/odb/) to solve the issue of mapping a C++ object to a record in a database table, instead of using SQL.

You can use the ODB packages in src/orbslam_client/Thirdparty/package or download from the [ODB fficial website](http://www.codesynthesis.com/products/odb/).

Unpack the packages and build the ODB according to the **INSTALL** file in their dirs.

The following errors may occur:

> configure : error: g++ does not support plugins; reconfigure GCC with --enable-plugin

```bash
sudo apt-get update
sudo apt-get install gcc-4.8-plugin-dev
```

> error: libcutl is not found

Download the source code of the libcutl from the [office website](https://curl.haxx.se/libcurl/).

Install the package according to the **INSTALL** file.

> Can not find odb, when test the odb-example

The reason is that the install directory */usr/local/lib* has not in the *ld.so.conf* file.

```
echo "/usr/local/lib" >> /etc/ld.so.conf
echo "/usr/local/bin" >> /etc/ld.so.conf
sudo ldconfig
```

### 2.3 PostGIS install

### 2.4 ROS install

## Reference
[1] Mur-Artal R, Montiel J M M, Tardos J D. ORB-SLAM: a versatile and accurate monocular SLAM system[J]. IEEE Transactions on Robotics, 2015, 31(5): 1147-1163.

[2] Mur-Artal R, Tardos J D. ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras[J]. arXiv preprint arXiv:1610.06475, 2016.

## License
M2SLAM is released under a [GPLv3 license](https://github.com/lifunudt/M2SLAM/blob/master/License-gpl.txt).

For a closed-source version of M2SLAM for commercial purposes, please contact the authors.
