
# M2SLAM

**Authors:** Fu Li, Shaowu Yang

**Current version:** 1.0.0

M2SLAM is a novel visual SLAM system with memory management to overcome two major challenges in reducing memory con- sumption of visual SLAM: efficient map data scheduling between the memory and the external storage, and map data persistence method (i.e., the data out- lives the process that created it). We redesign the framework of a visual SLAM system to contain a SLAM front end and a database server end. The front end maintains a localization and a mapping process, as well as map data scheduling between the memory and the database. We propose a map data scheduling method to organize map data considering the local spatial character among keyframes. The server end provides data services from a spatial database for global map data storage and accessing.

<img src="https://github.com/lifunudt/M2SLAM/blob/master/image/framework.png" alt="M2SLAM" height="180" />
