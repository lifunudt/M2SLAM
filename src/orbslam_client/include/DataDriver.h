//
// Created by lifu on 2/15/17.
//

#ifndef PROJECT_DATADRIVER_H
#define PROJECT_DATADRIVER_H

#include "System.h"
#include "Cache.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "LightKeyFrame.h"
#include "LightMapPoint.h"
#include "SerializeObject.h"
#include <cstdlib>
#include "ros/ros.h"
#include "boost/serialization/vector.hpp"
#include "boost/serialization/map.hpp"

namespace ORB_SLAM2{

    class Cache;
    class KeyFrame;
    class MapPoint;
    class LightKeyFrame;
    class LightMapPoint;

    class DataDriver {

    public:

        DataDriver( Cache *pCache) : pCacher(pCache) {

        }

        // trans the KeyFrames to ORBSlam server

        void TransKeyFramesToServer( std::vector<KeyFrame *> pKFs);

        // trans the KeyFrames from ORBSlam server
        std::set<KeyFrame *> TransKeyFramesFromServer( std::set<long unsigned int > pKFs );

        // trans the MapPoints to ORBSlam server
        std::vector<long unsigned int > TransMapPointsToServer( std::set<MapPoint *> pMPs);

        // trans the MapPoints from ORBSlam server

        std::set<MapPoint * > TransMapPointsFromServer( std::set<long unsigned int > pLMPs);

        // trans the MapPoints to ORBSlam server
        std::vector<long unsigned int > TransTopoMapPointsToServer( TopoId tId, std::set<MapPoint *> pMPs);

        // trans the MapPoints from ORBSlam server

        std::set<MapPoint * > TransTopoMapPointsFromServer( TopoId tId );

        std::vector<long unsigned int>  TransTopoKeyFramesToServer(TopoId tId, std::vector<KeyFrame *> pKFs);

        std::set<KeyFrame *> TransTopoKeyFramesFromServer( TopoId tId ) ;

        void TransKeyFramesToServerOneByOne( std::vector<KeyFrame *> pKFs);

        // trans the KeyFrames from ORBSlam server
        std::set<KeyFrame *> TransKeyFramesFromServerOneByOne( std::set<long unsigned int > pKFs );

        // trans the MapPoints to ORBSlam server
        std::vector<long unsigned int > TransMapPointsToServerOneByOne( std::set<MapPoint *> pMPs);

        // trans the MapPoints from ORBSlam server

        std::set<MapPoint * > TransMapPointsFromServerOneByOne( std::set<long unsigned int > pLMPs);

        KeyFrame* transOneKeyFrameFromServer( LightKeyFrame pLKF );

        bool TransOneKeyFrameToserver(KeyFrame *pKF);

        void getAllKeyFramePose();

        void getAllMapPointPose();

        void updateAllKeyFramePose();

        void updateAllMapPointPose();

    private:

        Cache * pCacher;

        ofstream f1;
        ofstream f2;
        ofstream f3;
        ofstream f4;

        std::mutex mMutexGetKeyFrame;
        std::mutex mMutexPushKeyFrame;
    };

} //namespace ORB_SLAM



#endif //PROJECT_DATADRIVER_H
