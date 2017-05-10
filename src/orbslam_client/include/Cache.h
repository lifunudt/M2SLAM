//
// Created by lifu on 17-1-31.
//

#ifndef ORB_SLAM2_CACHE_H
#define ORB_SLAM2_CACHE_H

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

#include "TopoMap.h"

#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "Map.h"
#include "SerializeObject.h"


/*
 * Cache class is the data layer which organize all the persistent data, such as keyframe, mappoint, all kinds of map.
 * Cache class provide the all metheds which access the data layer such as basic read and store methods
 *
 */

namespace ORB_SLAM2 {

    class Map;

    // class ORBVocabulary;
    class KeyFrameDatabase;

    class KeyFrame;

    class TopoMap;

    typedef long unsigned int TopoId;

    class LoopKeyPoint{
    private:
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive &ar,  const unsigned int) {
            ar & ptx & pty & ptur & octaveSigm;
        };
    public:
        LoopKeyPoint(){};
        LoopKeyPoint(float x, float y, float ur, float octsigm ): ptx(x), pty(y), ptur(ur), octaveSigm(octsigm) {};
        float ptx;
        float pty;
        float ptur;
        float octaveSigm;
    };

    enum KF_status { KF_IN_CACHE, KF_IN_SERVER, KF_UNEXIST };
    enum TopoId_status { UN_USE , IN_USE, IN_SERVER };
    enum MP_status { MP_IN_CACHE, MP_IN_SERVER, MP_UNEXIST };

    class Cache {

    public:
        //construct function
        Cache( const int  maxArea, const int Lmax, const int Lmin );

        //load the ORBVocabulary from the file
        bool loadORBVocabulary(const string &strVocFile);

        void createKeyFrameDatabase();

        void createMap();

        void run();

        // operate about keyframes

        // add one keyFrame to map
        void AddKeyFrameToMap(KeyFrame *pKF);

        void AddKeyFrameToTopoMap( KeyFrame * pKF );

        //erase one particular keyFrame in map
        void EraseKeyFrameFromMap(KeyFrame *pKF);

        void EraseKeyFrameFromTopoMap( KeyFrame * pKF) ;

        //get the number of keyframes in map
        const int getKeyFramesInMap();

        //get all keyframes in the map
        vector<KeyFrame *> getAllKeyFramesInMap();

        long unsigned int GetMaxKFidInMap();

        KeyFrame *getKeyFrameById(long unsigned int pId);

        MapPoint *getMapPointById(long unsigned int pId);

        KeyFrame *getKeyFrameFromServer(long unsigned int pId);

        // operate about mappoint

        //add one MapPoint to map
        void AddMapPointToMap(MapPoint *pMP);

        void AddMapPointToTopoMap(MapPoint *pMP, TopoId tid );

        // erase the paticular mapPoint from the map
        void EraseMapPointFromMap(MapPoint *pMP);

        void EraseMapPointFromTopoMap(long unsigned int mpId);

        // get all MapPoints in the map
        std::vector<MapPoint *> GetAllMapPointsFromMap();

        std::vector<LightMapPoint> GetAllLightMapPointsFromMap();


            //set ReferenceMapPoint in the map
        void SetReferenceMapPointsToMap(std::vector<LightMapPoint> pLocalMapPoints);

        //push back keyFrame to mvpKeyFrameOrigin in Map
        void SetmvpKeyFrameOrigins(KeyFrame *pKF);

        //get mvpKeyFrameOrigins from map
        vector<long unsigned int> getmvpKeyFrameOrigins();

        //Clear map
        void clearMap();

        //operate about keyframeDatabase
        //erase one keyfrmae from keyFramedatabase
        void EraseKeyFrameFromDB(KeyFrame *pKF);

        //KeyFrameDatabase detectRelocalizationCandidates keyframes
        vector<KeyFrame *> DetectRelocalizationCandidatesFromDB(Frame *F);

        //KeyFrameDatabase detect the loopCandi keyframes
        vector<long unsigned int> DetectLoopCandidatesFromDB(KeyFrame *pKF, float minScore);

        //add one keyFrame to keyFrameDatabase
        void addKeyFrametoDB(KeyFrame *pKF);

        //clear KeyFrameDatabase
        void clearKeyframeDatabase();

        void updatePoseInCache();

        void getAllKeyFramePose();

        void getAllMapPointPose();

        void updateAllPoseToServer();

        //get and set functions
        //get ORBVocabulary point
        ORBVocabulary *getMpVocabulary() const {
            return mpVocabulary;
        }

        //get the keyFrameDatabase point
        KeyFrameDatabase *getMpKeyFrameDatabase() const {
            return mpKeyFrameDatabase;
        }

        //get the map point
        Map *getMpMap() const {
            return mpMap;
        }

        cv::Mat GetPoseInverse( long unsigned int pKF );

        bool checkMapPointLegal( MapPoint * tmp);

        void keepTopoIdSetInCache( std::set<TopoId> tps );

        void setTopoIdSetUnUse( std::set<TopoId> tps );

        void SaveMap(const string &filename);

        void LoadMap(const string &filename);

        void RequestFinish();

        bool isFinished();

        void insertTmpKeyFrame( KeyFrame* pKF);

        void eraseTmpKeyFrame( long unsigned int pID );

        bool KeyFrameInCache( long unsigned int pID);

        bool Stop();

        bool Start();

        bool isStopped();

        std::vector< pair<long unsigned int , cv::Mat> > getKeyFramePoseInCache();

        void outputKeyframePose();

    private:
        /*  cache organize function   */

        bool CheckTopoMapUnSatisfied();

        void transTopoMapKeyFrames();

        bool checkInCache( LightKeyFrame tLKF );


        bool CheckFinish();
        void SetFinish();

        //transfer the keyframes to server and
        //get the keyframe from server using ros service
        void transKeyFrameToServer( std::set<long unsigned int>  pkfs );

        void transKeyFrameFromServer( long unsigned int tid, std::set<long unsigned int> pkfs );

        void transMapPointToServer( TopoId tId, std::set<MapPoint*> vMP);

        void transMapPointFromServer( TopoId tId );

    public:

        TopoMap * mTopoMap;

        std::mutex mMutexCacheMap;

        cv::Mat mK;

        float mbf;

    private:

        // ORB vocabulary used for place recognition and feature matching.
        ORBVocabulary *mpVocabulary;

        // KeyFrame database for place recognition (relocalization and loop detection).
        KeyFrameDatabase *mpKeyFrameDatabase;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
        Map *mpMap;

        TopoId mCurrentTopoId;

        std::set<TopoId> mTpInCache;

        std::map<long unsigned int, double > kfTimstamp;

        std::mutex mMutexlKFToKFmap;
        std::map<long unsigned int, KeyFrame *> lKFToKFmap;

        std::mutex mMutexMPToMPmap;
        std::map<long unsigned int, MapPoint *> lMPToMPmap;

        //cache orgnize data

        bool mbFinished;

        std::mutex mMutexStop;
        bool mbStopRequested;
        bool mbNotStop;
        bool mbStopped;

        std::mutex mMutexFinish;
        bool mbFinishRequested;

        std::mutex mMutexTmpKFMap;
        std::map<long unsigned int, KeyFrame*> tmpKFMap;

        std::mutex mMutexKFStatus;

        std::map<long unsigned int, KF_status > kfStatus;

        std::map<long unsigned int, MP_status > mpStatus;

        std::map<TopoId, TopoId_status> TopoIdStatus;

        std::mutex mCorrectLoopMutex;


    };


} //namespace ORB_SLAM
#endif //ORB_SLAM2_CACHE_H

