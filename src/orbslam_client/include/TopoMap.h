//
// Created by lifu on 3/27/17.
//

#ifndef PROJECT_TOPOMAP_H
#define PROJECT_TOPOMAP_H

#include <string>
#include <thread>

#include "Cache.h"
#include "KeyFrame.h"
#include "MapPoint.h"


using namespace std;


namespace ORB_SLAM2{

    class Cache;

    class KeyFrame;

    class MapPoint;

    class LoopKeyPoint;

    typedef long unsigned int TopoId;

    class ToPoIdGenetator{

    public:

        ToPoIdGenetator() {}
        ToPoIdGenetator(int MaxArea, int Lmin, int Lmax );
        TopoId generateId( cv::Point3d p3d);
        TopoId generateId( long unsigned int x, long unsigned int y, long unsigned int z );
        std::set< TopoId > generateOctIds( TopoId tId );

    private:
        int mMaxArea;
        int mLmin;
        int mLmax;
        int codebits;

    };


    class TopoMap{

    public:

        TopoMap() {}

        TopoMap( Cache * pCache, int MaxArea, int Lmin, int Lmax);

        TopoId generateId( cv::Point3d p3d);

        // add and erase keyframe from the one topomap area
        void addKeyFrame(KeyFrame *pkf);

        void eraseKeyFrame( KeyFrame * pkf );

        std::set<long unsigned int > getKFsbyTopoId( TopoId tpId );

        TopoId getKeyFrameTopoId( long unsigned int kf);

        // add and erase mappoint from the one topomap area
        void addMapPoint( MapPoint * mp, TopoId tid );

        void eraseMapPoint( long unsigned int mpid );

        std::set<long unsigned int > getMapPoints( TopoId tpId );

        // the strategy that keep the local topomap in the cache
        std::set<TopoId> getTopoMapsNeedInCache( TopoId tpId);

        // functions about the keyFrame observation graph
        void addKeyFrameObservations( long unsigned int kf1, long unsigned int kf2, const int weight );

        void eraseKeyFrameObservations( long unsigned int kf1, long unsigned int kf2);

        void eraseKeyFrameFromGraph( long unsigned int kf);

        void setKeyFrameObservation(long unsigned int kf, std::map<LightKeyFrame, int> kfcount );

        std::vector<long unsigned int> GetVectorCovisibleKeyFrames( long unsigned int kf);

        std::set<long unsigned int> GetConnectedKeyFrames( long unsigned int kf );

        std::vector<long unsigned int> GetBestCovisibilityKeyFrames( long unsigned int kf, const int &N );

        std::vector<long unsigned int> GetCovisiblesByWeight(long unsigned int kf ,const int &w);

        int GetWeight(long unsigned int fromKf, long unsigned int toKf);

        // functions about the keyFrame DBow vector map
        void addKeyFrameBowVector( long unsigned int kf, DBoW2::BowVector kfvec );

        bool searchKeyFrameBowVector( long unsigned int kf );

        void eraseKeyFrameBowVector( long unsigned int kf );

        DBoW2::BowVector getKeyFrameBowVector( long unsigned int kf );

        void ChangeParent(long unsigned int childKf, long unsigned int parentKf);

        long unsigned int GetParent( long unsigned int pKf);

        void AddLoopEdge(long unsigned int fromKf, long unsigned int toKf );

        std::set<long unsigned int> GetLoopEdges( long unsigned int pKf);

        std::set<long unsigned int> GetChilds( long unsigned int pKF );

        void updateAllPose();

    public:
        std::map<long unsigned int, cv::Mat > mpKfPose;

        std::map<long unsigned int, cv::Mat > mpMpPose;

        std::map<long unsigned int, std::vector< pair < long unsigned int, LoopKeyPoint > > > mpMpObservations;

        std::map<long unsigned int, cv::Mat > mpKfTcwGBA;

        std::map<long unsigned int, long unsigned int > mpKfBAGlobalForKF;

        std::map<long unsigned int, long unsigned int > mpMpBAGlobalForKF;

        std::map<long unsigned int, cv::Mat > mpKfTcwBefGBA;

        std::map<long unsigned int, long unsigned int > mpRefKf;

        std::map<long unsigned int, std::set<long unsigned int> > mpKFchilds;

        // stores the topoid and the keyFrames in one topo area in the topomap division
        std::map< TopoId, std::set<long unsigned int > > mpTopoKFs;

        // need to store the Mps and the associate KFs Mps need to treat it particular
        std::map< TopoId, std::set< long unsigned int > > mpTopoMps;


    private:
        Cache * mpCache;

        // this graph stores the keyFrame maps outside
        std::map< long unsigned int , std::vector< pair<long unsigned int, int> > > KFgraph;

        // this map stores the keyframe DBow vector globle or local
        std::map<long unsigned int, DBoW2::BowVector> DBowMap;

        std::map<long unsigned int , TopoId > KF2TopoId;

        std::map<long unsigned int , long unsigned int> KFParent;

        std::map<long unsigned int, set<long unsigned int > > KFLoopEdges;

        ToPoIdGenetator* mTopoIdGen;

        std::mutex mpTopoMpsMutex;
        std::mutex mKFgraphMutex;
        std::mutex mpTopoKFsMutex;

    };

}


#endif //PROJECT_TOPOMAP_H
