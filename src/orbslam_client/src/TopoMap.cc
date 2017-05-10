//
// Created by lifu on 3/28/17.
//

#include "TopoMap.h"
#include <cmath>
using namespace std;

namespace ORB_SLAM2 {

    ToPoIdGenetator::ToPoIdGenetator(int MaxArea, int Lmin, int Lmax ): mMaxArea( MaxArea ), mLmin( Lmin ), mLmax( Lmax ) {

        int tmp = ( MaxArea / Lmax ) * 2;
        codebits = 0;
        while( tmp > 0 ) {
            tmp = tmp / 2;
            codebits ++ ;
        }

    }

    TopoId ToPoIdGenetator::generateId( cv::Point3d p3d) {

        TopoId ans = 0;

        long unsigned int x = (long unsigned int) (( mMaxArea + p3d.x )/ mLmax + 1 );
        long unsigned int y = (long unsigned int) (( mMaxArea + p3d.y )/ mLmax + 1 );
        long unsigned int z = (long unsigned int) (( mMaxArea + p3d.z )/ mLmax + 1 );

        ans =  this->generateId( x, y, z );

//        cout << "KeyFrame x : " << p3d.x << " y : " << p3d.y << " z : " << p3d.z << " the code is " << ans <<  endl;

        return ans;

    }

    TopoId ToPoIdGenetator::generateId( long unsigned int x, long unsigned int y, long unsigned int z ){

        long unsigned int id = ( x << codebits ) + z;

        return id;

    }

    // this is the algorithm of keep what topomap in cache
    std::set< TopoId > ToPoIdGenetator::generateOctIds( TopoId tId ) {

        long unsigned int x = ( tId >> codebits );
        long unsigned int y = 0;
        long unsigned int z = ( tId % ( 1 << codebits ) ) ;

        std::set<TopoId > OctIds;

        for( int i = -5; i <= 5; i++ ) {
            for( int j = -5; j <= 5; j++ ) {
                TopoId tmp = this->generateId( x + i, y , z + j );
                OctIds.insert( tmp );
            }
        }

        return OctIds;
    }

    TopoMap::TopoMap(Cache *pCache, int MaxArea, int Lmin, int Lmax) {

        this->mpCache = pCache;

        mTopoIdGen = new ToPoIdGenetator(MaxArea, Lmin, Lmax);

        KFgraph.clear();

        this->mpTopoKFs.clear();

        this->mpTopoMps.clear();

        DBowMap.clear();


    }

    TopoId TopoMap::generateId( cv::Point3d p3d ){

        return mTopoIdGen->generateId( p3d );

    }


    void TopoMap::addKeyFrame(KeyFrame *pkf) {

        cv::Mat cord = pkf->GetStereoCenter();

        // need to calculate the point of the pkf

        cv::Point3d p3d( cord.at<float>(0), cord.at<float>(1), cord.at<float>(2) );

        TopoId topoKFid = mTopoIdGen->generateId(p3d);

        TopoId oldtopoKFid = pkf->mTopoId;

//        cout << "TopoMap addKeyFrame " << topoKFid << " " << oldtopoKFid << endl;

        if (!(topoKFid == oldtopoKFid)) {

            if( mpTopoKFs.find( oldtopoKFid ) != mpTopoKFs.end() )
                mpTopoKFs[oldtopoKFid].erase(pkf->mnId);

            mpTopoKFs[topoKFid].insert(pkf->mnId);

            pkf->mTopoId = topoKFid;

        }
        KF2TopoId[ pkf->mnId ] = topoKFid;

    }

    void TopoMap::eraseKeyFrame( KeyFrame * pkf ){

        TopoId mtpId = pkf->mTopoId;

        mpTopoKFs[mtpId].erase(pkf->mnId);

        if( KF2TopoId.find( pkf->mnId) != KF2TopoId.end() )
            KF2TopoId.erase( pkf->mnId );

    }

    TopoId TopoMap::getKeyFrameTopoId( long unsigned int kf){

        if( KF2TopoId.find( kf ) != KF2TopoId.end() )
            return KF2TopoId[ kf ];
        else
            return 0;

    }


    std::set<long unsigned int> TopoMap::getKFsbyTopoId(TopoId tpId) {

        std::set<long unsigned int> kfs;

        kfs.clear();

        if (mpTopoKFs.find(tpId) != mpTopoKFs.end()) {

            kfs = mpTopoKFs[tpId];

        }

        return kfs;

    }

    void TopoMap::addMapPoint(MapPoint * mp, TopoId tid) {

        unique_lock<mutex> lock( mpTopoMpsMutex );
        mpTopoMps[ tid ].insert( mp->mnId );

    }

    void TopoMap::eraseMapPoint( long unsigned int mpid ){

        unique_lock<mutex> lock( mpTopoMpsMutex );
        for ( std::map< TopoId, std::set<long unsigned int > >::iterator mit = mpTopoMps.begin();
                mit != mpTopoMps.end(); mit ++ ) {
            if( (*mit).second.count( mpid ) > 0 )
                (*mit).second.erase( mpid );
        }

    }

    std::set<long unsigned int > TopoMap::getMapPoints(TopoId tpId) {

        std::set<long unsigned int > mps;

        mps.clear();

        if (mpTopoMps.find(tpId) != mpTopoMps.end()) {

            mps = mpTopoMps[tpId];

        }

        return mps;

    }

    //this is the strategy of the keep the TopoMap in the cache
    //this stategy associated with the code
    std::set<TopoId> TopoMap::getTopoMapsNeedInCache( TopoId tpId ){

        return mTopoIdGen->generateOctIds( tpId );

    }

    // add or update the edge from kf1 to kf2 on the weight in the graph

    void TopoMap::addKeyFrameObservations( long unsigned int kf1, long unsigned int kf2, const int weight ){

        std::map<long unsigned int, std::vector< pair <long unsigned int, int > > >::iterator kfiter = this->KFgraph.find( kf1 );

        if( kfiter == KFgraph.end() ) {

            std::vector< pair<long unsigned int, int> > edge;
            edge.clear();
            edge.push_back( pair<long unsigned int, int > (kf2, weight));
            KFgraph[ kf1 ] = edge;

        } else {

            bool flag = true;

            for( int mit = 0 ; mit < (int) kfiter->second.size(); mit ++  ) {

                if( kfiter->second[ mit ].first == kf2 ) {
                    kfiter->second[ mit ].second = weight;
                    flag = false;
                    break;
                }
            }
            if( flag ) {
               kfiter->second.push_back( pair<long unsigned int, int > (kf2, weight) );
            }
        }

    }

    void TopoMap::setKeyFrameObservation(long unsigned int kf, std::map<LightKeyFrame, int> kfcount ){

        std::vector< pair<long unsigned int, int> > kfObservs;

        for( std::map<LightKeyFrame, int>::iterator mit = kfcount.begin(); mit != kfcount.end(); mit ++ ) {
            kfObservs.push_back( pair<long unsigned int, int> ((*mit).first.mnId, (*mit).second ) );
        }

        KFgraph[ kf ] = kfObservs;

    }

    void TopoMap::eraseKeyFrameObservations( long unsigned int kf1, long unsigned int kf2){

        std::map<long unsigned int, std::vector< pair <long unsigned int, int > > >::iterator kfiter = this->KFgraph.find( kf1 );

        if( kfiter == KFgraph.end() ) {

            return ;

        } else {

            for(std::vector<pair <long unsigned int, int > >::iterator mit = kfiter->second.begin() ; mit != kfiter->second.end(); mit ++  ) {

                if( (*mit).first == kf2 ) {
                    (*kfiter).second.erase( mit );
                    break;
                }

            }
        }

    }

    void TopoMap::eraseKeyFrameFromGraph( long unsigned int kf){

        std::map<long unsigned int, std::vector< pair <long unsigned int, int > > >::iterator kfiter = this->KFgraph.find( kf );

        if( kfiter == KFgraph.end() ) {

            return ;

        } else {

            for(std::vector<pair <long unsigned int, int > >::iterator mit = kfiter->second.begin() ; mit != kfiter->second.end(); mit ++  ) {

                this->eraseKeyFrameObservations( (*mit).first, kf );

            }

            KFParent.erase( (*kfiter).first );

            KFgraph.erase( kfiter );

        }

    }

    std::vector<long unsigned int> TopoMap::GetVectorCovisibleKeyFrames( long unsigned int kf){

        std::vector<long unsigned int> tCovisibleKFs;

        std::map<long unsigned int, std::vector< pair <long unsigned int, int > > >::iterator kfiter = this->KFgraph.find( kf );

        for(std::vector<pair <long unsigned int, int > >::iterator mit = kfiter->second.begin() ; mit != kfiter->second.end(); mit ++  ) {

            tCovisibleKFs.push_back( (*mit).first );

        }

        return tCovisibleKFs;

    }

    std::set<long unsigned int> TopoMap::GetConnectedKeyFrames( long unsigned int kf ){

        std::set<long unsigned int> tConnectedKFs;

        std::map<long unsigned int, std::vector< pair <long unsigned int, int > > >::iterator kfiter = this->KFgraph.find( kf );

        for(std::vector<pair <long unsigned int, int > >::iterator mit = kfiter->second.begin() ; mit != kfiter->second.end(); mit ++  ) {

            tConnectedKFs.insert( (*mit).first );

        }

        return tConnectedKFs;

    }

    std::vector<long unsigned int> TopoMap::GetBestCovisibilityKeyFrames(long unsigned int kf, const int &N ){

        std::map<long unsigned int, std::vector< pair <long unsigned int, int > > >::iterator kfiter = this->KFgraph.find( kf );

        std::vector< pair<int, unsigned long > > orderedConnectKFs;

        std::vector<long unsigned int> bestCovisbleKFs;

        for(std::vector<pair <long unsigned int, int > >::iterator mit = kfiter->second.begin() ; mit != kfiter->second.end(); mit ++  ) {

            orderedConnectKFs.push_back( pair<int, unsigned long >( (*mit).second, (*mit).first) );

        }

        sort( orderedConnectKFs.begin(), orderedConnectKFs.end() );

        for( int i = 0 ; i < std::min( N, (int)orderedConnectKFs.size() ); i++ )
            bestCovisbleKFs.push_back( orderedConnectKFs[i].second );

        return bestCovisbleKFs;

    }

    std::vector<long unsigned int> TopoMap::GetCovisiblesByWeight(long unsigned int kf, const int &w){

        std::map<long unsigned int, std::vector< pair <long unsigned int, int > > >::iterator kfiter = this->KFgraph.find( kf );

        std::vector< pair<int, unsigned long > > orderedConnectKFs;

        std::vector<long unsigned int> bestCovisbleKFs;

        for(std::vector<pair <long unsigned int, int > >::iterator mit = kfiter->second.begin() ; mit != kfiter->second.end(); mit ++  ) {

            orderedConnectKFs.push_back( pair<int, unsigned long >( (*mit).second, (*mit).first) );

        }

        sort( orderedConnectKFs.begin(), orderedConnectKFs.end() );

        for( int i = 0 ; i < orderedConnectKFs.size() ; i++ )
            if( orderedConnectKFs[i].first >= w )
                bestCovisbleKFs.push_back( orderedConnectKFs[i].second );

        return bestCovisbleKFs;

    }

    int TopoMap::GetWeight(long unsigned int fromKf, long unsigned int toKf){

        std::map<long unsigned int, std::vector< pair <long unsigned int, int > > >::iterator kfiter = this->KFgraph.find( fromKf );
        int weight = 0;
        if( kfiter != KFgraph.end() ) {
            for( std::vector< pair <long unsigned int , int > >::iterator mit = kfiter->second.begin() ;
                 mit != kfiter->second.end(); mit ++  ) {
                if( (*mit).first == toKf ) {
                    weight = (*mit).second ;
                    break;
                }
            }
        }
        return weight;

    }

    void TopoMap::addKeyFrameBowVector( long unsigned int kf, DBoW2::BowVector kfvec ){
        DBowMap[ kf ] = kfvec;

    }

    bool TopoMap::searchKeyFrameBowVector( long unsigned int kf ){

        if( DBowMap.find( kf ) == DBowMap.end() ) {
            return false;
        }
        return true;
    }

    void TopoMap::eraseKeyFrameBowVector( long unsigned int kf ){

        if( DBowMap.find( kf) != DBowMap.end() ) {
            DBowMap.erase( kf );
        }

    }

    DBoW2::BowVector TopoMap::getKeyFrameBowVector( long unsigned int kf ){

        return DBowMap[ kf ];

    }

    void TopoMap::ChangeParent(long unsigned int childKf, long unsigned int parentKf){

        if( KFParent.find( childKf ) != KFParent.end() ) {
            long unsigned int oldParent = KFParent[ childKf ];
            mpKFchilds[ oldParent ].erase( childKf);
        }
        KFParent[ childKf ] = parentKf;
        if( mpKFchilds.find( parentKf ) == mpKFchilds.end() )
        {
            std::set<long unsigned  int > tset;
            mpKFchilds[ parentKf ] = tset;
        }
        mpKFchilds[ parentKf ].insert( childKf );


    }

    long unsigned int TopoMap::GetParent( long unsigned int pKf){

        if( KFParent.find( pKf ) != KFParent.end() )
            return KFParent[ pKf ];
        return 1e8;

    }

    std::set<long unsigned int> TopoMap::GetChilds( long unsigned int pKF ){

        set<long unsigned int> tset;
        if( mpKFchilds.find( pKF ) != mpKFchilds.end() )
            tset = mpKFchilds[ pKF ];
        return tset;

    }


    void TopoMap::AddLoopEdge(long unsigned int fromKf, long unsigned int toKf ){

        std::map<long unsigned int, set<long unsigned int> >::iterator kfiter = KFLoopEdges.find( fromKf );
        if( kfiter == KFLoopEdges.end() ) {
            set<long unsigned int> tloop;
            tloop.insert( toKf );
            KFLoopEdges[ fromKf ] = tloop ;
        } else {
            (*kfiter).second.insert( toKf );
        }
    }

    std::set<long unsigned int> TopoMap::GetLoopEdges( long unsigned int pKf){

        set<long unsigned int> tloop;
        tloop.clear();

        std::map<long unsigned int, set<long unsigned int> >::iterator kfiter = KFLoopEdges.find( pKf );

        if( kfiter != KFLoopEdges.end() ) {
             return (*kfiter).second ;
        }
        return tloop;
    }

    void TopoMap::updateAllPose(){

//        for( std::map<long unsigned int, cv::Mat>::iterator mit = mpKfPose.begin();
//                mit != mpKfPose.end(); mit ++ ) {
//            KeyFrame * tKF = mpCache->getKeyFrameById( (*mit).first );
//            if( tKF ) {
//                tKF->SetPose( (*mit).second );
//            }
//        }
//
//        for( std::map<long unsigned int, cv::Mat>::iterator mit = mpMpPose.begin();
//                mit != mpMpPose.end(); mit ++ ) {
//            MapPoint * tMP = mpCache->getMapPointById( (*mit).first );
//            if( tMP ) {
//                tMP->SetWorldPos( (*mit).second );
//            }
//        }

        mpCache->updateAllPoseToServer();

    }

}