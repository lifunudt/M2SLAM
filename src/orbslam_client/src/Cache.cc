//
// Created by lifu on 17-1-31.
//

#include "System.h"
#include "Cache.h"
#include "Converter.h"
#include "DataDriver.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <time.h>

namespace ORB_SLAM2 {

    //construct function
    Cache::Cache(const int maxArea, const int Lmax, const int Lmin) {

        mpMap = nullptr;
        mpKeyFrameDatabase = nullptr;
        mpVocabulary = nullptr;
        lKFToKFmap.clear();
        lMPToMPmap.clear();
        mbFinished = false;
        mbStopRequested = false;
        mbNotStop = true;
        mbStopped = false;
        mbFinishRequested = false;
        kfStatus.clear();

        //init topomap
        {
            mTopoMap = new TopoMap(this, maxArea, Lmax, Lmin);

            mCurrentTopoId = mTopoMap->generateId(cv::Point3d(0, 0, 0));

            mTpInCache = mTopoMap->getTopoMapsNeedInCache(mCurrentTopoId);

            for (std::set<TopoId>::iterator mit = mTpInCache.begin();
                 mit != mTpInCache.end(); mit++) {

                TopoIdStatus[*mit] = UN_USE;

            }

        }

    }

    bool Cache::loadORBVocabulary(const string &strVocFile) {

        //Load ORB Vocabulary
        cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        if (!bVocLoad) {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }

        cout << "Vocabulary loaded!" << endl << endl;

        return bVocLoad;

    }

    void Cache::createKeyFrameDatabase() {
        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    }

    void Cache::createMap() {
        //Create the Map
        mpMap = new Map();

    }

    void Cache::AddKeyFrameToMap(KeyFrame *pKF) {

        // add KeyFrame to the LinghtKeyFrame to KeyFrame map
        unique_lock<mutex> lock(mMutexlKFToKFmap);
        mpMap->AddKeyFrame(pKF);
        lKFToKFmap[pKF->mnId] = pKF;
        kfStatus[pKF->mnId] = KF_IN_CACHE;
        tmpKFMap.erase(pKF->mnId);
        kfTimstamp[ pKF->mnId ] = pKF->mTimeStamp;

    }

    void Cache::AddKeyFrameToTopoMap(KeyFrame *pKF) {

        mTopoMap->addKeyFrame(pKF);

        this->mCurrentTopoId = pKF->mTopoId;

    }

    void Cache::EraseKeyFrameFromTopoMap(KeyFrame *pKF) {

        mTopoMap->eraseKeyFrame(pKF);

    }

    void Cache::EraseKeyFrameFromMap(KeyFrame *pKF) {

        // erase KeyFrame to the lightKeyFrame to KeyFrame
        {
            //unique_lock<mutex> lock2(mpMap->mMutexMapUpdate);
            unique_lock<mutex> lock(mMutexlKFToKFmap);
            lKFToKFmap.erase(pKF->mnId);
            kfStatus.erase(pKF->mnId);
        }

        mpMap->EraseKeyFrame(pKF);

    }

    void Cache::AddMapPointToMap(MapPoint *pMP) {

        mpMap->AddMapPoint(pMP);

        unique_lock<mutex> lock(mMutexMPToMPmap);
        lMPToMPmap[pMP->mnId] = pMP;
        mpStatus[pMP->mnId] = MP_IN_CACHE;

    }

    void Cache::AddMapPointToTopoMap(MapPoint *pMP, TopoId tid) {

        mTopoMap->addMapPoint(pMP, tid);

    }


    void Cache::EraseMapPointFromMap(MapPoint *pMP) {

        {
            unique_lock<mutex> lock(mMutexMPToMPmap);
            lMPToMPmap.erase(pMP->mnId);
            EraseMapPointFromTopoMap(pMP->mnId);
        }

        mpMap->EraseMapPoint(pMP);
        mpStatus.erase(pMP->mnId);

    }

    void Cache::EraseMapPointFromTopoMap(long unsigned int mpId) {

        mTopoMap->eraseMapPoint(mpId);

    }

    KeyFrame *Cache::getKeyFrameById(long unsigned int pId) {

        if (pId <= 0) return nullptr;

        KeyFrame *pKF = nullptr;

        if (tmpKFMap.find(pId) != tmpKFMap.end()) {
            pKF = tmpKFMap[pId];
        } else if (lKFToKFmap.find(pId) != lKFToKFmap.end())
            pKF = lKFToKFmap[pId];

        return pKF;
    }

    bool Cache::KeyFrameInCache(long unsigned int pID) {

        unique_lock<mutex> lock(mMutexKFStatus);

        if (kfStatus.find(pID) != kfStatus.end())
            return kfStatus[pID] == KF_IN_CACHE;

        return false;

    }

    MapPoint *Cache::getMapPointById(long unsigned int pId) {

        if (pId <= 0) return nullptr;

        MapPoint *pMP = nullptr;
        {
            //unique_lock<mutex> lock(mMutexMPToMPmap);
            if (lMPToMPmap.find(pId) != lMPToMPmap.end()) {
                unique_lock<mutex> lock(mMutexMPToMPmap);
                pMP = lMPToMPmap[pId];
            }

        }

        return pMP;

    }

    KeyFrame *Cache::getKeyFrameFromServer(long unsigned int pId) {

        cout << "getKeyFrameFromServer  " << pId << endl;

        KeyFrame *pKF = nullptr;

        DataDriver DB(this);

        LightKeyFrame tlkf(pId, this);
        {
            pKF = DB.transOneKeyFrameFromServer(tlkf);
        }

        if (pKF) {

            //unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            mpMap->AddKeyFrame(pKF);

            kfStatus[pKF->mnId] = KF_IN_CACHE;

            {
                unique_lock<mutex> lock(mMutexlKFToKFmap);
                // add KeyFrame to the LinghtKeyFrame to KeyFrame map
                lKFToKFmap[pKF->mnId] = pKF;
            }

        }

        return pKF;
    }

    bool Cache::checkMapPointLegal(MapPoint *tmp) {

        return (tmp->mnId <= MapPoint::nNextId && tmp->mnId >= 0);

    }

    std::vector<MapPoint *> Cache::GetAllMapPointsFromMap() {
        unique_lock<mutex> lock(mCorrectLoopMutex);
        return mpMap->GetAllMapPoints();
    }

    std::vector<LightMapPoint> Cache::GetAllLightMapPointsFromMap() {
        unique_lock<mutex> lock(mCorrectLoopMutex);

        return mpMap->GetAllLightMapPoints();
    }

    void Cache::SetReferenceMapPointsToMap(std::vector<LightMapPoint> pLocalMapPoints) {
        mpMap->SetReferenceMapPoints(pLocalMapPoints);
    }

    void Cache::SetmvpKeyFrameOrigins(KeyFrame *pKF) {
        mpMap->mvpKeyFrameOrigins.push_back((pKF)->mnId);
    }

    vector<long unsigned int> Cache::getmvpKeyFrameOrigins() {
        return mpMap->mvpKeyFrameOrigins;
    }


    const int Cache::getKeyFramesInMap() {
        return mpMap->KeyFramesInMap();
    }

    vector<KeyFrame *> Cache::getAllKeyFramesInMap() {
        return mpMap->GetAllKeyFrames();
    }

    long unsigned int Cache::GetMaxKFidInMap() {
        return mpMap->GetMaxKFid();
    }

    void Cache::clearMap() {

        mpMap->clear();
        {
            unique_lock<mutex> lock(mMutexlKFToKFmap);
            lKFToKFmap.clear();
        }
        lMPToMPmap.clear();
        tmpKFMap.clear();
        kfStatus.clear();
        mTpInCache.clear();
        mCurrentTopoId = 0;

    }

    void Cache::EraseKeyFrameFromDB(KeyFrame *pKF) {
        mpKeyFrameDatabase->erase(pKF);
    }

    vector<KeyFrame *> Cache::DetectRelocalizationCandidatesFromDB(Frame *F) {

        return mpKeyFrameDatabase->DetectRelocalizationCandidates(F);

    }

    vector<long unsigned int> Cache::DetectLoopCandidatesFromDB(KeyFrame *pKF, float minScore) {
        return mpKeyFrameDatabase->DetectLoopCandidatesInTopoMap(pKF, minScore);
    }

    void Cache::clearKeyframeDatabase() {

        mpKeyFrameDatabase->clear();
    }


    void Cache::addKeyFrametoDB(KeyFrame *pKF) {

        mpKeyFrameDatabase->add(pKF);

    }

    void Cache::SaveMap(const string &filename) {
        // save Map to files
        std::ofstream ofs("savetest.txt");
        boost::archive::text_oarchive oa(ofs);
        oa << mpMap;
        ofs.close();

    }

    void Cache::LoadMap(const string &filename) {
        // loadMap to files

        std::ifstream ifs("savetest.txt");
        boost::archive::text_iarchive ia(ifs);
        ia >> mpMap;
        ifs.close();

    }

    void Cache::getAllKeyFramePose() {

        DataDriver DB(this);
        DB.getAllKeyFramePose();

    }

    void Cache::getAllMapPointPose() {

        DataDriver DB(this);
        DB.getAllMapPointPose();

    }

    void Cache::updatePoseInCache() {

        for( std::map<long unsigned int, cv::Mat>::iterator mit = mTopoMap->mpKfPose.begin(); mit != mTopoMap->mpKfPose.end(); mit++) {
            KeyFrame * tKF = getKeyFrameById( (*mit).first );
            if( tKF ) {
                tKF->SetPose( (*mit).second );
            }
        }
        for( std::map<long unsigned int, cv::Mat>::iterator mit = mTopoMap->mpMpPose.begin(); mit != mTopoMap->mpMpPose.end(); mit++) {
            MapPoint * tMP = getMapPointById( (*mit).first );
            if( tMP ) {
                tMP->SetWorldPos( (*mit).second );
            }
        }


    }

    void Cache::updateAllPoseToServer(){

        updatePoseInCache();

        DataDriver DB(this);

        DB.updateAllKeyFramePose();

        DB.updateAllMapPointPose();

    }

    cv::Mat Cache::GetPoseInverse(long unsigned int pKF) {

        cv::Mat tmat = mTopoMap->mpKfPose[pKF];
        cv::Mat Twc = cv::Mat::eye(4, 4, tmat.type());
        return Twc;

    }

    std::vector< pair<long unsigned int , cv::Mat> > Cache::getKeyFramePoseInCache(){

        vector<KeyFrame * >  tKFs = this->mpMap->GetAllKeyFrames();

        vector<pair< long unsigned int, cv::Mat> > ans;
        for( int i = 0; i < tKFs.size(); i++ ) {

            ans.push_back( make_pair( tKFs[i]->mnId, tKFs[i]->GetPose() ) );

        }

        return ans;

    }

    void Cache::outputKeyframePose(){
        std::vector< pair<long unsigned int , cv::Mat> > kfpose = this->getKeyFramePoseInCache();

        for( std::map<long unsigned int, cv::Mat>::iterator mit = mTopoMap->mpKfPose.begin();
             mit != mTopoMap->mpKfPose.end(); mit++) {

            bool flag = true;

            for( int i = 0 ; i < kfpose.size(); i++ ) {
                if (kfpose[i].first == (*mit).first) {
                    flag = false;
                    break;
                }
            }
            if( flag )
                kfpose.push_back( make_pair( (*mit).first, (*mit).second ));

        }
        ofstream f;
        f.open("kfpose.txt");

        cout << "output kfpose " ;
        for( int i = 0 ; i < kfpose.size(); i++ ) {

            cout << kfpose[i].first << " ";
            cv::Mat Tcw;
            if( kfpose[i].second.rows <= 0 )
                continue;
            kfpose[i].second.copyTo( Tcw );
            cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
            cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
            cv::Mat Rwc = Rcw.t();
            cv::Mat Ow = -Rwc * tcw;
            cv::Mat R = Rcw.t();
            vector<float> q = Converter::toQuaternion(R);

            f << boost::lexical_cast<string>( kfTimstamp[ kfpose[i].first ] )<< " " << kfpose[i].first << " " << Ow.at<float>(0) << " " << Ow.at<float>(1) << " " << Ow.at<float>(2)
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }
        cout << endl;
        f.close();
    }

    /*  cache organize function   */

    void Cache::run() {

        while (1) {

            if (CheckTopoMapUnSatisfied()) {

                try {
                    unique_lock<mutex> lock(mMutexStop);
                    transTopoMapKeyFrames();
                } catch( ... ) {
                    cout << "error at cache\n";
                }

            } else if (isStopped()) {
                // Safe area to stop
                while (isStopped() && !CheckFinish()) {
                    usleep(3000);
                }
                if (CheckFinish())
                    break;
            }

            if (CheckFinish())
                break;

            malloc_trim(0);
            usleep(3000);

        }

        SetFinish();
    }

    //if the topo map in cache unsatisfy the currentTopoId need return true, else return false

    bool Cache::CheckTopoMapUnSatisfied() {

        if (mCurrentTopoId == 0) return false;

        std::set<TopoId> tpNeedInCache;

        tpNeedInCache = mTopoMap->getTopoMapsNeedInCache(mCurrentTopoId);

        bool flag = false;

        for (std::set<TopoId>::iterator mit = tpNeedInCache.begin(); mit != tpNeedInCache.end(); mit++) {

            if (mTpInCache.find((*mit)) == mTpInCache.end()) {

                flag = true;

            }
        }

        return flag;
    }


    void Cache::transTopoMapKeyFrames() {

        unique_lock<mutex> lock(mCorrectLoopMutex);
        std::set<TopoId> tpNeedInCache;

        tpNeedInCache = mTopoMap->getTopoMapsNeedInCache(mCurrentTopoId);

        std::set<TopoId> tpNeedOutCache;

        for (std::set<TopoId>::iterator mit = mTpInCache.begin(); mit != mTpInCache.end(); mit++) {

            if (tpNeedInCache.find((*mit)) == tpNeedInCache.end()) {

                if (TopoIdStatus[*mit] == UN_USE)

                    tpNeedOutCache.insert(*mit);

            } else {

                tpNeedInCache.erase((*mit));
            }

        }
        cout << "need out cache : " ;
        for (std::set<TopoId>::iterator mit = tpNeedOutCache.begin(); mit != tpNeedOutCache.end(); mit++) {
            mTpInCache.erase(*mit);
            cout << *mit << " " ;
        }
        cout << endl << "need in Cache : ";

        for (std::set<TopoId>::iterator mit = tpNeedInCache.begin(); mit != tpNeedInCache.end(); mit++) {

            mTpInCache.insert(*mit);
            cout << *mit <<" ";

        }

        cout << endl;

        for (std::set<TopoId>::iterator mit = tpNeedOutCache.begin(); mit != tpNeedOutCache.end(); mit++) {

            std::set<long unsigned int> tKFs = mTopoMap->getKFsbyTopoId((*mit));

            transKeyFrameToServer(tKFs);

            std::set<long unsigned int> tmps = mTopoMap->getMapPoints(*mit);

            std::set<MapPoint *> mps;

            mps.clear();

            for (std::set<long unsigned int>::iterator mpid = tmps.begin(); mpid != tmps.end(); mpid++)
                if (lMPToMPmap.find(*mpid) != lMPToMPmap.end())
                    mps.insert(lMPToMPmap[*mpid]);

            if (mps.size() > 0)
                transMapPointToServer(*mit, mps);

            TopoIdStatus[*mit] = IN_SERVER;
        }

        for (std::set<TopoId>::iterator mit = tpNeedInCache.begin(); mit != tpNeedInCache.end(); mit++) {

            if (TopoIdStatus.find(*mit) != TopoIdStatus.end() && TopoIdStatus[*mit] == IN_SERVER) {

                std::set<long unsigned int> tKFs = mTopoMap->getKFsbyTopoId((*mit));

                transKeyFrameFromServer(*mit, tKFs);

                transMapPointFromServer(*mit);

                TopoIdStatus[*mit] = UN_USE;

            }

        }

    }

    void Cache::transKeyFrameToServer(std::set<long unsigned int> pkfs) {

        DataDriver DB(this);

        vector<KeyFrame *> tpkfs;
        tpkfs.clear();

        long unsigned int tid = 0;
        {
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            for (std::set<long unsigned int>::iterator mit = pkfs.begin(); mit != pkfs.end(); mit++) {

                KeyFrame *tKF = getKeyFrameById(*mit);

                if (tKF) {

                    tpkfs.push_back(tKF);

                    tid = tKF->mTopoId;

                    mpMap->transKeyframeToBack(tKF);

                    {
                        unique_lock<mutex> lock(mMutexlKFToKFmap);
                        lKFToKFmap.erase(*mit);
                        kfStatus[*mit] = KF_IN_SERVER;
                    }

                }

            }
        }

        {
            DB.TransTopoKeyFramesToServer(tid, tpkfs);

        }
//        {
//
//            for (int i = 0; i < (int) tpkfs.size(); i++) {
//                if (tpkfs[i])
//                    delete tpkfs[i];
//            }
//        }


    }

    void Cache::keepTopoIdSetInCache(std::set<TopoId> tps) {

        for (std::set<TopoId>::iterator mit = tps.begin(); mit != tps.end(); mit++) {

            cout << *mit << endl;

            if (mTpInCache.find(*mit) == mTpInCache.end()) {

                std::set<long unsigned int> tKFs = mTopoMap->getKFsbyTopoId((*mit));

//                transKeyFrameFromServer(tKFs);

                transMapPointFromServer(*mit);

                mTpInCache.insert(*mit);

            }

            TopoIdStatus[*mit] = IN_USE;

        }

    }

    void Cache::setTopoIdSetUnUse(std::set<TopoId> tps) {

        for (std::set<TopoId>::iterator mit = tps.begin(); mit != tps.end(); mit++) {
            TopoIdStatus[*mit] = UN_USE;
        }
    }

    void Cache::transKeyFrameFromServer(long unsigned int tid, std::set<long unsigned int> pkfs) {

        DataDriver DB(this);

        std::set<KeyFrame *> kfs;

        if (pkfs.size() <= 0) return;

        kfs = DB.TransTopoKeyFramesFromServer(tid);
        {
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
            for (std::set<KeyFrame *>::iterator mit = kfs.begin(); mit != kfs.end(); mit++) {

                if (*mit) {

                    (*mit)->setCache(this);

                    AddKeyFrameToMap(*mit);

                }
            }
        }
    }


    void Cache::transMapPointToServer(TopoId tId, std::set<MapPoint *> vMP) {

        DataDriver DB(this);

        vector<long unsigned int> finishedTransMps = DB.TransTopoMapPointsToServer(tId, vMP);

        {
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            for (int mit = 0; mit < (int) finishedTransMps.size(); mit++) {

                long unsigned int tId = finishedTransMps[mit];

                MapPoint *tMP = getMapPointById(tId);

                if (tMP) {

                    std::set<TopoId> topoSet = tMP->mpTopoIds;

                    bool flag = false;

                    for (std::set<TopoId>::iterator ptId = topoSet.begin(); ptId != topoSet.end(); ptId++) {
                        if (mTpInCache.find(*ptId) != mTpInCache.end()) {
                            flag = true;
                            break;
                        }
                    }

                    if (!flag) {

                        mpMap->EraseMapPoint(tMP);
                        {
                            unique_lock<mutex> lock(mMutexMPToMPmap);
                            lMPToMPmap.erase(tId);
                        }

//                        delete tMP;

                    }
                }

            }
        }


    }

    void Cache::transMapPointFromServer(TopoId tId) {

        std::set<MapPoint *> vMPs;

        DataDriver DB(this);

        vMPs = DB.TransTopoMapPointsFromServer(tId);

        {
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            for (std::set<MapPoint *>::iterator mit = vMPs.begin(); mit != vMPs.end(); mit++) {

                long unsigned int tId = (*mit)->mnId;

                if (lMPToMPmap.find(tId) == lMPToMPmap.end()) {
                    mpMap->AddMapPoint(*mit);
                    {
                        unique_lock<mutex> lock(mMutexMPToMPmap);
                        lMPToMPmap[tId] = *mit;
                    }
                } else {
//                    delete (*mit);
                }
            }
        }

    }


    bool Cache::checkInCache(LightKeyFrame tLKF) {

        unique_lock<mutex> lock(mMutexKFStatus);

        if (kfStatus.find(tLKF.mnId) != kfStatus.end())
            return kfStatus[tLKF.mnId] == KF_IN_CACHE;

        return false;

    }

    void Cache::insertTmpKeyFrame(KeyFrame *pKF) {

        if (pKF) {

            unique_lock<mutex> lock(mMutexTmpKFMap);
            tmpKFMap[pKF->mnId] = pKF;
            kfStatus[pKF->mnId] = KF_IN_CACHE;

        }

    }

    void Cache::eraseTmpKeyFrame(long unsigned int pID) {

        unique_lock<mutex> lock(mMutexTmpKFMap);

        if (tmpKFMap.find(pID) != tmpKFMap.end()) {

            tmpKFMap.erase(pID);

        }
    }


    bool Cache::isStopped() {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Cache::CheckFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Cache::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        unique_lock<mutex> lock2(mMutexStop);
        mbStopped = true;
    }

    void Cache::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Cache::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    bool Cache::Stop() {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = true;
        cout << "Cache STOP" << endl;
        return true;
    }

    bool Cache::Start() {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
        cout << "Cache STOP" << endl;
        return true;
    }

}