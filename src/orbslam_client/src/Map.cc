/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2 {

    Map::Map() : mnMaxKFid(0) {

        new thread(&ORB_SLAM2::Map::anlyzMapsize, this);

    }

    void Map::AddKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexKFs);
        mspKeyFrames.insert(pKF);
        if (pKF->mnId > mnMaxKFid)
            mnMaxKFid = pKF->mnId;
    }

    void Map::AddMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMPs);
        mspMapPoints.insert(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP) {
        unique_lock<mutex> lock(mMutexMPs);
        mspMapPoints.erase(pMP);

    }

    void Map::EraseKeyFrame(KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexKFs);
        //cout << "erase keyframe :" << pKF->mnFrameId << endl;
        pKF->dropMapPointMatches();
        mspKeyFrames.erase(pKF);

    }

    void Map::transKeyframeToBack(KeyFrame *pKF ) {
        unique_lock<mutex> lock(mMutexKFs);
        mspKeyFrames.erase(pKF);
    }


    void Map::SetReferenceMapPoints(const vector<LightMapPoint> vpMPs) {
        unique_lock<mutex> lock(mMutexRFMPs);
        mvpReferenceMapPoints.clear();
        for (int i = 0; i < (int) vpMPs.size(); i++) {
            MapPoint * tmp = vpMPs[i].getMapPoint();
            if( tmp )
                mvpReferenceMapPoints[tmp->mnId ] = tmp->GetWorldPos();
        }
    }

    vector<KeyFrame *> Map::GetAllKeyFrames() {
        unique_lock<mutex> lock(mMutexKFs);
        return vector<KeyFrame *>(mspKeyFrames.begin(), mspKeyFrames.end());
    }

    vector<MapPoint *> Map::GetAllMapPoints() {
        unique_lock<mutex> lock(mMutexMPs);
        return vector<MapPoint *>(mspMapPoints.begin(), mspMapPoints.end());
    }

    vector<LightMapPoint> Map::GetAllLightMapPoints() {
        std::vector<LightMapPoint> allMps;
        for (std::set<MapPoint *>::iterator mit = mspMapPoints.begin(); mit != mspMapPoints.end(); mit++) {
            allMps.push_back( LightMapPoint(*mit));
        }
        return allMps;
    }

    std::map<long unsigned int, cv::Mat > Map::GetAllMapPointsPose(){
        unique_lock<mutex> lock(mMutexMPs);

        std::map<long unsigned int, cv::Mat > mpsPose;
        for( std::set<MapPoint*>::iterator mit = mspMapPoints.begin(); mit != mspMapPoints.end(); mit++ ) {
            if( !(*mit)->isBad() )
                mpsPose[ (*mit)->mnId ] = (*mit)->GetWorldPos();
        }
        return mpsPose;

    };

    long unsigned int Map::MapPointsInMap() {
        unique_lock<mutex> lock(mMutexMPs);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap() {
        unique_lock<mutex> lock(mMutexKFs);
        return mspKeyFrames.size();
    }

    std::map<long unsigned int, cv::Mat > Map::GetReferenceMapPoints() {
        unique_lock<mutex> lock(mMutexRFMPs);

        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid() {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::clear() {
        //TODO delete
//        for (set<MapPoint *>::iterator sit = mspMapPoints.begin(), send = mspMapPoints.end(); sit != send; sit++)
//            delete *sit;
//
//        for (set<KeyFrame *>::iterator sit = mspKeyFrames.begin(), send = mspKeyFrames.end(); sit != send; sit++)
//            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    void Map::anlyzMapsize(){

        ofstream f;

        f.open("mymapsize.txt");
        int t = 0;

        while( 1 ) {

            f << t <<" " << mspKeyFrames.size() << " " << mspMapPoints.size() << endl;
            usleep(1000000);

            t = t + 1;

        }

    }

} //namespace ORB_SLAM
