//
// Created by lifu on 2/15/17.
//

#include "DataDriver.h"
#include "sstream"
#include "ros/ros.h"
#include "orbslam_server/orbslam_save.h"
#include "time.h"
#include "orbslam_server/orbslam_get.h"
#include "orbslam_server/orbslam_muilt_get.h"
#include "orbslam_server/orbslam_muilt_save.h"
#include "orbslam_server/orbslam_pose_save.h"
#include "orbslam_server/orbslam_pose_get.h"
#include <map>
#include <malloc.h>

using namespace std;

namespace ORB_SLAM2 {


    bool DataDriver::TransOneKeyFrameToserver(KeyFrame *pKF) {

        time_t start_t, end_t;
        start_t = clock();

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_save>("saveOneKeyFrame");
        orbslam_server::orbslam_save srv;

        std::ostringstream os;
        boost::archive::text_oarchive oa(os);
        oa << pKF;
        std::string ss = os.str();

        srv.request.ID = pKF->mnId;

        srv.request.DATA = ss;

        if (client.call(srv)) {

            end_t = clock();

            cout << "TransOneKeyFrameToserver ID: " << pKF->mnId << " use time : " <<
            (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

        }
        else {
            ROS_ERROR("Failed to call service save");
            return false;
        }

        return true;
    }


    KeyFrame *DataDriver::transOneKeyFrameFromServer(LightKeyFrame pLKF) {

        time_t start_t, end_t;
        start_t = clock();
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_get>("getOneKeyFrame");
        orbslam_server::orbslam_get srv;

        KeyFrame *tKF = new KeyFrame();

        srv.request.ID = pLKF.mnId;

        if (client.call(srv)) {

            std::string ss = srv.response.DATA;
            if (ss == "") {
                return nullptr;
            } else {

                std::stringstream is(ss);

                boost::archive::text_iarchive ia(is);

                ia >> tKF;

                tKF->setCache(pLKF.mpCache);

            }

        }
        else {
            ROS_ERROR("Failed to call service get");
        }

        end_t = clock();

        cout << "get keyFrame ID: " << tKF->mnId << " use time : " <<
        (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

        return tKF;
    }

    void DataDriver::TransKeyFramesToServer(std::vector<KeyFrame *> pKFs) {

        if (pKFs.size() <= 0) return;

        std::vector<pair<unsigned long int, std::string> > tpkfs;

        tpkfs.clear();

        cout << "Trans kfs size " << pKFs.size() << endl;

        for (int mit = 0; mit < (int) pKFs.size(); mit++) {

            unsigned long int tId = pKFs[mit]->mnId;

            std::ostringstream tos;
            boost::archive::text_oarchive toa(tos);

            toa << pKFs[mit];

            std::string tss = tos.str();

            //cout << tss << endl;

            tpkfs.push_back(pair<unsigned long int, std::string>(tId, tss));

        }

        time_t start_t, end_t;
        start_t = clock();

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_muilt_save>("saveKeyFrames");
        orbslam_server::orbslam_muilt_save srv;

        std::ostringstream os;
        boost::archive::text_oarchive oa(os);
        oa << tpkfs;
        std::string ss = os.str();

        srv.request.DATA = ss;

        if (client.call(srv)) {

            end_t = clock();

            cout << "TransOneKeyFramesToserver use time : " <<
            (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

        }
        else {
            cout << "Failed to call service save KeyFrames" << endl;
        }

        return;

    }

    std::set<KeyFrame *> DataDriver::TransKeyFramesFromServer(std::set<long unsigned int> pKFs) {


        std::vector<long unsigned int> vpkfs;

        std::set<KeyFrame *> kfs_ans;

        if (pKFs.size() <= 0) return kfs_ans;

        std::copy(pKFs.begin(), pKFs.end(), std::back_inserter(vpkfs));

        time_t start_t, end_t;
        start_t = clock();

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_muilt_get>("getKeyFrames");
        orbslam_server::orbslam_muilt_get srv;

        std::string ids;

        std::stringstream os(ids);

        boost::archive::text_oarchive oa(os);

        oa << vpkfs;

        srv.request.IDS = ids;

        if (client.call(srv)) {

            std::string totleData = srv.response.DATA;

            std::vector<pair<long unsigned int, std::string> > vkfs;

            std::stringstream tis(totleData);

            boost::archive::text_iarchive tia(tis);

            tia >> vkfs;

            for (int mit = 0; mit < (int) vkfs.size(); mit++) {

                string sss = vkfs[mit].second;
                std::stringstream sis(sss);
                boost::archive::text_iarchive sia(sis);

                KeyFrame *tKF = new KeyFrame();

                sia >> tKF;

                tKF->setCache( pCacher );

                kfs_ans.insert(tKF);

            }

        }

        end_t = clock();

        cout << "TransOneKeyFramesFromServer use time : " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC <<
        endl;

        return kfs_ans;

    }

    // trans the MapPoints to ORBSlam server
    std::vector<long unsigned int>  DataDriver::TransMapPointsToServer(std::set<MapPoint *> pMPs) {

        std::vector<long unsigned int> mpTrulyTrans;
        mpTrulyTrans.clear();

        if (pMPs.size() > 0) {

            std::vector<pair<unsigned long int, std::string> > tpmps;

            tpmps.clear();

            for (std::set<MapPoint *>::iterator mit = pMPs.begin(); mit != pMPs.end(); mit++) {

                if (*mit) {

                    unsigned long int tId = (*mit)->mnId;

                    std::ostringstream tos;
                    boost::archive::text_oarchive toa(tos);

                    toa << (*mit);

                    std::string tss = tos.str();

                    tpmps.push_back(pair<unsigned long int, std::string>(tId, tss));

                    mpTrulyTrans.push_back(tId);
                }

            }

            time_t start_t, end_t;
            start_t = clock();

            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_muilt_save>("saveMapPoints");
            orbslam_server::orbslam_muilt_save srv;

            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << tpmps;
            std::string ss = os.str();

            srv.request.DATA = ss;

            if (client.call(srv)) {

                end_t = clock();

                cout << "TransOneMapPointsToserver use time : " <<
                (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

            }
            else {
                cout << "Failed to call service save KeyFrames" << endl;
            }
        }

        return mpTrulyTrans;

    }

    // trans the MapPoints from ORBSlam server

    std::set<MapPoint *> DataDriver::TransMapPointsFromServer(std::set<long unsigned int> pLMPs) {


        std::vector<long unsigned int> vpmps;

        std::set<MapPoint *> mps_ans;

        if (pLMPs.size() <= 0) return mps_ans;

        std::copy(pLMPs.begin(), pLMPs.end(), std::back_inserter(vpmps));

        time_t start_t, end_t;
        start_t = clock();

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_muilt_get>("getMapPoints");
        orbslam_server::orbslam_muilt_get srv;

        std::string ids;

        std::stringstream os(ids);

        boost::archive::text_oarchive oa(os);

        oa << vpmps;

        srv.request.IDS = ids;

        if (client.call(srv)) {

            std::string totleData = srv.response.DATA;

            std::vector<pair<long unsigned int, std::string> > tmps;

            std::stringstream tis(totleData);

            boost::archive::text_iarchive tia(tis);

            tia >> tmps;

            for (int mit = 0; mit < (int)tmps.size(); mit++) {

                string sss = tmps[mit].second;
                std::stringstream sis(sss);
                boost::archive::text_iarchive sia(sis);

                MapPoint *tMP = new MapPoint();

                sia >> tMP;

                tMP->setCache( pCacher );

                mps_ans.insert(tMP);

            }

        }

        end_t = clock();

        cout << "TransOneKeyFramesFromServer use time : " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC <<
        endl;

        return mps_ans;

    }

    std::vector<long unsigned int>  DataDriver::TransTopoMapPointsToServer(TopoId tId, std::set<MapPoint *> pMPs) {

        time_t start_t, end_t;
        start_t = clock();
        f1.open( "STTmps.txt", ios::app );

        f1 << start_t / CLOCKS_PER_SEC << " ";
        std::vector<long unsigned int> mpTrulyTrans;
        mpTrulyTrans.clear();

        if (pMPs.size() > 0) {

            std::vector<pair<unsigned long int, std::string> > tpmps;
            std::map<unsigned long int, pair<cv::Mat,  std::vector< pair < long unsigned int, LoopKeyPoint > > > > tpposes;
            tpmps.clear();
            tpposes.clear();

            for (std::set<MapPoint *>::iterator mit = pMPs.begin(); mit != pMPs.end(); mit++) {

                if (*mit) {

                    unsigned long int ttId = (*mit)->mnId;

                    std::ostringstream tos;
                    boost::archive::text_oarchive toa(tos);

                    toa << (*mit);

                    std::string tss = tos.str();

                    tpmps.push_back(pair<unsigned long int, std::string>(ttId, tss));

                    tpposes[ttId] = make_pair((*mit)->GetWorldPos(), (*mit)->getObeservationIds() ) ;

                    mpTrulyTrans.push_back(ttId);
                    tos.clear();
                }

            }

            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_save>("saveTopoMapPoint");
            orbslam_server::orbslam_save srv;

            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << tpmps;

            srv.request.DATA = os.str();;

            f1 << srv.request.DATA.size() << " ";
            std::ostringstream ps;
            boost::archive::text_oarchive pa(ps);
            pa << tpposes;

            srv.request.POSE = ps.str();

            srv.request.ID = tId;

            if (client.call(srv)) {

                cout <<"tid " << tId <<  " Data size " << srv.request.DATA.size() << " POSE size " << srv.request.POSE.size() << endl;


            }
            else {
                cout << "Failed to call service save KeyFrames" << endl;
            }
            tpmps.clear();
            tpposes.clear();
            srv.request.POSE.clear();
            srv.request.DATA.clear();
            os.clear();
            ps.clear();

        }

        end_t = clock();

        cout << "Trans TopoMapPoints to server size " << mpTrulyTrans.size() << " use time : " <<
        (double) (end_t - start_t) / (double) CLOCKS_PER_SEC  << endl;

        f1 << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC  << endl;

        f1.close();

        return mpTrulyTrans;

    }

    std::set<MapPoint *> DataDriver::TransTopoMapPointsFromServer( TopoId tId ) {


        std::set<MapPoint *> mps_ans;

        time_t start_t, end_t;
        start_t = clock();

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_get>("getTopoMapPoint");
        orbslam_server::orbslam_get srv;

        srv.request.ID = tId;

        if (client.call(srv)) {

            if( srv.response.DATA.size() <= 0 )
                return mps_ans;

            f2.open( "TTSmps.txt", ios::app );

            f2 << start_t / CLOCKS_PER_SEC << " " << srv.response.DATA.size() + srv.response.POSE.size() << " ";

            end_t = clock();

            f2 << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;
            f2.close();

            std::vector<pair<long unsigned int, std::string> > tmps;

            std::map<unsigned long int, pair<cv::Mat, std::vector< pair < long unsigned int, LoopKeyPoint > > > > tpposes;

            std::stringstream tis(srv.response.DATA);

            boost::archive::text_iarchive tia(tis);


            tia >> tmps;

            std::stringstream tps( srv.response.POSE );
            boost::archive::text_iarchive tpa(tps);

            tpa >> tpposes;


            for (int mit = 0; mit < (int)tmps.size(); mit++) {
                try {
                    std::stringstream sis(tmps[mit].second);

                    boost::archive::text_iarchive sia(sis);

                    MapPoint *tMP = new MapPoint();

                    sia >> tMP;

                    if (tpposes.find(tMP->mnId) != tpposes.end())
                        tMP->SetWorldPos(tpposes[tMP->mnId].first);

                    tMP->setCache(pCacher);

                    if (tMP->mnId <= MapPoint::nNextId && tMP->mnId >= 0)
                        mps_ans.insert(tMP);

                    sis.clear();
                } catch(...) {
                    ROS_INFO("error in the trans point from server");
                }

            }
            tmps.clear();
            tpposes.clear();
            tis.clear();
            tps.clear();

        }
        srv.response.DATA.clear();
        srv.response.POSE.clear();

        end_t = clock();

        cout << "Trans MP size " << mps_ans.size()  << " use time : " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

        return mps_ans;

    }

    std::vector<long unsigned int>  DataDriver::TransTopoKeyFramesToServer(TopoId tId, std::vector<KeyFrame *> pKFs) {

        time_t start_t, end_t;
        start_t = clock();

        std::vector<long unsigned int> mpTrulyTrans;

        mpTrulyTrans.clear();

        if (pKFs.size() > 0) {

            std::vector<pair<unsigned long int, std::string> > tpkfs;
            std::map<unsigned long int, cv::Mat > tpposes;
            tpkfs.clear();
            tpposes.clear();

            for (std::vector<KeyFrame *>::iterator mit = pKFs.begin(); mit != pKFs.end(); mit++) {

                if (*mit) {

                    unsigned long int ttId = (*mit)->mnId;

                    std::ostringstream tos;
                    boost::archive::text_oarchive toa(tos);

                    toa << (*mit);

                    std::string tss = tos.str();

                    tpkfs.push_back(pair<unsigned long int, std::string>(ttId, tss));

                    tpposes[ttId] = (*mit)->GetPose() ;

                    mpTrulyTrans.push_back(ttId);
                    tos.clear();
                }

            }

            ros::NodeHandle n;
            ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_save>("saveTopoKeyFrame");
            orbslam_server::orbslam_save srv;

            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << tpkfs;

            srv.request.DATA = os.str();;

            std::ostringstream ps;
            boost::archive::text_oarchive pa(ps);
            pa << tpposes;

            srv.request.POSE = ps.str();

            srv.request.ID = tId;

            if (client.call(srv)) {
                ;
            }
            else {
                cout << "Failed to call service save KeyFrames" << endl;
            }

            end_t = clock();

            cout << "Trans Topokeyframes to server size " << mpTrulyTrans.size() << " use time : " <<
            (double) (end_t - start_t) / (double) CLOCKS_PER_SEC  << endl;

            tpkfs.clear();
            tpposes.clear();
            os.clear();
            ps.clear();
            srv.request.POSE.clear();

        }

        return mpTrulyTrans;

    }

    std::set<KeyFrame *> DataDriver::TransTopoKeyFramesFromServer( TopoId tId ) {


        std::set<KeyFrame *> kfs_ans;

        time_t start_t, end_t;
        start_t = clock();

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_get>("getTopoKeyFrame");
        orbslam_server::orbslam_get srv;

        srv.request.ID = tId;

        if (client.call(srv)) {

            if( srv.response.DATA.size() <= 0 )
                return kfs_ans;

            end_t = clock();

            std::vector<pair<long unsigned int, std::string> > tkfs;

            std::map<unsigned long int, cv::Mat > tpposes;

            std::stringstream tis(srv.response.DATA);

            boost::archive::text_iarchive tia(tis);

            tia >> tkfs;

            std::stringstream tps( srv.response.POSE );
            boost::archive::text_iarchive tpa(tps);

            tpa >> tpposes;

            for (int mit = 0; mit < (int)tkfs.size(); mit++) {
                try {
                    std::stringstream sis(tkfs[mit].second);

                    boost::archive::text_iarchive sia(sis);

                    KeyFrame *tKF = new KeyFrame();

                    sia >> tKF;

                    if (tpposes.find(tKF->mnId) != tpposes.end())
                        tKF->SetPose(tpposes[tKF->mnId]);

                    tKF->setCache(pCacher);

                    if (tKF->mnId <= KeyFrame::nNextId && tKF->mnId >= 0)
                        kfs_ans.insert(tKF);

                    sis.clear();

                } catch(...) {
                    ROS_INFO("error in the trans keyframe from server");
                }

            }

            tkfs.clear();
            tpposes.clear();
            tis.clear();
            tps.clear();
        }

        srv.response.POSE.clear();
        srv.response.DATA.clear();

        end_t = clock();

        cout << "Trans TopoKF size " << kfs_ans.size()  << " use time : " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;
        return kfs_ans;

    }

    void DataDriver::TransKeyFramesToServerOneByOne(std::vector<KeyFrame *> pKFs) {

        if (pKFs.size() <= 0) return;

        time_t start_t, end_t;
        start_t = clock();

        f3.open( "STTkfs.txt", ios::app );
        f3 << start_t / CLOCKS_PER_SEC << " ";
        int size = 0;

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_save>("saveOneKeyFrame");
        orbslam_server::orbslam_save srv;

        for (int mit = 0; mit < (int) pKFs.size(); mit++) {

            std::ostringstream os;
            boost::archive::text_oarchive oa(os);
            oa << pKFs[mit];

            std::string ss = os.str();

            std::ostringstream ps;
            boost::archive::text_oarchive pa(ps);

            cv::Mat pose_mat = pKFs[mit]->GetPose();
            pa << pose_mat;

            std::string pose = ps.str();

            srv.request.ID = pKFs[mit]->mnId;

            srv.request.DATA = ss;
            size += ss.size();
            srv.request.POSE = pose;
            size += pose.size();
            if (client.call(srv)) { ;
            }
            else {
                ROS_ERROR("Failed to call service save");
            }

            os.clear();
            ps.clear();

        }

        end_t = clock();

        cout << "Trans to server KF size "<< pKFs.size() <<" use time " <<
        (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

        f3 << size << " " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;
        f3.close();
        return;

    }

    std::set<KeyFrame *> DataDriver::TransKeyFramesFromServerOneByOne(std::set<long unsigned int> pKFs) {

        time_t start_t, end_t;
        start_t = clock();
        f4.open( "TTSkfs.txt", ios::app );

        f4 << start_t / CLOCKS_PER_SEC << " ";
        std::set<KeyFrame *> ans_kfs;
        ans_kfs.clear();

        int size = 0;

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_get>("getOneKeyFrame");
        orbslam_server::orbslam_get srv;

        for (std::set<long unsigned int>::iterator mit = pKFs.begin(); mit != pKFs.end(); mit++) {

            KeyFrame *tKF = new KeyFrame();

            srv.request.ID = (*mit);

            try {
                if (client.call(srv)) {

                    std::string ss = srv.response.DATA;

                    std::string pose = srv.response.POSE;

                    size = size + ss.size() + pose.size();

                    if (ss == "") { ;
                    } else {

                        std::stringstream is(ss);

                        boost::archive::text_iarchive ia(is);

                        ia >> tKF;

                        tKF->setCache(pCacher);

                        if( pose != "") {
                            std::stringstream ps( pose );
                            boost::archive::text_iarchive pa(ps);
                            cv::Mat pose_mat;
                            pa >> pose_mat;
                            tKF->SetPose( pose_mat );
                        }

                        ans_kfs.insert(tKF);

                    }

                }
                else {
                    ROS_ERROR("Failed to call service get");
                }
            } catch (...) {
                ROS_INFO("get KeyFrame %d error", (int)(*mit));
            }

        }

        end_t = clock();

        cout << "Get kf size " << pKFs.size() << " use time " <<
        (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

        f4 << size << " " <<  (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

        f4.close();

        return ans_kfs;

    }

    // trans the MapPoints to ORBSlam server
    std::vector<long unsigned int>  DataDriver::TransMapPointsToServerOneByOne(std::set<MapPoint *> pMPs) {

        vector<long unsigned int> ans_mps;
        ans_mps.clear();
        if (pMPs.size() <= 0) return ans_mps;

        cout  << endl;

        time_t start_t, end_t;
        start_t = clock();

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_save>("saveOneMapPoint");
        orbslam_server::orbslam_save srv;

        for (std::set<MapPoint *>::iterator mit = pMPs.begin(); mit != pMPs.end(); mit++) {

            if ((*mit)) {
                std::ostringstream os;
                boost::archive::text_oarchive oa(os);
                oa << (*mit);

                std::string ss = os.str();

                srv.request.ID = (*mit)->mnId;

                srv.request.DATA = ss;

                if (client.call(srv)) {
                    ans_mps.push_back((*mit)->mnId);
                }
                else {
                    ROS_ERROR("Failed to call service save");
                }
            }

        }

        end_t = clock();

        cout << "TransMapPointToserver size " << pMPs.size() << " use time " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

        return ans_mps;


    }

    // trans the MapPoints from ORBSlam server

    std::set<MapPoint *> DataDriver::TransMapPointsFromServerOneByOne(std::set<long unsigned int> pLMPs) {

        time_t start_t, end_t;
        start_t = clock();

        std::set<MapPoint *> ans_mps;
        ans_mps.clear();

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_get>("getOneMapPoint");
        orbslam_server::orbslam_get srv;

        for (std::set<long unsigned int>::iterator mit = pLMPs.begin(); mit != pLMPs.end(); mit++) {

            MapPoint *tMP = new MapPoint();

            srv.request.ID = (*mit);

            try {
                if (client.call(srv)) {

                    std::string ss = srv.response.DATA;
                    if (ss == "") { ;
                    } else {

                        std::stringstream is(ss);

                        boost::archive::text_iarchive ia(is);

                        ia >> tMP;

                        tMP->setCache(pCacher);

                        ans_mps.insert(tMP);

                    }

                }
                else {
                    ROS_ERROR("Failed to call service get");
                }
            } catch (...) {
                ROS_INFO("get MapPoint %d error", (int)(*mit));
            }
        }

        end_t = clock();

        cout << "Get MapPoints use time : " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

        return ans_mps;

    }

    void DataDriver::getAllKeyFramePose(){

        cout << "-- begin getAllKeyFramePose\n";
        time_t start_t, end_t;
        start_t = clock();

        pCacher->mTopoMap->mpKfPose.clear();

        std::map<long unsigned int, std::string > kf_pose;

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_pose_get>("getAllKeyFramePose");
        orbslam_server::orbslam_pose_get srv;

        if (client.call(srv)) {

            std::stringstream sPose(srv.response.POSE);
            boost::archive::text_iarchive iPose( sPose );

            iPose >> kf_pose;

        }

        for( std::map<long unsigned int, std::string>::iterator mit = kf_pose.begin(); mit != kf_pose.end(); mit ++ ) {

            std::map<long unsigned int, cv::Mat> subkf_pose;
            stringstream tss( (*mit).second );
            boost::archive::text_iarchive tis( tss );
            tis >> subkf_pose;
            for( std::map<long unsigned int, cv::Mat>::iterator mmit = subkf_pose.begin(); mmit != subkf_pose.end(); mmit ++ ) {
                pCacher->mTopoMap->mpKfPose[ (*mmit).first ] = (*mmit).second;
            }
            subkf_pose.clear();
        }
        kf_pose.clear();

        vector<KeyFrame*> kfsInCache = pCacher->getMpMap()->GetAllKeyFrames();

        for( int i = 0; i < kfsInCache.size() ; i++) {
            pCacher->mTopoMap->mpKfPose[ kfsInCache[i]->mnId ] = kfsInCache[i]->GetPose();
        }

        end_t = clock();
        cout << "getAllKeyFramePose use time " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

    }

    void DataDriver::getAllMapPointPose(){

        cout << "-- begin getAllMapPointPose\n";
        time_t start_t, end_t;
        start_t = clock();

        pCacher->mTopoMap->mpMpPose.clear();
        pCacher->mTopoMap->mpMpObservations.clear();

        vector< pair<long unsigned int, std::string > > kf_pose;

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_pose_get>("getAllTopoMapPointPose");
        orbslam_server::orbslam_pose_get srv;

        if (client.call(srv)) {

            std::stringstream sPose(srv.response.POSE);
            boost::archive::text_iarchive iPose( sPose );

            iPose >> kf_pose;

        }

        for( int i = 0 ; i <kf_pose.size(); i++ ) {

            stringstream tss( kf_pose[i].second );
            boost::archive::text_iarchive tis( tss );
            std::map<unsigned long int, pair<cv::Mat,  std::vector< pair < long unsigned int, LoopKeyPoint > > > > tpposes;
            tis >> tpposes;
            for( std::map<unsigned long int, pair<cv::Mat, std::vector< pair < long unsigned int, LoopKeyPoint > > > >::iterator mit = tpposes.begin();
                    mit != tpposes.end(); mit ++ ) {
                pCacher->mTopoMap->mpMpPose[ (*mit).first ] = (*mit).second.first;
                pCacher->mTopoMap->mpMpObservations[ (*mit).first ] = (*mit).second.second;
            }
            tpposes.clear();

        }

        vector<MapPoint*> mpsInCache = pCacher->getMpMap()->GetAllMapPoints();

        for( int i = 0; i < mpsInCache.size() ; i++) {
            pCacher->mTopoMap->mpMpPose[ mpsInCache[i]->mnId ] = mpsInCache[i]->GetWorldPos();
            pCacher->mTopoMap->mpMpObservations[ mpsInCache[i]->mnId ] = mpsInCache[i]->getObeservationIds();
        }
        kf_pose.clear();
        mpsInCache.clear();
        end_t = clock();
        cout << "getAllMapPointPose use time " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;
    }

    void DataDriver::updateAllKeyFramePose(){

        cout << "-- begin updateAllKeyFramePose\n";
        time_t start_t, end_t;
        start_t = clock();

        std::map< long unsigned int, std::string >  vecKfPose;
        for( std::map< TopoId, std::set<long unsigned int > >::iterator topoKfIter = pCacher->mTopoMap->mpTopoKFs.begin();
             topoKfIter != pCacher->mTopoMap->mpTopoKFs.end(); topoKfIter++) {

            std::map<unsigned long int, cv::Mat > tpposes;
            for( std::set<long unsigned int>::iterator mit = (*topoKfIter).second.begin(); mit != (*topoKfIter).second.end() ; mit++ ) {
                tpposes[ *mit ] = pCacher->mTopoMap->mpKfPose[ *mit ];
            }
            std::ostringstream os;
            boost::archive::text_oarchive oa(os);

            oa << tpposes;
            vecKfPose[ (*topoKfIter).first ] = os.str();
            os.clear();
            tpposes.clear();
        }

        std::ostringstream os;
        boost::archive::text_oarchive oa(os);

        oa << vecKfPose;

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_pose_save>("updateAllKeyFramePose");
        orbslam_server::orbslam_pose_save srv;

        srv.request.POSE = os.str();

        if (client.call(srv)) {
            ;
        } else {
            ROS_INFO( "update all kf pose error");
        }

        vecKfPose.clear();
        os.clear();
        //pCacher->mTopoMap->mpKfPose.clear();

        end_t = clock();
        cout << "updateAllKeyFramePose use time " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

    }

    void DataDriver::updateAllMapPointPose(){


        cout << "-- begin updateAllMapPointPose\n";
        time_t start_t, end_t;
        start_t = clock();

        std::vector< pair<long unsigned int, std::string> >  vecMPPose;

        for( std::map<TopoId, set<long unsigned int> >::iterator mit = pCacher->mTopoMap->mpTopoMps.begin();
                mit != pCacher->mTopoMap->mpTopoMps.end(); mit++ ) {

            std::map<unsigned long int, pair<cv::Mat,  std::vector< pair < long unsigned int, LoopKeyPoint > > > > tpposes;

            for( std::set<long unsigned int>::iterator mpsit = (*mit).second.begin(); mpsit!= (*mit).second.end(); mpsit ++ ) {

                if( pCacher->mTopoMap->mpMpPose.find( *mpsit ) != pCacher->mTopoMap->mpMpPose.end() ) {
                    if( pCacher->mTopoMap->mpMpObservations.find( *mpsit ) != pCacher->mTopoMap->mpMpObservations.end()) {
                        tpposes[ *mpsit ] = make_pair(pCacher->mTopoMap->mpMpPose[*mpsit],
                                                      pCacher->mTopoMap->mpMpObservations[*mpsit] );
                    }
                }

            }

            std::ostringstream os;
            boost::archive::text_oarchive oa(os);

            oa << tpposes;

            vecMPPose.push_back( make_pair( (*mit).first, os.str() ));

            tpposes.clear();
            os.clear();

        }

        std::ostringstream os;
        boost::archive::text_oarchive oa(os);

        oa << vecMPPose;

        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<orbslam_server::orbslam_pose_save>("updateAllTopoMapPointPose");
        orbslam_server::orbslam_pose_save srv;

        srv.request.POSE = os.str();

        if (client.call(srv)) {
            ;
        } else {
            ROS_INFO( "update all mp pose error");
        }

        os.clear();
        vecMPPose.clear();
        end_t = clock();
        cout << "updateAllMapPointPose use time " << (double) (end_t - start_t) / (double) CLOCKS_PER_SEC << endl;

    }

}

