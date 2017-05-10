// file      : orbserver
// copyright : not copyrighted - public domain

#include <memory>   // std::auto_ptr
#include <iostream>
#include <sstream>

#include <odb/database.hxx>
#include <odb/transaction.hxx>


#include "ros/ros.h"
#include "orbslam_server/orbslam_save.h"
#include "orbslam_server/orbslam_get.h"
#include "orbslam_server/orbslam_muilt_save.h"
#include "orbslam_server/orbslam_muilt_get.h"
#include "orbslam_server/orbslam_pose_get.h"
#include "orbslam_server/orbslam_pose_save.h"

#include "database.h" // create_database

#include "person.h"
#include "person_odb.h"
#include "Data_KeyFrame.h"
#include "Data_KeyFrame-odb.hxx"
#include "Data_MapPoint.h"
#include "Data_MapPoint-odb.hxx"
#include "Data_TopoMapPoint.h"
#include "Data_TopoMapPoint-odb.hxx"
#include "Data_TopoKeyFrame.h"
#include "Data_TopoKeyFrame-odb.hxx"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>

#include "Serialization.h"

#include <boost/any.hpp>

using namespace std;
using namespace odb::core;

auto_ptr<database> db;

bool saveOneMapPoint(orbslam_server::orbslam_save::Request &req,
                     orbslam_server::orbslam_save::Response &res) {

    unsigned long usr_id;

    std::string ss = req.DATA;

    std::string pose = req.POSE;

    unsigned long id = req.ID;

    {
        typedef odb::query<Data_MapPoint> query;
        typedef odb::result<Data_MapPoint> result;

        {
            transaction t(db->begin());
            result r(db->query<Data_MapPoint>(query::mpid == id));
            if (r.size() > 0) {
                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                    Data_MapPoint *tmp = mit->getData_MapPoint();
                    tmp->setData_(ss);
                    tmp->setPose_(pose);
                    db->update(tmp);
                    usr_id = tmp->getId_();
                }
            } else {
                Data_MapPoint tmp(id, pose, ss);
                usr_id = db->persist(tmp);
            }
            t.commit();
        }

    }

    res.ID = usr_id;

    ROS_INFO("MapPoint ID : %d saved! ", (int) res.ID);

    return true;

}

bool getOneMapPoint(orbslam_server::orbslam_get::Request &req,
                    orbslam_server::orbslam_get::Response &res) {
    std::string ss = "";
    res.DATA = ss;
    std::string pose = "";
    res.POSE = pose;

    try {

        typedef odb::query<Data_MapPoint> query;
        typedef odb::result<Data_MapPoint> result;

        {
            transaction t(db->begin());

            result r(db->query<Data_MapPoint>(query::mpid == req.ID));
            for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                res.DATA = mit->getData_();
                res.POSE = mit->getPose_();
                break;
            }
            t.commit();
        }
    }
    catch (const odb::exception &e) {
        cerr << e.what() << endl;
        return false;
    }

    return true;

}


bool saveTopoMapPoint(orbslam_server::orbslam_save::Request &req,
                     orbslam_server::orbslam_save::Response &res) {

    unsigned long usr_id;

    std::string ss = req.DATA;

    std::string pose = req.POSE;

    unsigned long id = req.ID;

    {
        typedef odb::query<Data_TopoMapPoint> query;
        typedef odb::result<Data_TopoMapPoint> result;

        {
            transaction t(db->begin());
            result r(db->query<Data_TopoMapPoint>(query::topoId == id));
            if (r.size() > 0) {
                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                    Data_TopoMapPoint *tmp = mit->getData_TopoMapPoint();
                    tmp->setData_(ss);
                    tmp->setPose_(pose);
                    db->update(tmp);
                    usr_id = tmp->getId_();
                    break;
                }
            } else {
                Data_TopoMapPoint tmp(id,pose, ss);
                usr_id = db->persist(tmp);
            }
            t.commit();
        }

    }

    res.ID = usr_id;

    ROS_INFO("TopoMapPoint ID : %d saved! ", (int) res.ID);

    return true;

}

bool getTopoMapPoint(orbslam_server::orbslam_get::Request &req,
                    orbslam_server::orbslam_get::Response &res) {

    ROS_INFO("getTopoMapPoint function");

    std::string ss = "";

    std::string pose = "";

    res.DATA = ss;

    res.POSE = pose;

    try {

        typedef odb::query<Data_TopoMapPoint> query;
        typedef odb::result<Data_TopoMapPoint> result;

        {
            transaction t(db->begin());
            result r(db->query<Data_TopoMapPoint>(query::topoId == req.ID));
            if (r.size() > 0) {

                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                    res.DATA = mit->getData_();
                    res.POSE = mit->getPose_();
                    break;
                }
            }

            t.commit();
        }
    }
    catch (const odb::exception &e) {
        cerr << e.what() << endl;
        return false;
    }

    return true;

}


bool saveTopoKeyFrame(orbslam_server::orbslam_save::Request &req,
                      orbslam_server::orbslam_save::Response &res) {

    unsigned long usr_id;

    std::string ss = req.DATA;

    std::string pose = req.POSE;

    unsigned long id = req.ID;

    {
        typedef odb::query<Data_TopoKeyFrame> query;
        typedef odb::result<Data_TopoKeyFrame> result;

        {
            transaction t(db->begin());
            result r(db->query<Data_TopoKeyFrame>(query::topoId == id));
            if (r.size() > 0) {
                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                    Data_TopoKeyFrame *tmp = mit->getData_TopoKeyFrame();
                    tmp->setData_(ss);
                    tmp->setPose_(pose);
                    db->update(tmp);
                    usr_id = tmp->getId_();
                    break;
                }
            } else {
                Data_TopoKeyFrame tmp(id,pose, ss);
                usr_id = db->persist(tmp);
            }
            t.commit();
        }

    }

    res.ID = usr_id;

    ROS_INFO("Data_TopoKeyFrame ID : %d saved! ", (int) res.ID);

    return true;

}

bool getTopoKeyFrame(orbslam_server::orbslam_get::Request &req,
                     orbslam_server::orbslam_get::Response &res) {

    ROS_INFO("GetData_TopoKeyFrame function");

    std::string ss = "";

    std::string pose = "";

    res.DATA = ss;

    res.POSE = pose;

    try {

        typedef odb::query<Data_TopoKeyFrame> query;
        typedef odb::result<Data_TopoKeyFrame> result;

        {
            transaction t(db->begin());
            result r(db->query<Data_TopoKeyFrame>(query::topoId == req.ID));
            if (r.size() > 0) {

                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                    res.DATA = mit->getData_();
                    res.POSE = mit->getPose_();
                    break;
                }
            }

            t.commit();
        }
    }
    catch (const odb::exception &e) {
        cerr << e.what() << endl;
        return false;
    }

    return true;

}

bool saveOneKeyFrame(orbslam_server::orbslam_save::Request &req,
                     orbslam_server::orbslam_save::Response &res) {

    unsigned long usr_id;

    std::string ss = req.DATA;

    std::string pose = req.POSE;

    unsigned long id = req.ID;

    {
        typedef odb::query<Data_KeyFrame> query;
        typedef odb::result<Data_KeyFrame> result;

        {
            transaction t(db->begin());
            result r(db->query<Data_KeyFrame>(query::kfid == id));
            if (r.size() > 0) {
                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                    Data_KeyFrame *tkf = mit->getData_KeyFrame();
                    tkf->setData_(ss);
                    tkf->setPose_(pose);
                    db->update(tkf);
                    usr_id = tkf->getId_();
                }
            } else {
                Data_KeyFrame tkf(id, pose, ss);
                usr_id = db->persist(tkf);
            }
            t.commit();
        }

    }

    res.ID = usr_id;

    ROS_INFO("KeyFrame ID : %d saved!", (int) res.ID);

    return true;
}

bool getOneKeyFrame(orbslam_server::orbslam_get::Request &req,
                    orbslam_server::orbslam_get::Response &res) {

    std::string ss = "";
    res.DATA = ss;
    std::string pose = "";
    res.POSE = pose;

    try {

        typedef odb::query<Data_KeyFrame> query;
        typedef odb::result<Data_KeyFrame> result;

        {
            transaction t(db->begin());

            result r(db->query<Data_KeyFrame>(query::kfid == req.ID));
            for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                res.DATA = mit->getData_();
                res.POSE = mit->getPose_();
                break;
            }
            t.commit();
        }
    }
    catch (const odb::exception &e) {
        cerr << e.what() << endl;
        return false;
    }

    return true;

}

bool getAllKeyFramePose( orbslam_server::orbslam_pose_get::Request &req,
                         orbslam_server::orbslam_pose_get::Response &res ) {

    std::map< long unsigned int, std::string > ans_pose;

    try {

        typedef odb::query<Data_TopoKeyFrame> query;
        typedef odb::result<Data_TopoKeyFrame> result;

        {
            transaction t(db->begin());

            result r(db->query<Data_TopoKeyFrame>(query::topoId.is_not_null() ));
            for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                ans_pose [ mit->getTopoId_() ] =  mit->getPose_()  ;
            }

            t.commit();
        }
    }
    catch (const odb::exception &e) {
        cerr << e.what() << endl;
        return false;
    }

    std::stringstream aos;

    boost::archive::text_oarchive aoa(aos);

    aoa << ans_pose ;

    res.POSE = aos.str();

    ROS_INFO( "Get all keyFrame pose " );

    return true;

}

bool updateAllKeyFramePose( orbslam_server::orbslam_pose_save::Request &req,
                            orbslam_server::orbslam_pose_save::Response &res ) {

    std::map<long unsigned int, std::string > kf_pose;

    kf_pose.clear();

    std::stringstream is(req.POSE);

    boost::archive::text_iarchive ia(is);

    ia >> kf_pose;

    try {

        typedef odb::query<Data_TopoKeyFrame> query;
        typedef odb::result<Data_TopoKeyFrame> result;

        {
            transaction t(db->begin());
            for( std::map<long unsigned int, string>::iterator mit = kf_pose.begin(); mit != kf_pose.end(); mit ++ ) {
                result r(db->query<Data_TopoKeyFrame>(query::topoId == (*mit).first ));
                for (result::iterator mmit(r.begin()); mmit != r.end(); ++mmit) {
                    Data_TopoKeyFrame *tmp = mmit->getData_TopoKeyFrame();
                    tmp->setPose_( (*mit).second);
                    db->update(tmp);
                    break;
                }
            }
            t.commit();
        }
    }
    catch (const odb::exception &e) {
        cerr << e.what() << endl;
        return false;
    }

    ROS_INFO( "Update All KeyFrame pose size: %d", (int) kf_pose.size() );
    return true;

}

bool getAllTopoMapPointPose( orbslam_server::orbslam_pose_get::Request &req,
                         orbslam_server::orbslam_pose_get::Response &res ) {

    vector< pair<long unsigned int, std::string > > ans_pose;

    try {

        typedef odb::query<Data_TopoMapPoint> query;
        typedef odb::result<Data_TopoMapPoint> result;

        {
            transaction t(db->begin());

            result r(db->query<Data_TopoMapPoint>(query::topoId.is_not_null() ));
            for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                ans_pose.push_back( make_pair( mit->getTopoId_(), mit->getPose_() ) ) ;
            }

            t.commit();
        }
    }
    catch (const odb::exception &e) {
        cerr << e.what() << endl;
        return false;
    }

    std::stringstream aos;

    boost::archive::text_oarchive aoa(aos);

    aoa << ans_pose ;
    res.POSE = aos.str();
    ROS_INFO( "Get all Topo MapPoints " );

    return true;

}

bool updateAllTopoMapPointPose( orbslam_server::orbslam_pose_save::Request &req,
                            orbslam_server::orbslam_pose_save::Response &res ) {

    vector< pair<long unsigned int, std::string > > kf_pose;

    kf_pose.clear();

    std::stringstream is(req.POSE);

    boost::archive::text_iarchive ia(is);

    ia >> kf_pose;

    try {

        typedef odb::query<Data_TopoMapPoint> query;
        typedef odb::result<Data_TopoMapPoint> result;

        {
            transaction t(db->begin());

            for( int i = 0; i < kf_pose.size(); i ++ ) {
                result r(db->query<Data_TopoMapPoint>(query::topoId == kf_pose[i].first ));
                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
                    Data_TopoMapPoint *tmp = mit->getData_TopoMapPoint();
                    tmp->setPose_( kf_pose[i].second);
                    db->update(tmp);
                    break;
                }
            }
            t.commit();
        }
    }
    catch (const odb::exception &e) {
        cerr << e.what() << endl;
        return false;
    }

    ROS_INFO( "Update All Topo MapPoint pose size: %d", (int) kf_pose.size() );

    return true;

}

//
//bool saveMapPoints(orbslam_server::orbslam_muilt_save::Request &req,
//                   orbslam_server::orbslam_muilt_save::Response &res) {
//
//
//    ROS_INFO( "-- begin saveMapPoints --" );
//
//    vector<pair<unsigned long int, std::string> > pkfs;
//
//    pkfs.clear();
//
//    std::stringstream is(req.DATA);
//
//    boost::archive::text_iarchive ia(is);
//
//    ia >> pkfs;
//
//    typedef odb::query<Data_MapPoint> query;
//    typedef odb::result<Data_MapPoint> result;
//
//    cout << "MapPoint saved size " << pkfs.size() << endl;
//
//    for (int mit = 0; mit < pkfs.size(); mit++) {
//
//        unsigned long int tId = pkfs[mit].first;
//        string tss = pkfs[mit].second;
//        try {
//            transaction t(db->begin());
//            result r(db->query<Data_MapPoint>(query::mpid == tId));
//            if (r.size() > 0) {
//                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
//                    Data_MapPoint *tmp = mit->getData_MapPoint();
//                    tmp->setData_(tss);
//                    db->update(tmp);
//                }
//            } else {
//                Data_MapPoint tmp(tId, tss);
//                db->persist(tmp);
//            }
//            t.commit();
//        } catch (const odb::exception &e) {
//
//            cerr << e.what() << endl;
//
//            return false;
//        }
//
//    }
//
//    res.IDS = "";
//
//    ROS_INFO("-- finish MapPoints --");
//
//    return true;
//
//}

//bool getMapPoints(orbslam_server::orbslam_muilt_get::Request &req,
//                  orbslam_server::orbslam_muilt_get::Response &res) {
//
//    ROS_INFO( "-- begin getMapPoints --");
//
//    std::vector<long unsigned int> tIds;
//
//    tIds.clear();
//
//    std::stringstream is(req.IDS);
//
//    boost::archive::text_iarchive ia(is);
//
//    ia >> tIds;
//
//    globle_vec_pai.clear();
//
//    typedef odb::query<Data_MapPoint> query;
//
//    typedef odb::result<Data_MapPoint> result;
//
//    cout << "the MapPoint Id : ";
//
//    for (int i = 0; i <= tIds.size(); i++) {
//        cout << tIds[i] << " ";
//        try {
//
//            transaction t(db->begin());
//
//            long unsigned int tmp_id = tIds[i];
//
//            std::string tmp_ss;
//
//            result r(db->query<Data_MapPoint>(query::mpid == tmp_id));
//
//            for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
//                tmp_ss = mit->getData_();
//                break;
//            }
//
//            globle_vec_pai.push_back(pair<long unsigned int, std::string>(tmp_id, tmp_ss));
//
//            t.commit();
//
//        } catch (const odb::exception &e) {
//
//            cerr << e.what() << endl;
//
//            return false;
//        }
//
//    }
//    cout << endl;
//
//    std::stringstream aos(res.DATA);
//
//    boost::archive::text_oarchive aoa(aos);
//
//    aoa << globle_vec_pai ;
//
//    ROS_INFO( "-- end getMapPoints --");
//
//    return true;
//
//}

//bool saveKeyFrames(orbslam_server::orbslam_muilt_save::Request &req,
//                   orbslam_server::orbslam_muilt_save::Response &res) {
//
////    globle_string = req.DATA;
//
//    ROS_INFO( "-- begin saveKeyFrames -- ");
//
//    globle_vec_pai.clear();
//
//    std::stringstream is( req.DATA );
//
//    boost::archive::text_iarchive ia(is);
//
//    ia >> globle_vec_pai;
//
//    typedef odb::query<Data_KeyFrame> query;
//    typedef odb::result<Data_KeyFrame> result;
//
//    cout << "size of kfs " << globle_vec_pai.size() << endl;
//
//    for (int mit = 0; mit < globle_vec_pai.size(); mit++) {
//
//        cout << globle_vec_pai[mit ].first << " ";
//        unsigned long int tId = globle_vec_pai[mit].first;
//        string tss = globle_vec_pai[mit].second;
//        {
//            transaction t(db->begin());
//            result r(db->query<Data_KeyFrame>(query::kfid == tId));
//            if (r.size() > 0) {
//                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
//                    Data_KeyFrame *tkf = mit->getData_KeyFrame();
//                    tkf->setData_(tss);
//                    db->update(tkf);
//                }
//            } else {
//                Data_KeyFrame tkf(tId, tss);
//                db->persist(tkf);
//            }
//            t.commit();
//        }
//
//    }
//
//    res.IDS = "";
//
//    ROS_INFO("-- finish saveKeyFrames --");
//
//    return true;
//
//}

//bool getKeyFrames(orbslam_server::orbslam_muilt_get::Request &req,
//                  orbslam_server::orbslam_muilt_get::Response &res) {
//
//    ROS_INFO( "-- begin getKeyFrame -- ");
//
//    std::vector<long unsigned int> tIds;
//
//    tIds.clear();
//
//    std::stringstream is(req.IDS);
//
//    boost::archive::text_iarchive ia(is);
//
//    ia >> tIds;
//
//    globle_vec_pai.clear();
//
//    typedef odb::query<Data_KeyFrame> query;
//
//    typedef odb::result<Data_KeyFrame> result;
//
//    cout << "get keyFrame ID : ";
//    for (int i = 0; i <= tIds.size(); i++) {
//
//        cout << tIds[i] << " ";
//        try {
//
//            transaction t(db->begin());
//
//            long unsigned int tmp_id = tIds[i];
//
//            std::string tmp_ss;
//
//            result r(db->query<Data_KeyFrame>(query::kfid == tmp_id));
//
//            for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
//                tmp_ss = mit->getData_();
//                break;
//            }
//
//            globle_vec_pai.push_back(pair<long unsigned int, std::string>(tmp_id, tmp_ss));
//
//            t.commit();
//
//        } catch (const odb::exception &e) {
//
//            cerr << e.what() << endl;
//
//            return false;
//        }
//
//    }
//
//    std::stringstream aos(res.DATA );
//
//    boost::archive::text_oarchive aoa(aos);
//
//    aoa << globle_vec_pai ;
//
//    ROS_INFO("-- finish getKeyFrames --");
//
//    return true;
//
//
//}


//void test_save(int id, string ss) {
//
//
//    Data_KeyFrame tkf(id, ss);
//
//    {
//        transaction t(db->begin());
//
//        // Make objects persistent and save their ids for later use.
//
//        (unsigned long) db->persist(tkf);
//
//        t.commit();
//
//    }
//
//}
//
//void update_save(int id, string ss) {
//
//
//    {
//        typedef odb::query<Data_KeyFrame> query;
//        typedef odb::result<Data_KeyFrame> result;
//
//        {
//            transaction t(db->begin());
//            result r(db->query<Data_KeyFrame>(query::kfid == id));
//            if (r.size() > 0) {
//                for (result::iterator mit(r.begin()); mit != r.end(); ++mit) {
//
//                    Data_KeyFrame *tkf = mit->getData_KeyFrame();
//                    tkf->setData_(ss);
//                    db->update(tkf);
//                }
//            } else {
//                Data_KeyFrame tkf(id, ss);
//                db->persist(tkf);
//
//            }
//            t.commit();
//        }
//
//    }
//
//}

int main(int argc, char *argv[]) {

    try {

        int db_argc = 9;
        char *db_argv[] = {(char *) "pgsql", (char *) "--user", (char *) "odb_test", (char *) "--database",
                           (char *) "odb_test", (char *) "--host", (char *) "127.0.0.1", (char *) "--password",
                           (char *) "odb_test"};
        db = (create_database(db_argc, db_argv));

        ros::init(argc, argv, "save_test_server");

        ros::NodeHandle n;

        ros::ServiceServer saveOneKeyFrameService = n.advertiseService("saveOneKeyFrame", saveOneKeyFrame);
        ros::ServiceServer getOneKeyFrameService = n.advertiseService("getOneKeyFrame", getOneKeyFrame);
        ros::ServiceServer saveMapPointService = n.advertiseService("saveOneMapPoint", saveOneMapPoint);
        ros::ServiceServer getOneMapPointService = n.advertiseService("getOneMapPoint", getOneMapPoint);

//        ros::ServiceServer saveKeyFramesService = n.advertiseService("saveKeyFrames", saveKeyFrames);
//        ros::ServiceServer getKeyFramesService = n.advertiseService("getKeyFrames", getKeyFrames);
//        ros::ServiceServer saveMapPointsService = n.advertiseService("saveMapPoints", saveMapPoints);
//        ros::ServiceServer getMapPointsService = n.advertiseService("getMapPoints", getMapPoints);

        ros::ServiceServer saveTopoMapPointService = n.advertiseService("saveTopoMapPoint", saveTopoMapPoint);
        ros::ServiceServer getTopoMapPointService = n.advertiseService("getTopoMapPoint", getTopoMapPoint);
        ros::ServiceServer saveTopoKeyFrameService = n.advertiseService("saveTopoKeyFrame", saveTopoKeyFrame);
        ros::ServiceServer getTopoKeyFrameService = n.advertiseService("getTopoKeyFrame", getTopoKeyFrame);

        ros::ServiceServer getAllKeyFramePoseService = n.advertiseService("getAllKeyFramePose", getAllKeyFramePose );
        ros::ServiceServer updateAllKeyFramePoseService = n.advertiseService("updateAllKeyFramePose", updateAllKeyFramePose );
        ros::ServiceServer getAllTopoMapPointPoseService = n.advertiseService("getAllTopoMapPointPose", getAllTopoMapPointPose );
        ros::ServiceServer updateAllTopoMapPointPoseService = n.advertiseService("updateAllTopoMapPointPose", updateAllTopoMapPointPose );


        ROS_INFO("Ready to save OrbSlam and muilt to server.");

        ros::spin();

        return 0;
    }
    catch (const odb::exception &e) {
        cerr << e.what() << endl;
        return 1;
    }
}