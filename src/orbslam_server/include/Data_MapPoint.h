//
// Created by lifu on 3/23/17.
//

#ifndef PROJECT_DATA_MAPPOINT_H
#define PROJECT_DATA_MAPPOINT_H

#include <string>
#include <cstddef> // std::size_t

#include <vector>
#include <odb/core.hxx>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/any.hpp>

using namespace std;

#pragma db object

class Data_MapPoint {

public:
    Data_MapPoint() {

    }

    Data_MapPoint(long unsigned int kfid , std::string pose, std::string  data) {

        mpid_ = kfid;
        Data_MapPoint::pose_ = pose;
        Data_MapPoint::data_ = data;

    }
    Data_MapPoint* getData_MapPoint() {
        return this;
    }


    unsigned long getId_() const {
        return id_;
    }

    void setId_(unsigned long id_) {
        Data_MapPoint::id_ = id_;
    }

    unsigned long getMpid_() const {
        return mpid_;
    }

    void setMpid_(unsigned long mpid_) {
        Data_MapPoint::mpid_ = mpid_;
    }

    const string &getData_() const {
        return data_;
    }

    void setData_(const string &data_) {
        Data_MapPoint::data_ = data_;
    }

    const string &getPose_() const {
        return pose_;
    }

    void setPose_(const string &pose_) {
        Data_MapPoint::pose_ = pose_;
    }

private:
    friend class odb::access;

#pragma db id auto
    unsigned long id_;

    long unsigned int mpid_;

    std::string pose_;

    std::string data_;

};


#endif //PROJECT_DATA_MAPPOINT_H
