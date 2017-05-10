//
// Created by lifu on 4/7/17.
//

#ifndef PROJECT_DATA_TOPOMAPPOINT_H
#define PROJECT_DATA_TOPOMAPPOINT_H


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
#include "../../../../../../usr/include/boost/serialization/string.hpp"

using namespace std;

#pragma db object

class Data_TopoMapPoint {

public:
    Data_TopoMapPoint() {

    }

    Data_TopoMapPoint(long unsigned int Topoid ,std::string pose, std::string data) {

        topoId_ = Topoid;
        Data_TopoMapPoint::pose_ = pose;
        Data_TopoMapPoint::data_ = data;

    }
    Data_TopoMapPoint* getData_TopoMapPoint() {
        return this;
    }


    unsigned long getId_() const {
        return id_;
    }

    void setId_(unsigned long id_) {
        Data_TopoMapPoint::id_ = id_;
    }


    unsigned long getTopoId_() const {
        return topoId_;
    }

    void setTopoId_(unsigned long topoId_) {
        Data_TopoMapPoint::topoId_ = topoId_;
    }

    void setData_(const std::string &data_) {
        Data_TopoMapPoint::data_ = data_;
    }
    std::string getData_() const {
        return data_;
    }

    const string &getPose_() const {
        return pose_;
    }

    void setPose_(const string &pose_) {
        Data_TopoMapPoint::pose_ = pose_;
    }

private:
    friend class odb::access;

#pragma db id auto
    unsigned long id_;

    long unsigned int topoId_;

    std::string pose_;

    std::string data_;

};

#endif //PROJECT_DATA_TOPOMAPPOINT_H
