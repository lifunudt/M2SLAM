//
// Created by lifu on 2/20/17.
//

#ifndef PROJECT_DATA_KEYFRAME_H
#define PROJECT_DATA_KEYFRAME_H

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

class Data_KeyFrame {

public:
    Data_KeyFrame() {

    }

    Data_KeyFrame(long unsigned int kfid ,std::string pose, std::string  data) {

        kfid_ = kfid;
        Data_KeyFrame::pose_ = pose;
        Data_KeyFrame::data_ = data;

    }
    Data_KeyFrame* getData_KeyFrame() {
        return this;
    }

    unsigned long getId_() const {
        return id_;
    }

    void setId_(unsigned long id_) {
        Data_KeyFrame::id_ = id_;
    }

    unsigned long getKfid_() const {
        return kfid_;
    }

    void setKfid_(unsigned long kfid_) {
        Data_KeyFrame::kfid_ = kfid_;
    }


    const string &getData_() const {
        return data_;
    }

    void setData_(const string &data_) {
        Data_KeyFrame::data_ = data_;
    }

    const string &getPose_() const {
        return pose_;
    }

    void setPose_(const string &pose_) {
        Data_KeyFrame::pose_ = pose_;
    }

private:
    friend class odb::access;

#pragma db id auto
    unsigned long id_;

    long unsigned int kfid_;

    std::string pose_;

    std::string data_;

};

#endif //PROJECT_DATA_KEYFRAME_H