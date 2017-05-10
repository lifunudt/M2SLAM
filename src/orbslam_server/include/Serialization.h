//
// Created by lifu on 4/3/17.
//

#ifndef PROJECT_SERIALIZATION_H
#define PROJECT_SERIALIZATION_H

#include <opencv2/opencv.hpp>
#include <boost/any.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/detail/basic_oserializer.hpp>
#include "cstring"

using namespace std;

namespace boost {
    namespace serialization {

        //serialize the opencv mat class
        template<class Archive>
        void serialize(Archive &ar, pair<long unsigned int, std::string> &par, const unsigned int) {
            ar & par.first & par.second;
        }
    }
}

#endif //PROJECT_SERIALIZATION_H
