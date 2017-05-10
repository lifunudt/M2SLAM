// file      : hello/person.hxx
// copyright : not copyrighted - public domain

#ifndef PERSON_HXX
#define PERSON_HXX

#include <string>
#include <cstddef> // std::size_t

#include <odb/core.hxx>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/any.hpp>

#pragma db object

class person {

private:
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version) {
        ar & first_;
        ar & last_;
        ar & age_;
    }

public:
    person() {

    }

    person(const std::string &first,
           const std::string &last,
           unsigned short age)
            : first_(first), last_(last), age_(age) {
    }

    const std::string & first() const {
        return first_;
    }

    const std::string & last() const {
        return last_;
    }

    unsigned short age() const {
        return age_;
    }

    void age(unsigned short age) {
        age_ = age;
    }

private:
    friend class odb::access;

#pragma db id auto
    unsigned long id_;

    std::string first_;
    std::string last_;
    unsigned short age_;
};

#pragma db view object(person)
struct person_stat {
#pragma db column("count(" + person::id_ + ")")
    std::size_t count;

#pragma db column("min(" + person::age_ + ")")
    unsigned short min_age;

#pragma db column("max(" + person::age_ + ")")
    unsigned short max_age;
};

#endif // PERSON_HXX
