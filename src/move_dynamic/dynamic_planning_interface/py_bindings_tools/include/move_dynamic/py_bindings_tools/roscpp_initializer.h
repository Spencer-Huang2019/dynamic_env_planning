//
// Created by spencer on 1/4/23.
//

#ifndef DYNAMIC_FRANKA_ROSCPP_INITIALIZER_H
#define DYNAMIC_FRANKA_ROSCPP_INITIALIZER_H

#pragma once

#include <boost/python.hpp>
#include <string>

namespace move_dynamic
{
namespace py_bindings_tools
{
class ROScppInitializer
{
public:
    ROScppInitializer();

    ROScppInitializer(boost::python::list &argv);

    ROScppInitializer(const std::string &node_name, boost::python::list &argv);
};

void roscpp_set_arguments(const std::string &node_name, boost::python::list &argv);

void roscpp_init(const std::string &node_name, boost::python::list &argv);

void roscpp_init(boost::python::list &argv);

void roscpp_init();

void roscpp_shutdown();
}
}

#endif //DYNAMIC_FRANKA_ROSCPP_INITIALIZER_H
