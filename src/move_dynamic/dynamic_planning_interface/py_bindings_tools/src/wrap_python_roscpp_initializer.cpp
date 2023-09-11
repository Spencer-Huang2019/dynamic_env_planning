//
// Created by spencer on 1/4/23.
//

#include <move_dynamic/py_bindings_tools/roscpp_initializer.h>
#include <boost/python.hpp>
#include <Python.h>

namespace bp = boost::python;

static void wrap_roscpp_initializer()
{
  void (*init_fn)(const std::string&, bp::list&) = &move_dynamic::py_bindings_tools::roscpp_init;
  bp::def("roscpp_init", init_fn);
  bp::def("roscpp_shutdown", &move_dynamic::py_bindings_tools::roscpp_shutdown);
}

BOOST_PYTHON_MODULE(_move_dynamic_roscpp_initializer)
{
  wrap_roscpp_initializer();
}
