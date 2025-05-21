#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "pathfinder.h"

namespace py = pybind11;

PYBIND11_MODULE(pathfinder, m) {
    m.doc() = "Python bindings for Theta* pathfinding implementation";

    py::class_<std::vector<PathFinder::Point>>(m, "Path")
        .def(py::init<>())
        .def("__len__", [](const std::vector<PathFinder::Point> &v) { return v.size(); })
        .def("__iter__", [](std::vector<PathFinder::Point> &v) {
            return py::make_iterator(v.begin(), v.end());
        }, py::keep_alive<0, 1>());

    m.def("find_path", &PathFinder::findPath, "Theta* pathfinding algorithm");
}