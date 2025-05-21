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
    m.def("optimize_path", &PathFinder::optimizePath, "Optimize path by removing unnecessary waypoints");
    m.def("reverse_optimize_path", &PathFinder::reverseOptimizePath, "Optimize path from end to start");
    m.def("split_long_segments", &PathFinder::splitLongSegments, "Split path segments longer than max_length");
    m.def("multi_pass_optimize", &PathFinder::multiPassOptimize, "Perform alternating forward/reverse optimization passes");
}