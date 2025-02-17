#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "grid_map.h"
#include "penal_traj_opt.h"

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using  namespace planner;

PYBIND11_MODULE(planner, m) {
    m.doc() = R"pbdoc(
        currentmodule:: planner
    )pbdoc";

    py::class_<PenalTrajOpt>(m, "PenalTrajOpt")
    .def(py::init())
    .def("init", &PenalTrajOpt::init)
    .def("setMap", &PenalTrajOpt::setMap)
    .def("plan", &PenalTrajOpt::plan)
    .def("getTraj", &PenalTrajOpt::getTraj)
    .def("__repr__",
        [](const PenalTrajOpt &a) {
            return "<planner.PenalTrajOpt>";
        }
    );

    py::class_<SE2Trajectory>(m, "SE2Trajectory")
    .def(py::init())
    .def("getTotalDuration", &SE2Trajectory::getTotalDuration)
    .def("getPos", &SE2Trajectory::getPos)
    .def("__repr__",
        [](const SE2Trajectory &a) {
            return "<planner.SE2Trajectory>";
        }
    );

    py::class_<GridMap>(m, "GridMap")
    .def(py::init())
    .def("init", &GridMap::init)
    .def("setMap", &GridMap::setMap)
    .def("getMap", &GridMap::getMap)
    .def("astarPlan", &GridMap::astarPlan)
    .def("__repr__",
        [](const GridMap &a) {
            return "<planner.GridMap>";
        }
    );

    m.def("indexToPos", 
        [](GridMap &a, Eigen::Ref<Eigen::VectorXi> idx, Eigen::Ref<Eigen::VectorXd> pos)
        {
            Eigen::Vector2d pos_2(pos[0], pos[1]);
            Eigen::Vector2i idx_2(idx[0], idx[1]);
            a.indexToPos(idx_2, pos_2);
            pos[0] = pos_2[0];
            pos[1] = pos_2[1];
        });
    m.def("posToIndex", 
        [](GridMap &a, Eigen::Ref<Eigen::VectorXd> pos, Eigen::Ref<Eigen::VectorXi> idx)
        {
            Eigen::Vector2d pos_2(pos[0], pos[1]);
            Eigen::Vector2i idx_2(idx[0], idx[1]);
            a.posToIndex(pos_2, idx_2);
            idx[0] = idx_2[0];
            idx[1] = idx_2[1];
        });

    m.attr("__version__") = "0.0.1";
}
