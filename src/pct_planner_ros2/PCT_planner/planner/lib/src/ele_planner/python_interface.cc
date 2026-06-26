#include "ele_planner/offline_ele_planner.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

PYBIND11_MODULE(ele_planner, m) {
  auto pyOfflineElePlanner =
      py::class_<OfflineElePlanner>(m, "OfflineElePlanner");
  pyOfflineElePlanner
      .def(py::init<double, bool>(), py::arg("max_heading_rate"),
           py::arg("use_quintic") = false)
      .def("init_map", &OfflineElePlanner::InitMap)
      .def("plan", &OfflineElePlanner::Plan)
      .def("update_global_path_perception",
           &OfflineElePlanner::UpdateGlobalPathPerception)
      .def("decay_global_path_perception",
           &OfflineElePlanner::DecayGlobalPathPerception)
      .def("clear_global_path_perception_indices",
           &OfflineElePlanner::ClearGlobalPathPerceptionIndices)
      .def("clear_global_path_perception",
           &OfflineElePlanner::ClearGlobalPathPerception)
      .def("get_global_path_perception_cell_count",
           &OfflineElePlanner::GetGlobalPathPerceptionCellCount)
      .def("build_global_path_perception_mark_indices",
           &OfflineElePlanner::BuildGlobalPathPerceptionMarkIndices)
      .def("build_global_path_perception_clear_indices",
           &OfflineElePlanner::BuildGlobalPathPerceptionClearIndices)
      .def("debug", &OfflineElePlanner::Debug)
      .def("set_reference_height", &OfflineElePlanner::SetReferenceHeight)
      .def("set_max_iterations", &OfflineElePlanner::set_max_iterations)
      .def("get_path_finder", &OfflineElePlanner::get_path_finder)
      .def("get_map", &OfflineElePlanner::get_map)
      .def("get_trajectory_optimizer",
           &OfflineElePlanner::get_trajectory_optimizer)
      .def("get_trajectory_optimizer_wnoj",
           &OfflineElePlanner::get_trajectory_optimizer_wnoj)
      .def("get_debug_path", &OfflineElePlanner::GetDebugPath);
}
