#include "a_star/a_star_search.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

PYBIND11_MODULE(a_star, m) {
  // enum
  py::enum_<HeuristicType>(m, "HeuristicType")
      .value("EUCLIDEAN", HeuristicType::kEuclidean)
      .value("MANHATTAN", HeuristicType::kManhattan)
      .value("DIAGONAL", HeuristicType::kDiagonal)
      .export_values();

  auto pyAstar = py::class_<Astar>(m, "Astar");
  pyAstar
      .def(py::init<HeuristicType>(),
           py::arg("h_type") = HeuristicType::kDiagonal)
      .def("init", &Astar::Init)
      .def("search", &Astar::Search)
      .def("debug", &Astar::Debug)
      .def("get_result_matrix", &Astar::GetResultMatrix)
      .def("get_cost_layer", &Astar::GetCostLayer)
      .def("get_perception_cost_layer", &Astar::GetPerceptionCostLayer)
      .def("get_ele_layer", &Astar::GetEleLayer)
      .def("update_global_path_perception", &Astar::UpdateGlobalPathPerception)
      .def("decay_global_path_perception", &Astar::DecayGlobalPathPerception)
      .def("clear_global_path_perception_indices",
           &Astar::ClearGlobalPathPerceptionIndices)
      .def("clear_global_path_perception", &Astar::ClearGlobalPathPerception)
      .def("get_global_path_perception_cell_count",
           &Astar::GetGlobalPathPerceptionCellCount)
      .def("get_visited_set", &Astar::GetVisitedSet);
}
