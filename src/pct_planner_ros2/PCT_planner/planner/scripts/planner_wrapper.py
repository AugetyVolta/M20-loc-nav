import os
import sys
import pickle
import numpy as np
from scipy.interpolate import griddata  # 新增：用于XY坐标的高度插值

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPlanner(object):
    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate
        self.a_star_cost_threshold = getattr(self.cfg.planner, 'a_star_cost_threshold', 20)
        self.safe_cost_margin = getattr(self.cfg.planner, 'safe_cost_margin', 15)
        self.step_cost_weight = getattr(self.cfg.planner, 'step_cost_weight', 1)
        self.layer_match_height_tolerance = getattr(self.cfg.planner, 'layer_match_height_tolerance', 1.0)
        self.path_ground_offset = getattr(self.cfg.planner, 'path_ground_offset', 0.1)

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.tomogram=None
        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.layer_elev_grids = None  # 存储每个layer的高度网格（elev_g）
        self.grid_xy_coords = None    # 存储网格对应的物理XY坐标

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)
        self.last_plan_info = {}
        self.last_traj_layers = None

    def loadTomogram(self, tomo_file):
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            self.tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = self.tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [self.tomogram.shape[2], self.tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = self.tomogram[0]
        trav_gx = self.tomogram[1]
        trav_gy = self.tomogram[2]
        elev_g = self.tomogram[3]
        elev_g = np.nan_to_num(elev_g, nan=-100)
        elev_c = self.tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)
        # 存储所有layer的高度网格（关键：后续用于XY查高度）
        self.layer_elev_grids = elev_g
        # 预计算网格对应的物理XY坐标（避免重复计算）,可行性分析的trav导入，在目标点decide_layer，选择最低cost的layer
        self._precompute_grid_xy()
        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
      
    def update_global_path_perception(
        self,
        perception_indices,
        inflation_radius,
        inscribed_radius,
        perception_cost,
        cost_scaling_factor,
        stamp,
        persistence,
        clear_center,
        clear_radius,
    ):
        perception_indices = np.asarray(perception_indices, dtype=np.int32)
        if perception_indices.size == 0:
            perception_indices = np.zeros((0, 3), dtype=np.int32)
        perception_indices = perception_indices.reshape((-1, 3))
        clear_center = np.asarray(clear_center, dtype=np.int32).reshape((3,))
        return int(
            self.planner.update_global_path_perception(
                perception_indices,
                float(inflation_radius),
                float(inscribed_radius),
                float(perception_cost),
                float(cost_scaling_factor),
                float(stamp),
                float(persistence),
                clear_center,
                float(clear_radius),
            )
        )

    def apply_global_path_perception(
        self,
        mark_indices,
        clear_indices,
        inflation_radius,
        inscribed_radius,
        perception_cost,
        cost_scaling_factor,
        stamp,
        persistence,
        clear_center,
        clear_radius,
        window_bounds,
    ):
        mark_indices = np.asarray(mark_indices, dtype=np.int32)
        if mark_indices.size == 0:
            mark_indices = np.zeros((0, 3), dtype=np.int32)
        mark_indices = mark_indices.reshape((-1, 3))
        clear_indices = np.asarray(clear_indices, dtype=np.int32)
        if clear_indices.size == 0:
            clear_indices = np.zeros((0, 3), dtype=np.int32)
        clear_indices = clear_indices.reshape((-1, 3))
        clear_center = np.asarray(clear_center, dtype=np.int32).reshape((3,))
        window_bounds = np.asarray(window_bounds, dtype=np.int32).reshape((4,))
        return int(
            self.planner.apply_global_path_perception(
                mark_indices,
                clear_indices,
                float(inflation_radius),
                float(inscribed_radius),
                float(perception_cost),
                float(cost_scaling_factor),
                float(stamp),
                float(persistence),
                clear_center,
                float(clear_radius),
                window_bounds,
            )
        )

    def decay_global_path_perception(self, stamp, persistence):
        return int(self.planner.decay_global_path_perception(float(stamp), float(persistence)))

    def clear_global_path_perception_indices(self, clear_indices):
        clear_indices = np.asarray(clear_indices, dtype=np.int32)
        if clear_indices.size == 0:
            clear_indices = np.zeros((0, 3), dtype=np.int32)
        clear_indices = clear_indices.reshape((-1, 3))
        return int(self.planner.clear_global_path_perception_indices(clear_indices))

    def clear_global_path_perception(self):
        self.planner.clear_global_path_perception()

    def global_path_perception_cell_count(self):
        return int(self.planner.get_global_path_perception_cell_count())

    def set_global_path_perception_enabled(self, enabled):
        self.planner.set_global_path_perception_enabled(bool(enabled))

    def has_lethal_global_path_perception(self, points_xy, layers):
        points_xy = np.asarray(points_xy, dtype=np.float32)
        layers = np.asarray(layers, dtype=np.int32).reshape((-1, 1))
        if points_xy.size == 0:
            return False
        points_xy = points_xy.reshape((-1, 2))
        if len(points_xy) != len(layers):
            raise ValueError(
                f"Dynamic collision query size mismatch: points={len(points_xy)}, "
                f"layers={len(layers)}"
            )
        path_cells = self.points2rowcol(points_xy)
        indices = np.concatenate([layers, path_cells], axis=1)
        return bool(self.planner.has_lethal_global_path_perception(indices))

    def points2rowcol(self, points_xy):
        points_xy = np.asarray(points_xy, dtype=np.float32)
        if points_xy.size == 0:
            return np.zeros((0, 2), dtype=np.int32)
        points_xy = points_xy.reshape((-1, 2))
        idx = np.rint((points_xy - self.center[:2]) / self.resolution).astype(np.int32)
        idx += self.offset
        return idx.astype(np.int32, copy=False)

    def build_global_path_perception_mark_indices(
        self,
        mark_cells,
        current_layer,
        robot_height,
        skip_static_obstacles,
        static_skip_cost,
        layer_height_tolerance,
        mark_all_layers,
    ):
        mark_cells = np.asarray(mark_cells, dtype=np.int32)
        if mark_cells.size == 0:
            mark_cells = np.zeros((0, 2), dtype=np.int32)
        mark_cells = mark_cells.reshape((-1, 2))
        return self.planner.build_global_path_perception_mark_indices(
            mark_cells,
            int(current_layer),
            float(robot_height),
            bool(skip_static_obstacles),
            float(static_skip_cost),
            float(layer_height_tolerance),
            bool(mark_all_layers),
        )

    def build_global_path_perception_clear_indices(
        self,
        origin_cell,
        endpoint_cells,
        current_layer,
        robot_height,
        layer_height_tolerance,
        mark_all_layers,
    ):
        origin_cell = np.asarray(origin_cell, dtype=np.int32).reshape((2,))
        endpoint_cells = np.asarray(endpoint_cells, dtype=np.int32)
        if endpoint_cells.size == 0:
            endpoint_cells = np.zeros((0, 3), dtype=np.int32)
        endpoint_cells = endpoint_cells.reshape((-1, 3))
        return self.planner.build_global_path_perception_clear_indices(
            origin_cell,
            endpoint_cells,
            int(current_layer),
            float(robot_height),
            float(layer_height_tolerance),
            bool(mark_all_layers),
        )



    def _precompute_grid_xy(self):
        """预计算每个网格点对应的物理XY坐标（匹配map的center和resolution）"""
        # 生成网格索引
        grid_h, grid_w = self.map_dim[0], self.map_dim[1]
        y_grid_idx, x_grid_idx = np.meshgrid(np.arange(grid_w), np.arange(grid_h))
        # 转换为物理坐标（反向复用pos2idx的逻辑）
        x_grid = (x_grid_idx - self.offset[0]) * self.resolution + self.center[0]
        y_grid = (y_grid_idx - self.offset[1]) * self.resolution + self.center[1]
        # 存储为 [grid_h, grid_w, 2] （x,y）
        self.grid_xy_coords = np.stack([x_grid, y_grid], axis=-1)

    def get_layer_height_by_xy(self, layer_idx, x, y):
        """
        输入layer索引和物理XY坐标，返回该layer下此XY对应的真实高度
        :param layer_idx: 分层索引（0~n_slice-1）
        :param x: 物理X坐标
        :param y: 物理Y坐标
        :return: 插值后的高度值（无有效值返回-100）
        """
        # 边界检查
        if layer_idx < 0 or layer_idx >= self.n_slice:
            return -100.0

        elev_grid = self.layer_elev_grids[layer_idx]
        xy_coords = self.grid_xy_coords.reshape(-1, 2)  
        elev_vals = elev_grid.reshape(-1)              

        valid_mask = elev_vals > -99.0
        valid_xy = xy_coords[valid_mask]
        valid_elev = elev_vals[valid_mask]

        if len(valid_xy) == 0:
            return -100.0

        query_height = griddata(valid_xy, valid_elev, (x, y), method='nearest', fill_value=-100.0)
        return float(query_height)
    

    

    def match_best_layer(self, x, y, target_height):
        """
        迭代所有layer，找到与目标高度最匹配的layer（核心方法）
        :param x: 物理X坐标
        :param y: 物理Y坐标
        :param target_height: 待匹配的地面高度
        :return: 最佳匹配的layer索引
        """
        best_layer = None
        best_key = None
        grid_idx = self.pos2idx(np.array([x, y])).astype(int)

        for layer_idx in range(self.n_slice):
            height, width = self.tomogram[0][layer_idx].shape
            row = np.clip(grid_idx[1], 0, height - 1)
            col = np.clip(grid_idx[0], 0, width - 1)
            exact_height = float(self.layer_elev_grids[layer_idx][row][col])
            if exact_height <= -99.0 or not np.isfinite(exact_height):
                continue
            diff = abs(exact_height - target_height)
            layer_cost = float(self.tomogram[0][layer_idx][row][col])
            if layer_cost == 0:
                continue

            blocked = layer_cost >= self.a_star_cost_threshold
            in_height_band = diff <= self.layer_match_height_tolerance
            # Layer identity is primarily geometric. A blocked cell on the
            # robot's physical floor is still a better start layer than a
            # low-cost cell several floors away. Cost only breaks ties between
            # candidates at the same height.
            candidate = (
                not in_height_band,
                diff,
                blocked,
                layer_cost,
                layer_idx,
            )
            if best_key is None or candidate < best_key:
                best_key = candidate
                best_layer = layer_idx

        if best_layer is not None:
            return best_layer
        return 0

    def get_layer_cost_info(self, layer_idx, x, y):
        grid_idx = self.pos2idx(np.array([x, y])).astype(int)
        dim_x, dim_y = self.tomogram[0][layer_idx].shape
        row = int(np.clip(grid_idx[1], 0, dim_x - 1))
        col = int(np.clip(grid_idx[0], 0, dim_y - 1))
        return {
            "layer": int(layer_idx),
            "row": row,
            "col": col,
            "cost": float(self.tomogram[0][layer_idx][row][col]),
            "height": float(self.layer_elev_grids[layer_idx][row][col]),
        }

    def resolve_layer(self, x, y, target_height, layer_hint=None):
        if layer_hint is None:
            return self.match_best_layer(x, y, target_height)
        layer = int(layer_hint)
        if layer < 0 or layer >= self.n_slice:
            raise ValueError(
                f"Layer hint {layer} is outside valid range [0, {self.n_slice - 1}]"
            )
        return layer
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        self.planner.init_map(
            self.a_star_cost_threshold, self.safe_cost_margin,
            self.resolution, self.n_slice, self.step_cost_weight,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )
        self.planner.set_reference_height(float(self.path_ground_offset))



    def plan(
        self,
        start_pos,
        end_pos,
        use_dynamic=True,
        search_bounds=None,
        goal_heading=None,
        start_layer_hint=None,
        end_layer_hint=None,
        start_height_hint=None,
    ):
        self.last_traj_layers = None
        start_match_height = (
            float(start_pos[2])
            if start_height_hint is None
            else float(start_height_hint)
        )
        start_layer = self.resolve_layer(
            start_pos[0],
            start_pos[1],
            start_match_height,
            layer_hint=start_layer_hint,
        )
        end_layer = self.resolve_layer(
            end_pos[0],
            end_pos[1],
            end_pos[2],
            layer_hint=end_layer_hint,
        )
        self.start_idx[1:] = self.pos2idx(start_pos[:2])
        self.end_idx[1:] = self.pos2idx(end_pos[:2])
        self.start_idx[0] = start_layer
        self.end_idx[0] = end_layer
        self.last_plan_info = {
            "start": self.get_layer_cost_info(start_layer, start_pos[0], start_pos[1]),
            "goal": self.get_layer_cost_info(end_layer, end_pos[0], end_pos[1]),
            "a_star_cost_threshold": float(self.a_star_cost_threshold),
            "safe_cost_margin": float(self.safe_cost_margin),
            "step_cost_weight": float(self.step_cost_weight),
            "start_layer_hint": None if start_layer_hint is None else int(start_layer_hint),
            "goal_layer_hint": None if end_layer_hint is None else int(end_layer_hint),
            "start_match_height": start_match_height,
        }
    

        self.planner.set_global_path_perception_enabled(bool(use_dynamic))
        if search_bounds is None:
            self.planner.clear_search_bounds()
        else:
            bounds = np.asarray(search_bounds, dtype=np.int32).reshape((4,))
            self.planner.set_search_bounds(bounds)
        try:
            # Optimizer x/y axes are transposed relative to map x/y:
            # internal_x = map_y and internal_y = map_x.
            heading = (
                float("nan")
                if goal_heading is None
                else 0.5 * np.pi - float(goal_heading)
            )
            plan_success = self.planner.plan(
                self.start_idx,
                self.end_idx,
                True,
                heading,
            )
        finally:
            self.planner.clear_search_bounds()
        if not plan_success:
            return None
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = np.asarray(optimizer.get_layers(), dtype=np.float32).reshape((-1,))
        heights = optimizer.get_heights()
        # print("heights origin:", heights)


        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)
        if goal_heading is not None and len(traj_3d) > 0:
            traj_3d[-1, :3] = np.asarray(end_pos[:3], dtype=traj_3d.dtype)

        if use_dynamic and len(traj_3d) > 0:
            sampled_points = [traj_3d[0]]
            sampled_layers = [layers[0]]
            sample_spacing = max(0.01, 0.5 * self.resolution)
            for index in range(1, len(traj_3d)):
                segment_length = np.linalg.norm(
                    traj_3d[index, :2] - traj_3d[index - 1, :2]
                )
                sample_count = max(1, int(np.ceil(segment_length / sample_spacing)))
                for sample_index in range(1, sample_count + 1):
                    ratio = float(sample_index) / float(sample_count)
                    sampled_points.append(
                        traj_3d[index - 1] * (1.0 - ratio) + traj_3d[index] * ratio
                    )
                    sampled_layers.append(
                        layers[index - 1] * (1.0 - ratio) + layers[index] * ratio
                    )
            sampled_points = np.asarray(sampled_points, dtype=np.float32)
            path_cells = self.points2rowcol(sampled_points[:, :2])
            path_layers = np.rint(sampled_layers).astype(np.int32).reshape((-1, 1))
            dynamic_indices = np.concatenate([path_layers, path_cells], axis=1)
            if self.planner.has_lethal_global_path_perception(dynamic_indices):
                self.last_plan_info["dynamic_collision_rejected"] = True
                return None

        self.last_traj_layers = np.rint(layers).astype(np.int32)
        return traj_3d
    

    def pos2idx(self, pos):
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.float32)
        return idx
