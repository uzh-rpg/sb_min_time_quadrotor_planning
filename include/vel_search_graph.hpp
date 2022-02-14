#pragma once
#include <vector>

#include "base_map.hpp"
#include "dijkstra.hpp"
#include "drone.hpp"
#include "esdf_map.hpp"
#include "point_speed3d.hpp"

struct TrajectoryPoint {
  TrajectoryPoint() {}
  TrajectoryPoint(Vector<3> position_, double angle_deg_, bool is_gate_,
                  int target_gate_id_, int topology_path_id_,
                  double portion_between_gates_) {
    position = position_;
    angle_deg = angle_deg_;
    is_gate = is_gate_;
    target_gate_id = target_gate_id_;
    topology_path_id = topology_path_id_;
    portion_between_gates = portion_between_gates_;
  }
  Vector<3> position;
  Vector<3> velocity;
  double angle_deg;
  bool is_gate = false;
  int target_gate_id;
  int topology_path_id;
  double portion_between_gates;
};

struct TrajectoryState {
  TrajectoryState() {
    state.setZero();
    is_gate = false;
  }
  TrajectoryState(DroneState state_, bool gate_ = false) {
    state = state_;
    is_gate = gate_;
  }
  DroneState state;
  bool is_gate = false;
};

struct VelocitySearchResult {
  double time;
  std::vector<Vector<3>> found_gates_speeds;
  std::vector<TrMaxAcc3D> trajectories;
  std::vector<TrajectoryPoint> gates_waypoints;
  bool operator()(VelocitySearchResult& a, VelocitySearchResult& b) {
    return a.time > b.time;
  }
};

struct VelocitySearchCollision {
  VelocitySearchCollision() {
    collision_free = true;
    position = Vector<3>::Constant(NAN);
    trajectory_index = -1;
  }
  bool collision_free;
  Vector<3> position;
  int trajectory_index;
};

class VelSearchGraph {
 public:
  VelSearchGraph(const Drone* drone, std::shared_ptr<BaseMap> map,
                 const bool check_collisions,
                 const double collision_distance_check,
                 const double min_clearance, const double max_yaw_pitch_ang,
                 const double precision_yaw_pitch_ang,
                 const double yaw_pitch_cone_angle_boundary,
                 const double min_velocity_size_boundary,
                 const double min_velocity_size, const double max_velocity_size,
                 const double precision_velocity_size,
                 const std::string output_folder);


  VelocitySearchResult find_velocities_in_positions(
    const std::vector<TrajectoryPoint>& gates_waypoints,
    const bool check_collisions_during_search, const bool end_free);
  void save_track_trajectory(const std::vector<TrMaxAcc3D>& trajectories,
                             const double trajectories_time,
                             std::string filename);
  void save_track_trajectory_equidistant(
    const std::vector<TrMaxAcc3D>& trajectories, const double trajectories_time,
    std::string filename);

  static VelocitySearchCollision isCollisionFree(const TrMaxAcc3D& trajectory);
  static VelocitySearchCollision isCollisionFree(
    const VelocitySearchResult& trajectory);
  static std::vector<Vector<3>> sampleTrajectory(const TrMaxAcc3D& trajectory,
                                                 const double ds_desired);

 private:
  static std::shared_ptr<BaseMap> map_;
  static double collision_distance_check_;
  static double min_clearance_;
  static bool check_collisions_;

  // std::vector<std::vector<std::pair<int, double>>> shortest_samples_times_;
  // const std::vector<DroneState> gates_;
  // const std::vector<double> gates_orientations_;
  // const DroneState start_;
  // const DroneState end_;
  const Drone* drone_;
  const double max_yaw_pitch_ang_;
  const double precision_yaw_pitch_ang_;
  const double max_velocity_size_;
  const double min_velocity_size_;
  const double min_velocity_size_boundary_;
  const double yaw_pitch_cone_angle_boundary_;
  const double precision_velocity_size_;

  const std::string output_folder_;
};