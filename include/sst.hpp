/*
 * sst.hpp
 *
 *  Created on: Jan 21, 2021
 *      Author: Robert Penicka
 */

#pragma once


#include <cfloat>
#include <unordered_map>

#include "drone.hpp"
#include "esdf_map.hpp"
#include "motion_primitive.hpp"
#include "point_speed3d.hpp"
#include "prm.hpp"
#include "timer.hpp"
#include "topological_prm.hpp"
#include "tree_node.hpp"
#include "vel_search_graph.hpp"

std::vector<std::vector<DroneState>> get_samples();
std::vector<Command> get_references();

using index_type = long unsigned int;
using tdata_type = TreeNode<DroneState>;
using maptype_pair = std::pair<index_type, tdata_type*>;


struct Witness {
  Witness() {
    id = 0;
    state = NULL;
    representative = NULL;
  }
  bool operator==(const Witness& otn) const {
    if (this->id == otn.id) {
      return true;
    } else {
      return false;
    }
  }

  struct HashFunction {
    size_t operator()(const Witness& tn) const {
      return std::hash<int>()(tn.id);
    }
  };
  index_type id;
  TreeNode<DroneState>* state;
  TreeNode<DroneState>* representative;
};

struct GateTreePart {
  flann::Index<DroneDistance<float>>* flann_index = NULL;
  flann::Index<DroneDistance<float>>* flann_index_witness = NULL;
  flann::Index<flann::L2_3D<float>>* flann_index_reference_pmm = NULL;

  std::unordered_map<index_type, tdata_type*> open_list;
  std::unordered_map<index_type, tdata_type*> close_list;

  std::vector<tdata_type*> nodes;
  std::vector<Witness*> witnesses;
};

struct LogEntry {
  int iter;
  int max_reached_gate;


  double min_dist_goal;  // distance to goal
};

class SST {
 public:
  SST(const YAML::Node& planner_config, const YAML::Node& drone_config);
  void parse_cmd_args(int argc, char** argv);
  void iterate();
  void best_near(const DroneState& random_state, const double dv,
                 const unsigned int gate_index, double* found_distance,
                 long unsigned int* node_index);
  void nearest_witness(const DroneState& state, const unsigned int gate_index,
                       double* found_distance, long int* node_index);
  void nearest_reference(const DroneState& state, const unsigned int gate_index,
                         double* found_distance,
                         long unsigned int* found_index);
  index_type add_witness(TreeNode<DroneState>* new_node,
                         const unsigned int gate_index,
                         const bool newly_reached_gate);
  std::vector<DroneState> simulate_trajectory(
    const std::vector<TreeNode<DroneState>*>& trajectory,
    bool command_in_node_before = false);
  std::tuple<std::vector<DroneState>, std::vector<std::vector<DroneState>>>
  simulate_expand(const DroneState& from_state,
                  const std::vector<Command>& references, const double max_time,
                  const bool goal_bias);
  std::vector<Command> get_commands_from_node(TreeNode<DroneState>* node);
  std::vector<std::vector<DroneState>> get_samples_from_node(
    TreeNode<DroneState>* node);
  void append_rest_of_references(std::vector<Command>& commands_part,
                                 std::vector<Command>& reference);
  bool is_collisions_between(const DroneState& from, const DroneState& to,
                             const std::vector<DroneState>& states_between);
  void test_hower(DroneState start);
  void signal(int sig);
  DroneState sample_araund(
    const std::vector<std::vector<DroneState>>& references_samples,
    const unsigned int gate, const bool randomize, const bool goal_bias);
  unsigned int equalize_openlist_size_random_gate();
  void check_goal_distances(TreeNode<DroneState>* new_node,
                            const int current_gate_idx);
  void set_best_node(TreeNode<DroneState>* new_node,
                     const int current_gate_idx);
  Scalar get_pointmass_time_to_gate(const DroneState& from,
                                    const DroneState& to, const Scalar amax);
  void inform_progress();

 private:
  std::vector<std::vector<path_with_length<Vector<3>>>>
  find_geometrical_paths();
  std::tuple<Primitive, std::vector<Scalar>, std::vector<std::tuple<int, int>>,
             std::vector<std::tuple<int, int>>>
  join_primitives(std::vector<Primitive>& prims,
                  std::vector<TrajectoryState>& gates_waypoints_end);
  std::pair<Vector<3>, double> getPositionInPathPart(
    const path_with_length<Vector<3>> path_to_snap, const double portion);
  std::pair<Vector<3>, double> getPositionInPathClosest(
    const path_with_length<Vector<3>> path, Vector<3> position);
  std::vector<path_with_length<Vector<3>>> splitPathPart(
    const path_with_length<Vector<3>> path, const double portion);
  // void test_splitting_path(path_with_length<Eigen::Vector3d>
  // paths_between_gate,
  //                          TrMaxAcc3D trajectory);

  const YAML::Node planner_config_;
  const YAML::Node drone_config_;
  TreeNode<DroneState>* root;
  int iter;
  int iter_last_impro;
  Timer comp_timer;
  Drone* drone;
  DroneState start;
  DroneState end;
  bool enforce_end_velocity;
  bool equalize_openlist_sizes;
  int maximally_reached_gate;
  double desired_dt;
  double goal_bias;
  double max_expand_time;
  double end_vel_distance_tolerance;
  double min_near_t{DBL_MAX};
  double randomize_p_around;
  double randomize_v_around;
  double randomize_w_around;
  double randomize_q_angle_around;
  double scale_randomization_goal_bias;
  int max_num_iterations;
  int max_num_iterations_wo_impr;

  double reaching_gate_time_ratio_to_pmm;

  double desired_num_reference_samples;
  double rotation_sample_mult_ratio;
  double rand_rotate_rotation_vector_ang;
  double rand_rotate_rotation_vector_ang_goal_bias;

  double bias_start_from_previous_gate;
  double bias_use_reference_reach_gate_commands;

  std::unordered_map<int, int> test;
  std::vector<Scalar> pmm_cumulative_times;

  double ref_time_deviation;
  double ref_time_deviation_goal_bias;

  // distance calc scale
  Scalar pos_scale;
  Scalar att_scale;
  Scalar vel_scale;
  Scalar omega_scale;

  double scale_tree_expansion_break_lowerbound;

  // distance where to search for the best node around the
  // random created
  double dv;
  // distance considered for creation new witness
  double ds;

  std::string name;

  double pos_tolerance_diameter, pos_tolerance_radius;

  std::vector<Vector<3>> gates_with_start_end_poses;
  std::vector<std::vector<double>> gates_poses;
  std::vector<DroneState> gates;
  std::vector<double> gates_orientations;  // just for now
  int num_gates;
  std::vector<Scalar> primitive_cum_times;

  std::vector<std::pair<std::string, int>> motor_primitive_priorities;
  std::vector<int> motor_primitive_priorities_cum;
  std::unordered_map<std::string, Vector<4>> motor_primitives;

  bool continue_iterate;
  int signal_received;

  // flann
  std::vector<std::vector<float>> flann_dists;
  std::vector<std::vector<long unsigned int>> flann_indices;


  std::vector<GateTreePart> tree_parts;
  std::vector<std::vector<TreeNode<DroneState>*>> gate_nodes;
  // std::vector<std::vector<std::vector<Command>>> gate_nodes_commands;

  TreeNode<DroneState>* best_node{NULL};

  flann::Matrix<float> search_state;
  std::vector<Command> best_node_commands;
  std::vector<std::vector<DroneState>> best_node_samples;

  // map data
  std::string map_type;
  std::string map_file;
  std::shared_ptr<BaseMap> map;
  double collision_distance_check_;
  std::shared_ptr<PRM<Vector<3>>> prm;

  double min_clearance_;
  bool check_collisions_;

  // for saving logging data
  void log_results();
  std::string logfile;
  std::string output_folder{"./"};
  int goal_reached_iter{-1};
  long goal_reached_time_ms{-1};
  long total_calc_time_ms{-1};
  int total_iter{-1};
  Scalar goal_reached_quality{-1};

  double max_dist_from_reference;
  // flann::Index<flann::L2_3D<float>>* flann_index_reference;
  float nearest_reference_data[3];
  std::vector<Command> reference_commands;

  // for saving the best distances to goal
  double min_next_gate_distance{DBL_MAX};
  long unsigned int min_next_gate_reference_id{0};
  double end_distance_pos_min{DBL_MAX};
  double end_distance_vel_min{DBL_MAX};
  TreeNode<DroneState>* end_distance_vel_min_node{NULL};
  TreeNode<DroneState>* end_distance_pos_min_node{NULL};
  TreeNode<DroneState>* end_distance_min_time_node{NULL};
  double end_distance_min_time_dist_pos{DBL_MAX};
  double end_distance_min_time_dist_vel{DBL_MAX};
};
