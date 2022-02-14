/*
 * sst.cpp
 *
 *  Created on: Jan 21, 2021
 *      Author: Robert Penicka
 */

/*
TODO: - informative steer silumation
      - scale the distance based on time to reach state
      - guiding along exisitng estimate
      - guiding as time sample in existing path and then the sphere
      - admissible is only lower-bound guess!!!
      - i.e. velocity can be changed by turning around + acc to direction
      - randomize the endpoint to bias better
      - change reference commands based on current solution
      - more samples during acc!!!!!
      - do not add samples behind the next gate?
      - perturbate the rotation axis
      - set the reference command sequence according to the one found
      - use gausian distribution instead of the uniform
      - more samples per simulated inputs
      - print distance to the next gate
      - test rrt2
      - test free final velocity
      - goal bias in form of using the referece commands as the nearest to goal
      - shortening has still some issues

      - consider Default
      - consider using maximization of progress over point-mass model

      - use receiding horizon principle such that the path with shortest time
reaching the gate is used to focuse sampling in previous gates

      - graph of min max time for different acc initialization
      - add to the velocity model also the estimated time for changing the acc
vector



*/

#include "sst.hpp"

#include <float.h>
#include <math.h>
#include <tclap/CmdLine.h>
#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <functional>
#include <queue>
#include <unordered_map>

SST::SST(const YAML::Node &planner_config, const YAML::Node &drone_config)
    : planner_config_(planner_config), drone_config_(drone_config) {
  //: drone(0.85, 0.15, 0.15)
  drone = new Drone(drone_config);
  std::vector<std::vector<double>> sampling_pos_min_max; //[x,y,z][min,max]
  parseArrayParam(planner_config, "sampling_pos_min_max", sampling_pos_min_max);
  std::vector<std::vector<double>> sampling_vel_min_max; //[x,y,z][min,max]
  parseArrayParam(planner_config, "sampling_vel_min_max", sampling_vel_min_max);
  iter = 0;
  iter_last_impro = 0;
  maximally_reached_gate = 0;
  root = NULL;

  map_type = loadParam<std::string>(planner_config, "map_type");
  map_file = loadParam<std::string>(planner_config, "map");
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");
  check_collisions_ = loadParam<bool>(planner_config_, "check_collisions");
  collision_distance_check_ =
      loadParam<double>(planner_config_, "collision_distance_check");

  continue_iterate = true;
  signal_received = -1;
  desired_dt = loadParam<double>(planner_config, "desired_dt");
  goal_bias = loadParam<double>(planner_config, "goal_bias");
  max_expand_time = loadParam<double>(planner_config, "max_expand_time");
  randomize_p_around = loadParam<double>(planner_config, "randomize_p_around");
  randomize_v_around = loadParam<double>(planner_config, "randomize_v_around");
  randomize_w_around = loadParam<double>(planner_config, "randomize_w_around");
  randomize_q_angle_around =
      loadParam<double>(planner_config, "randomize_q_angle_around");
  scale_randomization_goal_bias =
      loadParam<double>(planner_config, "scale_randomization_goal_bias");
  end_vel_distance_tolerance =
      loadParam<double>(planner_config, "end_vel_distance_tolerance");

  reaching_gate_time_ratio_to_pmm =
      loadParam<double>(planner_config, "reaching_gate_time_ratio_to_pmm");

  enforce_end_velocity =
      loadParam<bool>(planner_config, "enforce_end_velocity");
  if (!enforce_end_velocity) {
    end_vel_distance_tolerance = DBL_MAX;
    INFO_YELLOW(
        "allowing infinite end velocity difference for !enforce_end_velocity")
  }

  max_num_iterations = loadParam<int>(planner_config, "max_num_iterations");
  max_num_iterations_wo_impr =
      loadParam<int>(planner_config, "max_num_iterations_wo_impr");
  logfile = loadParam<std::string>(planner_config, "logfile");
  equalize_openlist_sizes =
      loadParam<bool>(planner_config, "equalize_openlist_sizes");

  ref_time_deviation = loadParam<double>(planner_config, "ref_time_deviation");

  scale_tree_expansion_break_lowerbound = loadParam<double>(
      planner_config, "scale_tree_expansion_break_lowerbound");

  desired_num_reference_samples =
      loadParam<double>(planner_config, "desired_num_reference_samples");
  max_dist_from_reference =
      loadParam<double>(planner_config, "max_dist_from_reference");
  rotation_sample_mult_ratio =
      loadParam<double>(planner_config, "rotation_sample_mult_ratio");
  rand_rotate_rotation_vector_ang =
      loadParam<double>(planner_config, "rand_rotate_rotation_vector_ang");
  rand_rotate_rotation_vector_ang_goal_bias = loadParam<double>(
      planner_config, "rand_rotate_rotation_vector_ang_goal_bias");
  ref_time_deviation_goal_bias =
      loadParam<double>(planner_config, "ref_time_deviation_goal_bias");

  bias_start_from_previous_gate =
      loadParam<double>(planner_config, "bias_start_from_previous_gate");
  bias_use_reference_reach_gate_commands = loadParam<double>(
      planner_config, "bias_use_reference_reach_gate_commands");

  parseArrayParam(planner_config, "gates_orientations", gates_orientations);
  for (size_t gatei = 0; gatei < gates_orientations.size(); gatei++) {
    AngleAxis ang_ax((M_PI * gates_orientations[gatei] / 180.0),
                     Vector<3>::UnitZ());
    Quaternion q(ang_ax);
    INFO("gate " << 0 << " angle " << gates_orientations[gatei]
                 << " quaternion " << q.w() << " " << q.x() << " " << q.y()
                 << " " << q.z() << " ")
  }
  // exit(1);

  parseArrayParam(planner_config, "gates", gates_poses);

  // gate_nodes_commands.resize(num_gates + 1);

  dv = loadParam<double>(planner_config, "dv");
  ds = loadParam<double>(planner_config, "ds");

  pos_tolerance_diameter =
      loadParam<double>(planner_config, "pos_tolerance_diameter");
  pos_tolerance_radius = pos_tolerance_diameter / 2.0;
  // setting up the min-max values for position/velocity/angrates when creating
  // random samples

  // setting up the distance mixture scales
  pos_scale = loadParam<Scalar>(planner_config, "pos_scale");
  DroneDistance<float>::pos_scale_square = pos_scale * pos_scale;
  att_scale = loadParam<Scalar>(planner_config, "att_scale");
  DroneDistance<float>::att_scale_square = att_scale * att_scale;
  vel_scale = loadParam<Scalar>(planner_config, "vel_scale");
  DroneDistance<float>::vel_scale_square = vel_scale * vel_scale;
  omega_scale = loadParam<Scalar>(planner_config, "omega_scale");
  DroneDistance<float>::omega_scale_square = omega_scale * omega_scale;

  // std::vector<std::vector<double>> sampling_vel_min_max;  //[x,y,z][min,max]
  // parseArrayParam(planner_config, "sampling_vel_min_max",
  // sampling_vel_min_max);
  start.setZero();
  if (planner_config["start"] && planner_config["end"]) {
    // define start pos
    if (planner_config["start"]["position"]) {
      std::vector<double> start_pos;
      parseArrayParam(planner_config["start"], "position", start_pos);
      start.p(0) = start_pos[0];
      start.p(1) = start_pos[1];
      start.p(2) = start_pos[2];
    } else {
      INFO_RED("you must specify start position");
      exit(1);
    }

    if (planner_config["start"]["velocity"]) {
      std::vector<double> start_vel;
      parseArrayParam(planner_config["start"], "velocity", start_vel);
      start.v(0) = start_vel[0];
      start.v(1) = start_vel[1];
      start.v(2) = start_vel[2];
    }

    // define end
    end.setZero();
    if (planner_config["end"]["position"]) {
      std::vector<double> end_pos;
      parseArrayParam(planner_config["end"], "position", end_pos);
      end.p(0) = end_pos[0];
      end.p(1) = end_pos[1];
      end.p(2) = end_pos[2];
    } else {
      INFO_RED("you must specify end position");
      exit(1);
    }

    if (planner_config["end"]["velocity"]) {
      std::vector<double> end_vel;
      parseArrayParam(planner_config["end"], "velocity", end_vel);
      end.v(0) = end_vel[0];
      end.v(1) = end_vel[1];
      end.v(2) = end_vel[2];
    }
  } else {
    INFO_RED("you must specify start and end");
    exit(1);
  }

  search_state = flann::Matrix<float>(new float[1 * DroneState::SIZE_NN], 1,
                                      DroneState::SIZE_NN);

  // prm = std::make_shared<PRM<Vector<3>>>(planner_config, map);
}

void SST::parse_cmd_args(int argc, char **argv) {
  // overwrite the variables from the yaml with the one from command line

  try {
    TCLAP::CmdLine cmd("SST program", ' ', "0.0");

    TCLAP::ValueArg<double> dvArg(
        "v", "dv",
        "distance where to search for the best node around the random created",
        false, dv, "double", cmd);
    TCLAP::ValueArg<double> dsArg(
        "s", "ds", "distance considered for creation new witness", false, ds,
        "double", cmd);

    TCLAP::ValueArg<std::string> nameArg("", "name", "name", false,
                                         std::string("no_name"), "string", cmd);

    TCLAP::ValueArg<std::string> logfileArg("", "logfile", "logfile", false,
                                            logfile, "string", cmd);

    TCLAP::ValueArg<std::string> output_folderArg("", "output_folder",
                                                  "output_folder", false,
                                                  output_folder, "string", cmd);

    TCLAP::ValueArg<std::string> mapArg("", "map", "map", false, map_file,
                                        "string", cmd);
    TCLAP::ValueArg<bool> check_collisionsArg("", "check_collisions",
                                              "check_collisions", false,
                                              check_collisions_, "bool", cmd);

    TCLAP::ValueArg<LoadVector<double>> start_p_Arg("", "start_p", "start_p",
                                                    false, LoadVector<double>(),
                                                    "Vector<3>", cmd);
    TCLAP::ValueArg<LoadVector<double>> goal_p_Arg(
        "", "goal_p", "goal_p", false, LoadVector<double>(), "Vector<3>", cmd);
    TCLAP::ValueArg<LoadVector2D<double, 3>> gates_p_Arg(
        "", "gates_p", "gates_p", false, LoadVector2D<double, 3>(),
        "Vector<N*3>", cmd);

    TCLAP::ValueArg<LoadVector<double>> gates_orientations_Arg(
        "", "gate_orientation", "gate_orientation", false, LoadVector<double>(),
        "LoadVector<double>", cmd);

    TCLAP::ValueArg<int> max_q_iterations_wo_imprArg(
        "", "max_num_iterations_wo_impor", "max_num_iterations_wo_impor", false,
        max_num_iterations_wo_impr, "int", cmd);

    cmd.parse(argc, argv);

    name = nameArg.getValue();
    INFO("loaded name " << name)
    output_folder = output_folderArg.getValue();
    map_file = mapArg.getValue();
    logfile = logfileArg.getValue();

    check_collisions_ = check_collisionsArg.getValue();

    INFO("loaded logfile " << logfile)
    INFO("map_file " << map_file)
    INFO("creating output_folder " << output_folder)

    std::filesystem::create_directories(output_folder);

    max_num_iterations_wo_impr = max_q_iterations_wo_imprArg.getValue();
    INFO("loaded max_num_iterations_wo_imporArg" << max_num_iterations_wo_impr)

    dv = dvArg.getValue();
    INFO("loaded cmd arg dv " << dv);
    ds = dsArg.getValue();
    INFO("loaded cmd arg ds " << ds);
    if (start_p_Arg.isSet()) {
      LoadVector<double> start_cmd = start_p_Arg.getValue();
      Vector<3> new_start(start_cmd.vector[0], start_cmd.vector[1],
                          start_cmd.vector[2]);
      INFO_CYAN("changing start from " << start.p.transpose() << " to "
                                       << new_start.transpose())
      start.p = new_start;
      // exit(1);
    }

    if (goal_p_Arg.isSet()) {
      LoadVector<double> goal_cmd = goal_p_Arg.getValue();
      Vector<3> new_goal(goal_cmd.vector[0], goal_cmd.vector[1],
                         goal_cmd.vector[2]);
      INFO_CYAN("changing goal from " << start.p.transpose() << " to "
                                      << new_goal.transpose())
      end.p = new_goal;
      // exit(1);
    }

    if (gates_p_Arg.isSet()) {
      LoadVector2D<double, 3> gates_loaded = gates_p_Arg.getValue();

      INFO("new gates set with size " << gates_loaded.vector.size())
      INFO("old gates size " << gates_poses.size())
      for (size_t gi = 0; gi < gates_loaded.vector.size(); gi++) {
        Vector<3> new_gate(gates_loaded.vector[gi][0],
                           gates_loaded.vector[gi][1],
                           gates_loaded.vector[gi][2]);
        INFO("new gate " << gi << " is " << new_gate)
      }

      gates_poses = gates_loaded.vector;
      // exit(1);
    }

    if (gates_orientations_Arg.isSet()) {
      LoadVector<double> orientations_cmd = gates_orientations_Arg.getValue();
      for (size_t goi = 0; goi < orientations_cmd.vector.size(); goi++) {
        INFO("changing gate " << goi << " orientation from "
                              << gates_orientations[goi] << " to "
                              << orientations_cmd.vector[goi])
      }

      this->gates_orientations = orientations_cmd.vector;
    }

    // exit(1);
  } catch (TCLAP::ArgException &e) {
    std::cerr << "cmd args error: " << e.error() << " for arg " << e.argId()
              << std::endl;
    exit(1);
  }
}

DroneState SST::sample_araund(
    const std::vector<std::vector<DroneState>> &references_samples,
    const unsigned int gate, const bool randomize, const bool goal_bias) {
  // INFO("references_samples[gate].size() " <<
  // references_samples[gate].size());
  int index = randIntMinMax(0, references_samples[gate].size() - 1);
  // INFO("index" << index);
  if (gate >= references_samples.size()) {
    INFO("bad gate idnex " << gate)
    INFO_VAR(references_samples.size())
    exit(1);
  }
  if (index >= references_samples[gate].size()) {
    INFO("bad idnex " << index)
    exit(1);
  }
  DroneState state = references_samples[gate][index];
  // INFO("state before random" << state);
  double scale_randomize = 1.0;
  if (goal_bias) {
    scale_randomize = scale_randomization_goal_bias;
  }
  if (randomize) {
    state.p += Vector<3>::Random() * scale_randomize * randomize_p_around;
    state.v += Vector<3>::Random() * scale_randomize * randomize_v_around;
    state.w += Vector<3>::Random() * scale_randomize * randomize_w_around;
    state.w = Vector<3>::Random().cwiseProduct(Vector<3>(
        drone->omega_max_xy, drone->omega_max_xy, drone->omega_max_z));

    const Quaternion current_q(state.qx(0), state.qx(1), state.qx(2),
                               state.qx(3));
    Vector<3> rot_vec = Vector<3>::Random();
    rot_vec(2) = 0;
    rot_vec.normalize();
    const double rand_ang =
        randDoubleMinMax(-scale_randomize * randomize_q_angle_around,
                         scale_randomize * randomize_q_angle_around);
    // INFO_VAR(rand_ang)
    const Quaternion small_deviation_xy_q(cos(rand_ang / 2.0),
                                          rot_vec(0) * sin(rand_ang / 2.0),
                                          rot_vec(1) * sin(rand_ang / 2.0), 0);

    const Quaternion rand_quat_around =
        current_q * small_deviation_xy_q.normalized();

    state.qx(0) = rand_quat_around.w();
    state.qx(1) = rand_quat_around.x();
    state.qx(2) = rand_quat_around.y();
    state.qx(3) = rand_quat_around.z();
  }
  // INFO("state after random" << state);
  return state;
}

/*
 * return random gate number such that the open_list per gate is equalized
 */
unsigned int SST::equalize_openlist_size_random_gate() {
  double num_inverse_count = 0;
  std::vector<double> cum_sum_gate_ol_nodes;
  cum_sum_gate_ol_nodes.resize(maximally_reached_gate + 1);
  // INFO_VAR(maximally_reached_gate)
  for (int gid = 0; gid <= maximally_reached_gate; ++gid) {
    num_inverse_count += 1.0 / ((double)tree_parts[gid].open_list.size());
    cum_sum_gate_ol_nodes[gid] = num_inverse_count;
    // INFO_VAR(tree_parts[gid].open_list.size())
    // INFO("gid " << gid << " open list size " <<
    // tree_parts[gid].open_list.size()
    //            << " cum_sum_inv " << cum_sum_gate_ol_nodes[gid])
  }

  const double open_list_select = randDoubleMinMax(0, num_inverse_count);

  for (int gid = 0; gid <= maximally_reached_gate; ++gid) {
    if (open_list_select < cum_sum_gate_ol_nodes[gid]) {
      // INFO("open_list_select " << open_list_select << " selected " << gid);
      return gid;
    }
  }
  return 0;
}

std::tuple<Primitive, std::vector<Scalar>, std::vector<std::tuple<int, int>>,
           std::vector<std::tuple<int, int>>>
SST::join_primitives(std::vector<Primitive> &prims,
                     std::vector<TrajectoryState> &gates_waypoints_end) {
  INFO("joining primitives")
  std::vector<std::tuple<int, int>> gate_primitive_ids;
  std::vector<std::tuple<int, int>> command_primitive_ids;
  std::vector<Scalar> primitive_cum_times_loc;
  primitive_cum_times_loc.resize(num_gates + 1);
  int gatei = 0;
  double time_sum = 0;
  Primitive pr_all;
  for (int var = 0; var < prims.size(); ++var) {
    INFO("");
    INFO("------------------------------------------------------------")
    INFO("prim " << var);
    INFO("prim time " << prims[var].time);
    INFO("start " << prims[var].rotations.front().get_start_state())
    INFO("end " << prims[var].rotations.back().get_end_state())
    INFO_VAR(prims[var].rotations.size())
    INFO_VAR(prims[var].translations.size())
    // for (size_t i = 0; i < prims[var].translations.size(); i++) {
    //   INFO(prims[var].translations[i])
    //   INFO(prims[var].rotations[i + 1])
    // }
    time_sum += prims[var].time;
    if (gates_waypoints_end[var].is_gate) {
      INFO_VAR(gatei);
      primitive_cum_times_loc[gatei] = time_sum;

      INFO_VAR(primitive_cum_times_loc[gatei]);
      gatei++;
    }
    INFO("");
    INFO("------------------------------------------------------------")
  }

  // pr_all = prims[0];
  // gate_primitive_ids.push_back({0, prims[0].rotations.size() - 1});

  // int num_rot_commands = 0;
  // for (size_t rci = 0; rci < prims[0].rotations.size(); rci++) {
  //   if (prims[0].rotations[rci].rotation.t2 > 0) {
  //     num_rot_commands += 3;
  //     if (rci == prims[0].rotations.size() - 1) {
  //       num_rot_commands -= 1;
  //     }
  //   } else {
  //     num_rot_commands += 2;
  //   }
  // }
  // command_primitive_ids.push_back(
  //   {0, prims[0].translations.size() + num_rot_commands - 1});
  int rotations_between = 0;
  int commands_between = 0;
  int gate_id = 0;
  for (int var = 0; var < prims.size(); ++var) {
    INFO("primitive num rotations " << prims[var].rotations.size() << " trans "
                                    << prims[var].translations.size());
    prims[var].setGateId(gate_id);
    Primitive pr_all_new;
    if (var > 0) {
      pr_all_new = Primitive::connect_primitives(pr_all, prims[var], drone);
    } else {
      pr_all_new = prims[var];
    }

    rotations_between += prims[var].rotations.size(); //- 1;
    if (var > 0) {
      rotations_between -= 1;
    }

    // int num_rot_commands = 0;
    int start_rot_prim = 0;
    if (var > 0) {
      start_rot_prim = 1;
    }
    // for (int rci = start_rot_prim; rci < prims[var].rotations.size(); rci++)
    // {
    //   if (prims[var].rotations[rci].rotation.t2 > 0) {
    //     // ang acc, const speed and ang dec.
    //     num_rot_commands += 3;
    //   } else {
    //     INFO_BLUE("leeeeeees motors")
    //     // exit(1);
    //     num_rot_commands += 2;
    //   }
    // }

    // if (var > 0 && var + 1 < prims.size()) {
    //   num_rot_commands -= 2;  // from the middle ones substrac 2
    // } else if ((var == 0 && var + 1 != prims.size()) ||
    //            (var != 0 && var + 1 == prims.size())) {
    //   num_rot_commands -= 1;  // if start or end, but not both, substrac 1
    // }

    // commands_between += prims[var].translations.size() + num_rot_commands;
    // INFO_RED("commands_between " << commands_between)
    INFO_RED("rotations_between " << rotations_between)
    // INFO_RED("num_rot_commands " << num_rot_commands)

    if (gates_waypoints_end[var].is_gate) {
      if (gate_primitive_ids.size() > 0) {
        int curr_last_id = std::get<1>(gate_primitive_ids.back());
        gate_primitive_ids.push_back(
            {curr_last_id, curr_last_id + rotations_between});
        INFO_CYAN("from " << curr_last_id << " to "
                          << (curr_last_id + rotations_between));
      } else {
        gate_primitive_ids.push_back({0, rotations_between - 1});
        INFO_CYAN("from " << 0 << " to " << (rotations_between - 1));
      }

      gate_id += 1;
      // int curr_last_command_id = 0;
      // if (command_primitive_ids.size() > 0) {
      //   curr_last_command_id = std::get<1>(command_primitive_ids.back());
      // }
      // command_primitive_ids.push_back(
      //   {curr_last_command_id, curr_last_command_id + commands_between - 1});
      rotations_between = 0;
      // commands_between = 0;
    }
    pr_all = pr_all_new;
  }

  pr_all = MotionPrimitive::fix_rotation_continuity(pr_all, drone);
  INFO_VAR(pr_all.rotations.size());

  gate_id = 0;
  commands_between = 0;
  for (size_t rci = 0; rci < pr_all.rotations.size(); rci++) {
    INFO("rci " << rci << " gate id " << pr_all.rotations[rci].gate_idx)

    // the previous rotation was partially in the new gate!!!!!
    if (gate_id != pr_all.rotations[rci].gate_idx) {
      int curr_last_command_id = 0;
      if (command_primitive_ids.size() > 0) {
        curr_last_command_id = std::get<1>(command_primitive_ids.back());
      }
      INFO_RED("commands_between " << commands_between)
      INFO("add command primitive from "
           << curr_last_command_id << " to "
           << (curr_last_command_id + commands_between - 2))
      command_primitive_ids.push_back(
          {curr_last_command_id, curr_last_command_id + commands_between - 2});
      gate_id += 1;
      commands_between = 2;
    }

    if (rci > 0) {
      commands_between += 1; // translation
    }
    if (pr_all.rotations[rci].rotation.t2 > 0) {
      // ang acc, const speed and ang dec.
      commands_between += 3;
    } else {
      INFO_BLUE("leeeeeees motors")
      // exit(1);
      commands_between += 2;
    }
    INFO_VAR(commands_between)
  }

  int curr_last_command_id = 0;
  if (command_primitive_ids.size() > 0) {
    curr_last_command_id = std::get<1>(command_primitive_ids.back());
  }
  INFO("add command primitive from "
       << curr_last_command_id << " to "
       << (curr_last_command_id + commands_between - 1))
  command_primitive_ids.push_back(
      {curr_last_command_id, curr_last_command_id + commands_between - 1});

  return {pr_all, primitive_cum_times_loc, gate_primitive_ids,
          command_primitive_ids};
}

/*
void SST::test_splitting_path(
  path_with_length<Eigen::Vector3d> paths_between_gate, TrMaxAcc3D trajectory)
{ bool has_collision = true; std::vector<path_with_length<Vector<3>>>
path_to_snap;

  while (has_collision) {
    has_collision = false;
    auto [pos_between, angle] = getPositionInPathPart(paths_between_gate,
0.5); std::vector<path_with_length<Vector<3>>> path_splited =
      splitPathPart(paths_between_gate, 0.5);
    path_to_snap.erase(path_to_snap.begin() + gatei);
    path_to_snap.insert(path_to_snap.begin() + gatei, path_splited.begin(),
                        path_splited.end());
    INFO("pos between " << pos_between.transpose() << " angle " << angle)
    TrajectoryPoint point_between(pos_between, false, target_gate_id);

    gates_and_waypoints_positions.insert(
      gates_and_waypoints_positions.begin() + gatei, point_between);
    gates_orientations.insert(gates_orientations.begin() + gatei,
                              180.0 * (angle / M_PI));
    for (size_t gi = 0; gi < gates_and_waypoints_positions.size(); gi++) {
      INFO("gate " << gi << " is "
                   << gates_and_waypoints_positions[gi].position.transpose()
                   << " with angle " << gates_orientations[gi]
                   << " deg , is gate "
                   << gates_and_waypoints_positions[gi].gate)
    }
    std::tie(gate_with_velocities, trajectories) =
      vel_search_graph.find_velocities_in_positions(
        start, end, gates_and_waypoints_positions, gates_orientations,
        !enforce_end_velocity);
  }
}
*/

void SST::iterate() {
  //  start.u(0) = 0;
  //  start.u(1) = 0;
  //  start.u(2) = 0;
  //  start.u(3) = 0;

  for (int var = 0; var < gates_poses.size(); ++var) {
    DroneState gate;
    gate.setZero();
    gate.p(0) = gates_poses[var][0];
    gate.p(1) = gates_poses[var][1];
    gate.p(2) = gates_poses[var][2];

    gates.push_back(gate);
    gates_with_start_end_poses.push_back(gate.p);
  }
  num_gates = gates.size();
  primitive_cum_times.resize(num_gates + 1);
  tree_parts.resize(num_gates + 1);
  gate_nodes.resize(num_gates + 1);

  // finish array of gates in case of changed start end
  gates.push_back(end);
  gates_with_start_end_poses.insert(gates_with_start_end_poses.begin(),
                                    start.p);
  gates_with_start_end_poses.push_back(end.p);

  // load map
  if (map_type == "ESDF") {
    map = std::make_shared<ESDFMap>();
  } else if (map_type == "PC") {
  } else {
    ERROR("map type " << map_type << " not recognized")
    exit(1);
  }
  map->load(map_file);

  INFO("map loaded")

  // count time from here
  comp_timer.start();

  std::vector<std::vector<path_with_length<Vector<3>>>> paths_between_gates =
      TopologicalPRM<Vector<3>>::find_geometrical_paths(
          planner_config_, map, gates_with_start_end_poses, gates_orientations,
          output_folder);

  std::vector<path_with_length<Vector<3>>> shortest;
  double shortest_length = 0;
  shortest.resize(paths_between_gates.size());
  for (size_t i = 0; i < paths_between_gates.size(); ++i) {
    double len_shortest = DBL_MAX;
    for (size_t j = 0; j < paths_between_gates[i].size(); ++j) {
      if (paths_between_gates[i][j].length < len_shortest) {
        len_shortest = paths_between_gates[i][j].length;
        shortest[i] = paths_between_gates[i][j];
      }
    }
    shortest_length += len_shortest;
  }
  INFO("shortest path has length " << shortest_length)
  for (size_t i = 0; i < shortest.size(); i++) {
    INFO("part " << i)
    shortest[i].print();
  }

  // exit(1);

  const double max_yaw_pitch_ang = 15.0;
  const double precision_yaw_pitch_ang = 15.0;
  const double yaw_pitch_cone_angle_boundary = 60.0;

  const double min_velocity_size = 7.0;
  const double min_velocity_size_boundary = 2.0;
  const double max_velocity_size = 17.0;
  const double precision_velocity_size = 5.0;
  VelSearchGraph vel_search_graph(
      drone, map, check_collisions_, collision_distance_check_, min_clearance_,
      max_yaw_pitch_ang, precision_yaw_pitch_ang, yaw_pitch_cone_angle_boundary,
      min_velocity_size_boundary, min_velocity_size, max_velocity_size,
      precision_velocity_size, output_folder);
  // std::vector<Vector<3>> gate_with_velocities;

  std::vector<TrajectoryPoint>
      gates_and_waypoints_positions; //= (gates.begin(), gates.end() - 1);
  gates_and_waypoints_positions.resize(gates.size() + 1); // gates include end

  gates_and_waypoints_positions[0].position = start.p;
  gates_and_waypoints_positions[0].velocity = start.v;
  gates_and_waypoints_positions[0].is_gate = true;
  gates_and_waypoints_positions[0].target_gate_id = 1;
  gates_and_waypoints_positions[0].portion_between_gates = 0;
  gates_and_waypoints_positions[0].topology_path_id = -1;
  gates_and_waypoints_positions[0].angle_deg = gates_orientations[0];

  for (size_t i = 1; i < gates_and_waypoints_positions.size(); i++) {
    gates_and_waypoints_positions[i].position = gates[i - 1].p;
    gates_and_waypoints_positions[i].is_gate = true;
    gates_and_waypoints_positions[i].target_gate_id = i + 1;
    gates_and_waypoints_positions[i].portion_between_gates = 0;
    gates_and_waypoints_positions[i].topology_path_id = -1;
    gates_and_waypoints_positions[i].angle_deg = gates_orientations[i];
  }
  gates_and_waypoints_positions.back().velocity = end.v;

  VelocitySearchResult init_vel = vel_search_graph.find_velocities_in_positions(
      gates_and_waypoints_positions, false, !enforce_end_velocity);

  INFO_VAR(gates_and_waypoints_positions.size());
  INFO_VAR(init_vel.gates_waypoints.size())
  INFO_VAR(init_vel.trajectories.size())

  std::priority_queue<VelocitySearchResult, std::deque<VelocitySearchResult>,
                      VelocitySearchResult>
      pq_velocity_search;
  pq_velocity_search.push(init_vel);

  bool has_collision = true;
  VelocitySearchResult fastest_trajectory;
  while (has_collision) {
    has_collision = false;
    INFO_CYAN("loop still have collision")
    auto pq_velocity_search_copy = pq_velocity_search;
    while (!pq_velocity_search_copy.empty()) {
      INFO("pq " << pq_velocity_search_copy.top().time)
      pq_velocity_search_copy.pop();
    }

    // get the fastest trajectory from the priority queue
    VelocitySearchResult current_fastest = pq_velocity_search.top();

    INFO_BLUE("current_fastest time " << current_fastest.time)
    pq_velocity_search.pop(); // remove it from the priority queue
    VelocitySearchCollision csc = VelSearchGraph::isCollisionFree(
        current_fastest); // check if it has collision
    if (csc.collision_free) {
      INFO_CYAN("found collision free path")
      fastest_trajectory = current_fastest; // iff no collision then use it
      break;
    } else {
      // has collision, therefore add intermidiate point
      has_collision = true;
      INFO_RED("has collision, trajectory_index " << csc.trajectory_index)
      INFO_RED("has collision, in pos " << csc.position.transpose())
      INFO_VAR(current_fastest.gates_waypoints.size())

      TrajectoryPoint point_before_collision =
          current_fastest.gates_waypoints[csc.trajectory_index];
      TrajectoryPoint point_after_collision =
          current_fastest.gates_waypoints[csc.trajectory_index + 1];
      int target_gate_id = point_before_collision.target_gate_id;

      INFO_RED("trajectory with time "
               << current_fastest.time
               << " is not collision free! , collision in "
               << csc.position.transpose() << " for target gate id "
               << target_gate_id)
      INFO("it should be between "
           << current_fastest.gates_waypoints[csc.trajectory_index]
                  .position.transpose()
           << " and "
           << current_fastest.gates_waypoints[csc.trajectory_index + 1]
                  .position.transpose())
      // exit(1);
      bool is_between_gates = false;
      is_between_gates =
          current_fastest.gates_waypoints.size() == 0 ||
          (current_fastest.gates_waypoints[csc.trajectory_index].is_gate &&
           current_fastest.gates_waypoints[csc.trajectory_index + 1].is_gate);

      if (is_between_gates) {
        // if two adjacent gate node, then snap the point to all possible
        // topological paths between the gates
        INFO("is between gates")
        INFO("target_gate_id " << target_gate_id)

        for (int pbgi = 0;
             pbgi < paths_between_gates[target_gate_id - 1].size(); pbgi++) {
          path_with_length<Vector<3>> path_to_snap =
              paths_between_gates[target_gate_id - 1][pbgi];
          // auto [pos_between, angle] = getPositionInPathPart(path_to_snap,
          // 0.5);
          auto [pos_between, angle] = getPositionInPathPart(path_to_snap, 0.5);
          // std::vector<path_with_length<Vector<3>>> path_splited =
          //   splitPathPart(path_to_snap, 0.5);
          VelocitySearchResult current_fastest_copy = current_fastest;
          TrajectoryPoint new_trajectory_point(pos_between,
                                               180.0 * (angle / M_PI), false,
                                               target_gate_id, pbgi, 0.5);
          current_fastest_copy.gates_waypoints.insert(
              current_fastest_copy.gates_waypoints.begin() +
                  csc.trajectory_index + 1,
              new_trajectory_point);

          VelocitySearchResult new_vel_result =
              vel_search_graph.find_velocities_in_positions(
                  current_fastest_copy.gates_waypoints, false,
                  !enforce_end_velocity);
          pq_velocity_search.push(new_vel_result);
          INFO("added new VelocitySearchResult")
          // exit(1);
        }
        // exit(1);

        // INFO("pos between " << pos_between.transpose() << " angle " <<
        // angle) TrajectoryPoint point_between(pos_between, false,
        // target_gate_id);

        // gates_and_waypoints_positions.insert(
        //   gates_and_waypoints_positions.begin() + gatei, point_between);
        // gates_orientations.insert(gates_orientations.begin() + gatei,
        //                           180.0 * (angle / M_PI));
      } else {
        // collision on some of those topological paths, then snap only on
        // that one
        INFO("not between gates")

        INFO_VAR(paths_between_gates[target_gate_id - 1].size())

        double portion_between_gates_before =
            point_before_collision.portion_between_gates;
        double portion_between_gates_after =
            point_after_collision.portion_between_gates;
        int topologi_path_id_before = point_before_collision.topology_path_id;
        int topologi_path_id_after = point_after_collision.topology_path_id;
        INFO_VAR(portion_between_gates_before)
        INFO_VAR(portion_between_gates_after)
        INFO_VAR(topologi_path_id_before)
        INFO_VAR(topologi_path_id_after)
        int topologi_path_id = -1;
        if (topologi_path_id_after >= 0) {
          topologi_path_id = topologi_path_id_after;
        } else if (topologi_path_id_before >= 0) {
          topologi_path_id = topologi_path_id_before;
        } else {
          INFO_RED("bad topologi_path_id")
          exit(1);
        }
        INFO_VAR(target_gate_id)
        path_with_length<Vector<3>> path_to_snap =
            paths_between_gates[target_gate_id - 1][topologi_path_id];

        if (portion_between_gates_after == 0) { // next is gate
          portion_between_gates_after = 1.0;
        }
        double new_portion_between_gates =
            (portion_between_gates_after + portion_between_gates_before) / 2.0;

        INFO_VAR(new_portion_between_gates)

        auto [pos_between, angle] =
            getPositionInPathPart(path_to_snap, new_portion_between_gates);

        VelocitySearchResult current_fastest_copy = current_fastest;
        TrajectoryPoint new_trajectory_point(
            pos_between, 180.0 * (angle / M_PI), false, target_gate_id,
            topologi_path_id, new_portion_between_gates);

        current_fastest_copy.gates_waypoints.insert(
            current_fastest_copy.gates_waypoints.begin() +
                csc.trajectory_index + 1,
            new_trajectory_point);

        VelocitySearchResult new_vel_result =
            vel_search_graph.find_velocities_in_positions(
                current_fastest_copy.gates_waypoints, false,
                !enforce_end_velocity);
        pq_velocity_search.push(new_vel_result);
        INFO("added new VelocitySearchResult")
      }
    }
  }

  vel_search_graph.save_track_trajectory(fastest_trajectory.trajectories,
                                         fastest_trajectory.time,
                                         output_folder + "samples_pmm.csv");

  INFO("found path pmm")
  INFO("fastest_trajectory time " << fastest_trajectory.time)
  // exit(1);

  std::vector<TrajectoryState> gates_waypoints_end;
  gates_waypoints_end.resize(fastest_trajectory.gates_waypoints.size() - 1);
  INFO_VAR(fastest_trajectory.gates_waypoints.size())
  INFO_VAR(gates_waypoints_end.size())

  int gate_i = 0;
  for (size_t i = 1; i < fastest_trajectory.gates_waypoints.size(); i++) {
    if (fastest_trajectory.gates_waypoints[i].is_gate) {
      gates[gate_i].v = fastest_trajectory.found_gates_speeds[i - 1];
      gate_i++;
    }
    gates_waypoints_end[i - 1].state.p =
        fastest_trajectory.gates_waypoints[i].position;
    gates_waypoints_end[i - 1].state.v =
        fastest_trajectory.found_gates_speeds[i - 1];
    gates_waypoints_end[i - 1].is_gate =
        fastest_trajectory.gates_waypoints[i].is_gate;
  }
  end.v = fastest_trajectory.found_gates_speeds.back();

  INFO("after")
  for (size_t i = 0; i < gates_waypoints_end.size(); i++) {
    INFO(i << " gate " << gates_waypoints_end[i].is_gate << " "
           << gates_waypoints_end[i].state.p.transpose());
  }

  double time_total = 0;
  TrMaxAcc3D xyz =
      calc_max_acc_thrust(start, gates_waypoints_end[0].state, drone);
  time_total += xyz.time();
  if (gates_waypoints_end[0].is_gate) {
    pmm_cumulative_times.push_back(time_total);
  }
  for (size_t i = 1; i < gates_waypoints_end.size(); i++) {
    TrMaxAcc3D xyz = calc_max_acc_thrust(gates_waypoints_end[i - 1].state,
                                         gates_waypoints_end[i].state, drone);
    time_total += xyz.time();
    if (gates_waypoints_end[i].is_gate) {
      pmm_cumulative_times.push_back(time_total);
    }
  }

  INFO("time_total pmm" << time_total)
  INFO("pmm_cumulative_times")
  std::cout << pmm_cumulative_times << std::endl;
  // exit(1);

  Primitive pr_all;
  std::vector<std::tuple<int, int>> gate_primitive_ids;
  std::vector<std::tuple<int, int>> command_primitive_ids;
  if (gates_waypoints_end.size() > 1) {
    std::vector<Primitive> prims;
    INFO_MAGENTA("creating primitive start")
    Primitive pr_start = MotionPrimitive::acc_primitive(
        start, gates_waypoints_end[0].state, drone);
    INFO_MAGENTA("end creating primitive start")
    prims.push_back(pr_start);
    for (int var = 1; var < gates_waypoints_end.size(); ++var) {
      INFO_MAGENTA("creating primitive gate/wp " << (var - 1) << " and "
                                                 << (var))
      Primitive pr =
          MotionPrimitive::acc_primitive(gates_waypoints_end[var - 1].state,
                                         gates_waypoints_end[var].state, drone);
      INFO_MAGENTA("end creating primitive between gate " << (var - 1)
                                                          << " and " << (var))
      prims.push_back(pr);
    }

    //  exit(1);
    INFO_MAGENTA("joining primitives")
    std::tie(pr_all, primitive_cum_times, gate_primitive_ids,
             command_primitive_ids) =
        SST::join_primitives(prims, gates_waypoints_end);
    INFO("pr_all time " << pr_all.time);
    INFO("pr_all rotations " << pr_all.rotations.size());
  } else {
    pr_all = MotionPrimitive::acc_primitive(start, end, drone);
    primitive_cum_times[0] = pr_all.time;
    gate_primitive_ids.push_back({0, pr_all.rotations.size() - 1});
    int num_rot_commands = 0;
    for (size_t rci = 0; rci < pr_all.rotations.size(); rci++) {
      if (pr_all.rotations[rci].rotation.t2 > 0) {
        num_rot_commands += 3;
        if (rci == pr_all.rotations.size() - 1) {
          num_rot_commands -= 1;
        }
      } else {
        num_rot_commands += 2;
      }
    }
    command_primitive_ids.push_back(
        {0, pr_all.translations.size() + num_rot_commands - 1});
  }

  for (size_t j = 0; j < gate_primitive_ids.size(); j++) {
    INFO("prim id from " << std::get<0>(gate_primitive_ids[j]) << " to "
                         << std::get<1>(gate_primitive_ids[j]))
    INFO("commands from " << std::get<0>(command_primitive_ids[j]) << " to "
                          << std::get<1>(command_primitive_ids[j]))
  }

  for (int var = 0; var < primitive_cum_times.size(); ++var) {
    INFO_VAR(primitive_cum_times[var])
  }
  for (int var = 0; var < pmm_cumulative_times.size(); ++var) {
    INFO_VAR(pmm_cumulative_times[var])
  }
  pmm_cumulative_times = primitive_cum_times;
  // exit(1);

  INFO("checking all bef")
  MotionPrimitive::check_primitive(pr_all, true);
  INFO("checking all aft")

  // exit(1);

  INFO("get motor commands")
  std::vector<Command> commands =
      MotionPrimitive::get_motor_commands(pr_all, drone);

  /* fill the reference trajectory begin */
  reference_commands = commands; // get_references();
  // reference_commands = get_references();

  double max_time = 0;
  for (int var = 0; var < reference_commands.size(); ++var) {
    max_time += reference_commands[var].time;
    std::cout << reference_commands[var] << std::endl;
    if (!isfinite(reference_commands[var].command(0))) {
      INFO("mottor command not finite")
      exit(1);
    }
    if (var > 0 &&
        reference_commands[var - 1].id + 1 != reference_commands[var].id) {
      INFO("motor command bad")
      exit(1);
    }
  }

  // for (size_t var = 0; var < ((int)command_primitive_ids.size()) - 1; var++)
  // {
  //   int cid = std::get<1>(command_primitive_ids[var]);
  //   if (commands[cid].type != MAX_OMEGA_ROTATION) {
  //     INFO("command between should be max rotation")
  //     INFO_VAR(cid)
  //     exit(1);
  //   }
  // }

  INFO("max_time " << max_time);
  // exit(1);

  INFO_VAR(commands.size());
  if ((gates_waypoints_end.size() > 1) &&
      (std::get<1>(command_primitive_ids.back()) !=
           ((int)commands.size()) - 1 ||
       std::get<1>(gate_primitive_ids.back()) !=
           ((int)pr_all.rotations.size()) - 1)) {
    INFO("bad commands size");
    INFO_VAR(std::get<1>(command_primitive_ids.back()));
    INFO_VAR((((int)commands.size()) - 1));
    INFO_VAR(std::get<1>(gate_primitive_ids.back()));
    INFO_VAR((((int)pr_all.rotations.size()) - 1));
    exit(1);
  }

  // std::vector<std::vector<DroneState>> references_samples = get_samples();
  std::vector<std::vector<DroneState>> references_samples =
      MotionPrimitive::get_samples(pr_all, gate_primitive_ids,
                                   desired_num_reference_samples,
                                   rotation_sample_mult_ratio, drone);

  INFO_VAR(num_gates)
  const double plain_samples_desired_dt = desired_dt;
  INFO_VAR(plain_samples_desired_dt)
  std::vector<std::vector<DroneState>> references_samples_plain =
      MotionPrimitive::get_samples_plain(pr_all, gate_primitive_ids,
                                         plain_samples_desired_dt, drone);

  int num_ref_samples_plain = 0;
  INFO("references_samples")
  for (size_t i = 0; i < references_samples_plain.size(); i++) {
    num_ref_samples_plain += references_samples_plain[i].size();
    // for (size_t j = 0; j < references_samples_plain[i].size(); j++) {
    //   INFO(references_samples_plain[i][j].t
    //        << " " << references_samples_plain[i][j].p.transpose())
    // }
  }

  // INFO_VAR(num_ref_samples_plain)
  // exit(1);

  if (tree_parts.size() != references_samples_plain.size()) {
    INFO("diff tree parts size");
    exit(1);
  }
  for (size_t tpi = 0; tpi < tree_parts.size(); tpi++) {
    int num_ref_samples_plain_part = references_samples_plain[tpi].size();
    flann::Matrix<float> samples_matrix_part(
        new float[num_ref_samples_plain_part * 3], num_ref_samples_plain_part,
        3);
    for (size_t rj = 0; rj < num_ref_samples_plain_part; rj++) {
      for (size_t i = 0; i < 3; i++) {
        samples_matrix_part[rj][i] = references_samples_plain[tpi][rj].p(i);
      }
    }
    tree_parts[tpi].flann_index_reference_pmm =
        new flann::Index<flann::L2_3D<float>>(samples_matrix_part,
                                              flann::KDTreeIndexParams(4));
    tree_parts[tpi].flann_index_reference_pmm->buildIndex();
  }

  DroneState::saveSamplesToFile(output_folder + "samples.csv",
                                references_samples_plain);

  for (int var = 0; var < gate_primitive_ids.size(); ++var) {
    INFO(std::get<0>(gate_primitive_ids[var])
         << " " << std::get<1>(gate_primitive_ids[var]));
  }
  INFO("references_samples size " << references_samples.size());

  // exit(1);
  /* fill the reference trajectory end */

  root = new TreeNode<DroneState>();
  root->data = start;
  root->distance_from_start = 0;
  root->id = 0;
  root->nn_id = 0;
  long unsigned int num_nodes = 1;
  flann::Matrix<float> start_matrix(new float[1 * DroneState::SIZE_NN], 1,
                                    DroneState::SIZE_NN);
  for (int var = 0; var < DroneState::SIZE_NN; ++var) {
    start_matrix[0][var] = start.x(var);
  }
  tree_parts[0].flann_index = new flann::Index<DroneDistance<float>>(
      start_matrix, flann::KDTreeIndexParams(4));
  tree_parts[0].flann_index->buildIndex();
  tree_parts[0].nodes.push_back(root);
  tree_parts[0].open_list.insert(maptype_pair(root->id, root));
  gate_nodes[0].push_back(root);

  Witness *start_w = new Witness();
  start_w->id = 0;
  start_w->representative = root;
  start_w->state = root;
  tree_parts[0].witnesses.push_back(start_w);

  tree_parts[0].flann_index_witness = new flann::Index<DroneDistance<float>>(
      start_matrix, flann::KDTreeIndexParams(4));
  tree_parts[0].flann_index_witness->buildIndex();

  INFO("created");

  TreeNode<DroneState> *new_node = new TreeNode<DroneState>();

  while (continue_iterate) {
    iter += 1;
    inform_progress();

    double near_distance;
    long unsigned int near_index;
    bool bias_goal = false;
    DroneState random_state;

    int random_gate_idx;
    if (equalize_openlist_sizes) {
      random_gate_idx = equalize_openlist_size_random_gate();
    } else {
      random_gate_idx = randIntMinMax(std::max(0, maximally_reached_gate - 3),
                                      maximally_reached_gate);
    }

    if (best_node != NULL &&
        randDoubleMinMax(0.0, 1.0) < bias_use_reference_reach_gate_commands &&
        best_node_commands.size() > 0) {
      bias_goal = true;
    }

    if (bias_goal) {
      random_state =
          sample_araund(best_node_samples, random_gate_idx, true, bias_goal);
    } else {
      random_state =
          sample_araund(references_samples, random_gate_idx, true, bias_goal);
    }

    // double dv_local = dv;
    // if (drone->distance_pos(random_state, gates[random_gate_idx]) <
    //     pos_tolerance_radius) {
    //   dv_local = dv / 10.0;
    // }

    best_near(random_state, dv, random_gate_idx, &near_distance, &near_index);
    // INFO("best near intex " << near_index);
    if (randDoubleMinMax(0.0, 1.0) < bias_start_from_previous_gate &&
        gate_nodes[random_gate_idx].size() > 0) {
      for (size_t tst = 0; tst < 10; tst++) {
        const int rand_gate_node =
            randDoubleMinMax(0, gate_nodes[random_gate_idx].size() - 1);
        const int gate_node_id =
            gate_nodes[random_gate_idx][rand_gate_node]->id;
        if (tree_parts[random_gate_idx].open_list.count(gate_node_id) > 0) {
          near_index = gate_node_id;
          break;
        }
      }
    }

    TreeNode<DroneState> *parent =
        tree_parts[random_gate_idx].open_list[near_index];

    const DroneState &from_state = parent->data;

    std::vector<Command> reference_to_use = reference_commands;

    if (bias_goal) {
      reference_to_use = best_node_commands;
    }

    max_time =
        pmm_cumulative_times[random_gate_idx] * reaching_gate_time_ratio_to_pmm;
    auto [new_states, states_between] =
        simulate_expand(from_state, reference_to_use, max_time, bias_goal);

    // INFO_VAR(states_between.size());
    // for (size_t i = 0; i < new_states.size(); i++) {
    //   INFO_VAR(new_states[i].p.transpose())

    //   INFO("i " << i << " size " << states_between[i].size());
    //   for (size_t j = 0; j < states_between[i].size(); j++) {
    //     INFO_VAR(states_between[i][j].p.transpose())
    //   }
    // }
    // INFO_VAR(new_states.size());

    // if (random_gate_idx == num_gates) {

    bool reached_between = false;
    if (true) {
      bool reached_by_states_between = false;
      double min_dist_vel = DBL_MAX;
      DroneState near;
      const size_t sb_s = states_between.size();
      for (size_t i = 0; i < sb_s; i++) {
        const size_t sb_s2 = states_between[i].size();
        for (size_t j = 0; j < sb_s2; j++) {
          const DroneState &sb = states_between[i][j];
          // INFO("next gate distance")
          const double next_gate_distance =
              drone->distance_pos(gates[random_gate_idx], sb);

          const bool is_within_next_gate =
              next_gate_distance <= pos_tolerance_radius;

          const bool newly_reached_gate =
              is_within_next_gate &&
              random_gate_idx + 1 > maximally_reached_gate;
          if (newly_reached_gate and
              abs(sb.command.id -
                  std::get<1>(command_primitive_ids[random_gate_idx])) <= 1) {
            // INFO_GREEN("newly_reached_gate by state between time "
            //            << sb.t << " while pmm "
            //            << pmm_cumulative_times[random_gate_idx])

            if (sb.t < pmm_cumulative_times[random_gate_idx] *
                           reaching_gate_time_ratio_to_pmm &&
                sb.command.type == MAX_OMEGA_ROTATION) {
              // INFO_CYAN("would pass , time " << sb.t)
              // INFO_CYAN("command type " << sb.command.type)

              if (random_gate_idx < gates.size() - 1) {
                const Scalar min_time_to_gate = get_pointmass_time_to_gate(
                    sb, gates[random_gate_idx], drone->max_acc_);
                const Scalar time_to_gate_with_elapsed =
                    sb.t + min_time_to_gate;

                /*
                if (time_to_gate_with_elapsed >
                    pmm_cumulative_times[random_gate_idx] * 1.1) {
                  INFO("could not reach next gate");
                  INFO_VAR(time_to_gate_with_elapsed)
                  INFO_VAR(min_time_to_gate)
                  INFO_VAR(pmm_cumulative_times[random_gate_idx])
                  INFO_VAR(random_gate_idx)
                  INFO_VAR(gates[random_gate_idx])
                  INFO_VAR(sb)
                  exit(1);
                }
                */
              }

              if (new_states[i].t !=
                  states_between[i][j].t) { // only if not there
                reached_between = true;

                new_states.insert(new_states.begin() + i, states_between[i][j]);
                states_between.insert(states_between.begin() + i,
                                      states_between[i]);

                states_between[i].resize(j);
                states_between[i + 1].erase(states_between[i + 1].begin(),
                                            states_between[i + 1].begin() + j +
                                                1);

                // INFO_VAR(states_between[i].size())
                // INFO_VAR(states_between[i + 1].size())

                // INFO("after states")
                // for (size_t nsi = 0; nsi < new_states.size(); nsi++) {
                //   for (size_t sbi = 0; sbi < states_between[nsi].size();
                //        sbi++) {
                //     INFO(nsi << " " << sbi << " time "
                //              << states_between[nsi][sbi].t)
                //   }
                //   INFO(nsi << " state time " << new_states[nsi].t)
                // }

                break;
                // exit(1);
              }
            }
          }

          // if (reached_between) {
          //   INFO("reached by between")
          //   exit(1);
          // }
          // if (newly_reached_gate) {
          //   const double vel_dist =
          //     drone->distance_vel(gates[random_gate_idx], sb);
          //   double best_time = 0;
          //   if (end_distance_min_time_node != NULL) {
          //     best_time = end_distance_min_time_node->data.t;
          //   }
          //   if (vel_dist < end_vel_distance_tolerance &&
          //       vel_dist < min_dist_vel && sb.t < best_time) {
          //     reached_by_states_between = true;

          //     near = sb;
          //     min_dist_vel = vel_dist;
          //   }
          // }
        }

        if (reached_between) {
          break;
        }
      }
      // if (reached_by_states_between and near.t < min_near_t) {
      //   min_near_t = near.t;
      //   INFO_GREEN("newly reached by intermidiate state in iter "
      //              << iter << " gate " << random_gate_idx)
      //   INFO_CYAN(near)
      // }
    }

    // if (new_states.size() > 0) {
    //   INFO("new_states size "
    //        << new_states.size() << " time diff "
    //        << new_states[new_states.size() - 1].t - from_state.t << " pos
    //        diff "
    //        << (new_states[new_states.size() - 1].p - from_state.p).norm())
    // }
    // for(){}

    // INFO("loop states")
    int num_not_added = 0;
    for (int nni = 0; nni < new_states.size(); ++nni) {
      DroneState &new_state = new_states[nni];
      // INFO("nni" << nni << " new_state, random_gate_idx " <<
      // random_gate_idx);

      // do not get out of reference
      double nearest_reference_distance = 0;
      long unsigned int found_index = 0;
      nearest_reference(new_state, random_gate_idx, &nearest_reference_distance,
                        &found_index);
      const double nearest_pmm_time =
          references_samples_plain[random_gate_idx][nearest_reference_distance]
              .t;

      // if (new_state.t >
      //     nearest_pmm_time * reaching_gate_time_ratio_to_pmm + 1.0) {
      //   // INFO("nearest_pmm_time " << nearest_pmm_time << " new_state.t "
      //   //                          << new_state.t)
      //   break;
      // }

      if (nearest_reference_distance > max_dist_from_reference) {
        break;
      }

      // use same command id as it is supposed to and pmm reference time
      if ((random_gate_idx < num_gates) &&
          ((new_state.command.id -
                std::get<1>(command_primitive_ids[random_gate_idx]) >
            1) ||
           (new_state.t > pmm_cumulative_times[random_gate_idx] *
                              reaching_gate_time_ratio_to_pmm))) {
        // break if using command id above next gate or new state t above pmm
        // cumulative times
        break;
      }

      if (random_gate_idx < gates.size() - 1) {
        const Scalar min_time_to_gate = get_pointmass_time_to_gate(
            new_state, gates[random_gate_idx], drone->max_acc_);
        const Scalar time_to_gate_with_elapsed = new_state.t + min_time_to_gate;

        if (time_to_gate_with_elapsed > pmm_cumulative_times[random_gate_idx] *
                                            reaching_gate_time_ratio_to_pmm) {
          // INFO("break")
          break;
        }
      }

      // INFO_VAR(states_between[nni].size())
      // INFO_VAR(near_index)
      // INFO_VAR(random_gate_idx)
      // INFO_VAR(tree_parts[random_gate_idx].open_list.count(near_index))
      bool is_in_collision =
          is_collisions_between(parent->data, new_state, states_between[nni]);

      if (is_in_collision) {
        // INFO("in collision")
        break;
      }

      double witness_distance = DBL_MAX;
      long int witness_index = -1;

      double witness_distance_next_gate;
      long int witness_index_next_gate = -1;

      // INFO("next gate distance ")
      // INFO_VAR(gates[random_gate_idx])
      const double next_gate_distance =
          drone->distance_pos(gates[random_gate_idx], new_state);
      const bool is_within_next_gate =
          (next_gate_distance <= pos_tolerance_radius) &&
          (random_gate_idx < num_gates);

      // if (random_gate_idx < gates.size() - 1 && !is_within_next_gate) {
      //   const Scalar min_time_to_gate = get_pointmass_time_to_gate(
      //     new_state, gates[random_gate_idx], drone->max_acc_);
      //   const Scalar time_to_gate_with_elapsed = new_state.t +
      //   min_time_to_gate;

      //   if (time_to_gate_with_elapsed >
      //   pmm_cumulative_times[random_gate_idx]) {
      //     break;
      //   }
      // }

      if (is_within_next_gate) {
        if (new_state.command.id -
                std::get<1>(command_primitive_ids[random_gate_idx]) >
            1) {
          break;
        } else if (new_state.command.id -
                           std::get<1>(command_primitive_ids[random_gate_idx]) <
                       1 &&
                   new_state.command.type == CommandType::RISE) {
          int rise_before_gate =
              std::get<1>(command_primitive_ids[random_gate_idx]);
          while (reference_commands[rise_before_gate].type !=
                 CommandType::RISE) {
            rise_before_gate--;
          }

          INFO_CYAN("reached between by RISE "
                    << new_state.command.id
                    << " should consider continuing as id " << rise_before_gate)
          // INFO(reference_commands[rise_before_gate].type << " == "
          //                                                <<
          //                                                CommandType::RISE)

          new_states.resize(nni + 1);
          // TreeNode<DroneState> *parent_change_id = parent;
          // while (parent_change_id != NULL &&
          //        parent_change_id->data.command.id == new_state.command.id) {
          //   parent_change_id->data.command.id = rise_before_gate;
          //   parent_change_id = parent_change_id->parent;
          // }
          new_state.command.id = rise_before_gate;
        } else {
          break;
        }
      }

      // if (is_within_next_gate &&
      //     abs(new_state.command.id -
      //         std::get<1>(command_primitive_ids[random_gate_idx])) > 1) {
      //   if (random_gate_idx + 1 >
      //       maximally_reached_gate) {  // newly reached gate but command id
      //                                  // is wrong
      //     INFO_RED("wrong command id "
      //              << new_state.command.id << " to reach next gate "
      //              << std::get<1>(command_primitive_ids[random_gate_idx]))
      //   }
      //   break;
      //   if (new_state.command.id >
      //       std::get<1>(command_primitive_ids[random_gate_idx])) {
      //     break;
      //   }
      // }

      bool newly_reached_gate = false;

      new_node->data = new_state;
      new_node->distance_from_start = new_state.t;

      // check if next gate is reached and if it is newly reached
      if (is_within_next_gate) {
        // INFO("is_within_next_gate")
        newly_reached_gate = random_gate_idx + 1 > maximally_reached_gate;
        maximally_reached_gate =
            std::max(random_gate_idx + 1, maximally_reached_gate);

        random_gate_idx += 1;
        // INFO("random_gate_idx " << random_gate_idx);
        if (newly_reached_gate) {
          min_next_gate_distance = DBL_MAX;
          min_next_gate_reference_id = 0;
          best_node = NULL;
          INFO_GREEN("newly reached gate "
                     << maximally_reached_gate << " iter " << iter << " time "
                     << new_state.t << " with command " << new_state.command);
          INFO_VAR(next_gate_distance)
          INFO_VAR(gates[random_gate_idx])
          for (size_t gnid = 0; gnid < gate_nodes[random_gate_idx - 1].size();
               gnid++) {
            INFO(gnid << " state reach gate " << (random_gate_idx - 1)
                      << " time: "
                      << gate_nodes[random_gate_idx - 1][gnid]->data.t
                      << " command: "
                      << gate_nodes[random_gate_idx - 1][gnid]->data.command)
          }
          INFO_VAR((random_gate_idx - 1))
          INFO("end id should be "
               << std::get<1>(command_primitive_ids[random_gate_idx - 1]))
        }
      }

      // nearest witness for this gate
      if (newly_reached_gate) {
        // the only witness is also the only new one
        witness_index =
            add_witness(new_node, random_gate_idx, newly_reached_gate);
      } else {
        // find the nearest witness
        nearest_witness(new_state, random_gate_idx, &witness_distance,
                        &witness_index);
        if (witness_distance > ds) {
          witness_index = add_witness(new_node, random_gate_idx, false);
        }
      }

      TreeNode<DroneState> *current_wit_rep =
          tree_parts[random_gate_idx].witnesses[witness_index]->representative;
      // INFO("current_wit_rep " << current_wit_rep);

      if (current_wit_rep == NULL || new_state.t < current_wit_rep->data.t) {
        if (current_wit_rep != NULL) {
          // INFO_RED("remove from openlist " << current_wit_rep->id);
          tree_parts[random_gate_idx].open_list.erase(current_wit_rep->id);

          // do not add removed to the close list close list
          // tree_parts[random_gate_idx].close_list.insert(
          //  maptype_pair(current_wit_rep->id, current_wit_rep));

          const long unsigned int flann_id = current_wit_rep->nn_id;

          tree_parts[random_gate_idx].flann_index->removePoint(flann_id);
        }

        // populate node
        new_node->id = num_nodes;
        new_node->nn_id = tree_parts[random_gate_idx].nodes.size();
        new_node->parent = parent;
        parent->children.push_back(new_node);

        // get_commands_from_node(new_node);

        if (is_within_next_gate) {
          gate_nodes[random_gate_idx].push_back(new_node);
        }

        // INFO("open_list insert " << new_node->id);
        tree_parts[random_gate_idx].witnesses[witness_index]->representative =
            new_node;
        tree_parts[random_gate_idx].open_list.insert(
            maptype_pair(new_node->id, new_node));
        tree_parts[random_gate_idx].nodes.push_back(new_node);

        flann::Matrix<float> add_point_matrix(
            new float[1 * DroneState::SIZE_NN], 1, DroneState::SIZE_NN);
        for (int var = 0; var < DroneState::SIZE_NN; ++var) {
          add_point_matrix[0][var] = new_node->data.x(var);
        }

        // INFO("flann insert inser");
        if (newly_reached_gate) {
          INFO_RED("flann insert " << random_gate_idx);
          tree_parts[random_gate_idx].flann_index =
              new flann::Index<DroneDistance<float>>(
                  add_point_matrix, flann::KDTreeIndexParams(4));

          tree_parts[random_gate_idx].flann_index->buildIndex();
        } else {
          tree_parts[random_gate_idx].flann_index->addPoints(add_point_matrix,
                                                             2.0);
        }
        // INFO("flann inserted ok");
        check_goal_distances(new_node, random_gate_idx);

        // since we are adding the nodes in loop, set parent for next new
        // node
        parent = new_node;
        // create new_node for next iteration
        new_node = new TreeNode<DroneState>();
        num_nodes += 1;

      } else {
        // break adding the nodes in the list if the current one does not
        // create a new witness break; if (num_not_added > 2) {
        //   break;
        // }
        num_not_added++;
        // since we are adding the nodes in loop, set parent for next new
        // node tree_parts[random_gate_idx].close_list.insert(
        //  maptype_pair(new_node->id, new_node));
        // INFO("a")
        new_node->id = -1;
        // new_node->nn_id = tree_parts[random_gate_idx].nodes.size();
        new_node->parent = parent;
        // INFO("b")
        parent->children.push_back(new_node);
        // tree_parts[random_gate_idx].nodes.push_back(new_node);
        // INFO("c")
        parent = new_node;
        // create new_node for next iteration
        // INFO("d")
        new_node = new TreeNode<DroneState>();
        num_nodes += 1;
        // INFO("e")
      }
    }
    // exit(1);
    // usleep(1000000);
    if (iter >= max_num_iterations) {
      continue_iterate = false;
    }
    if (iter_last_impro != 0 &&
        iter - iter_last_impro >= max_num_iterations_wo_impr) {
      continue_iterate = false;
    }
  }

  comp_timer.stop();
  total_calc_time_ms = comp_timer.getTimeMS();
  total_iter = iter;

  INFO("loop ended");
  INFO_VAR(total_calc_time_ms);
  INFO("iter " << iter << " end_distance_pos_min " << end_distance_pos_min);
  if (signal_received >= 0) {
    INFO("signal_received during iteration " << signal_received);
  }
  if (end_distance_min_time_node != NULL) {
    INFO("iterate ended with time " << end_distance_min_time_node->data.t);
    INFO("end_distance_min_node " << end_distance_min_time_node->data);
    goal_reached_quality = end_distance_min_time_node->data.t;
    TreeNode<DroneState>::savePathToFile(output_folder + "path.csv",
                                         end_distance_min_time_node);

    std::vector<TreeNode<DroneState> *> trajectory =
        TreeNode<DroneState>::backtrace_to_root(end_distance_min_time_node);
    for (size_t tri = 0; tri < trajectory.size(); tri++) {
      INFO("time:" << trajectory[tri]->data.t
                   << " command:" << trajectory[tri]->data.command)
    }

    std::vector<DroneState> all_states_between =
        SST::simulate_trajectory(trajectory, true);
    TreeNode<DroneState>::savePathToFile(output_folder + "path_dense.csv",
                                         all_states_between);
  }

  // std::vector<std::vector<TreeNode<DroneState> *>> all_nodes;
  // for (size_t i = 0; i < tree_parts.size(); i++) {
  //   all_nodes.push_back(tree_parts[i].nodes);
  // }

  // TreeNode<DroneState>::saveTreeToFile("tree.csv", all_nodes);

  // std::vector<std::unordered_map<long unsigned int, TreeNode<DroneState> *>>
  //   all_open_lists;
  // INFO_VAR(tree_parts.size())
  // for (size_t i = 0; i < tree_parts.size(); i++) {
  //   all_open_lists.push_back(tree_parts[i].open_list);
  // }
  // TreeNode<DroneState>::saveOpenlistToFile("open_list.csv", all_open_lists);

  log_results();
}

void SST::log_results() {
  INFO("logging data");
  std::vector<std::pair<std::string, std::string>> log_data;

  // results
  log_data.push_back({"NAME", name});
  log_data.push_back({"COMP_TIME_MS", std::to_string(total_calc_time_ms)});
  log_data.push_back({"NUM_ITERS", std::to_string(total_iter)});
  log_data.push_back({"GOAL_REACHED_MS", std::to_string(goal_reached_time_ms)});
  log_data.push_back({"GOAL_REACHED_ITER", std::to_string(goal_reached_iter)});
  log_data.push_back(
      {"SOLUTION_QUALITY", std::to_string(goal_reached_quality)});
  log_data.push_back(
      {"MAX_REACHED_GATE", std::to_string(maximally_reached_gate)});

  // params
  log_data.push_back({"bias_start_from_previous_gate",
                      std::to_string(bias_start_from_previous_gate)});
  log_data.push_back({"bias_use_reference_reach_gate_commands",
                      std::to_string(bias_use_reference_reach_gate_commands)});
  log_data.push_back({"max_expand_time", std::to_string(max_expand_time)});
  log_data.push_back(
      {"ref_time_deviation", std::to_string(ref_time_deviation)});

  // distance scales
  log_data.push_back({"pos_scale", std::to_string(pos_scale)});
  log_data.push_back({"att_scale", std::to_string(att_scale)});
  log_data.push_back({"vel_scale", std::to_string(vel_scale)});
  log_data.push_back({"omega_scale", std::to_string(omega_scale)});

  std::stringstream ss_start;
  ss_start << start.p(0) << "," << start.p(1) << "," << start.p(2);
  log_data.push_back({"start", ss_start.str()});
  std::stringstream ss_end;
  ss_end << end.p(0) << "," << end.p(1) << "," << end.p(2);
  log_data.push_back({"goal", ss_end.str()});

  log_data.push_back(
      {"randomize_p_around", std::to_string(randomize_v_around)});
  log_data.push_back(
      {"randomize_v_around", std::to_string(randomize_v_around)});
  log_data.push_back(
      {"randomize_w_around", std::to_string(randomize_w_around)});
  log_data.push_back(
      {"randomize_q_angle_around", std::to_string(randomize_q_angle_around)});

  std::ofstream myfile;
  myfile.open(logfile.c_str(), std::ios_base::app);
  if (myfile.is_open()) {
    for (size_t i = 0; i < log_data.size(); i++) {
      myfile << log_data[i].first << ":" << log_data[i].second << ";";
    }
    myfile << std::endl;
    myfile.close();
  }
}

void SST::inform_progress() {
  if (iter % 500 == 0) {
    if (end_distance_vel_min_node != NULL) {
      INFO("iter " << iter << "min next gate " << min_next_gate_distance
                   << " id " << min_next_gate_reference_id
                   << " min end dist pos " << end_distance_pos_min
                   << " min passing vel dist " << end_distance_vel_min
                   << " with t " << end_distance_vel_min_node->data.t);
    } else {
      INFO("iter " << iter << "min next gate " << min_next_gate_distance
                   << " id " << min_next_gate_reference_id
                   << " min end dist pos " << end_distance_pos_min
                   << " min passing vel dist " << end_distance_vel_min
                   << " with t " << best_node->data.t << " max tim allowed "
                   << pmm_cumulative_times[maximally_reached_gate]);
    }
    std::stringstream ss;
    for (size_t i = 0; i < gate_nodes.size(); i++) {
      ss << i << ":" << gate_nodes[i].size() << ";";
    }
    INFO(ss.str())

    if (end_distance_min_time_node != NULL) {
      INFO("time " << end_distance_min_time_node->data.t << " dist pos "
                   << end_distance_min_time_dist_pos << " dist vel "
                   << end_distance_min_time_dist_vel);
      INFO("node " << end_distance_min_time_node->data);
      INFO("min_near_t " << min_near_t)
    }
  }
}

void SST::check_goal_distances(TreeNode<DroneState> *new_node,
                               const int current_gate_idx) {
  // INFO("current_gate_idx " << current_gate_idx << " num_gates " <<
  // num_gates)
  if (current_gate_idx == maximally_reached_gate) {
    double nearest_reference_distance = 0;
    long unsigned int found_index = 0;
    nearest_reference(new_node->data, current_gate_idx,
                      &nearest_reference_distance, &found_index);
    // found_index = 0;
    const double next_gate_distance =
        drone->distance_pos(gates[current_gate_idx], new_node->data);
    if (found_index > min_next_gate_reference_id ||
        (found_index == min_next_gate_reference_id &&
         next_gate_distance < min_next_gate_distance)) {
      min_next_gate_distance = next_gate_distance;
      min_next_gate_reference_id = found_index;
      if (current_gate_idx != num_gates) {
        // for goal biasing not near last node
        set_best_node(new_node, current_gate_idx);
      }
    }
  }

  if (current_gate_idx == num_gates) {
    // const double end_distance = drone->distance(new_node->data, end);
    // INFO("calc dist end");
    const double end_distance_pos = drone->distance_pos(new_node->data, end);
    // const double end_distance_wo_pos =
    //      drone->distance_without_pos(new_node->data, end);
    const double end_distance_vel = drone->distance_vel(new_node->data, end);

    if (end_distance_pos < end_distance_pos_min) {
      end_distance_pos_min = end_distance_pos;
      end_distance_pos_min_node = new_node;
      if (end_distance_pos > pos_tolerance_radius) {
        // for goal biasing if pos_tolerance_radius not reached in goal
        set_best_node(new_node, current_gate_idx);
      }
    }

    // INFO_VAR(pos_tolerance_radius)
    if (end_distance_pos <= pos_tolerance_radius) {
      if (end_distance_vel < end_distance_vel_min) {
        end_distance_vel_min = end_distance_vel;
        end_distance_vel_min_node = new_node;

        if (end_distance_vel > end_vel_distance_tolerance) {
          // for goal biasing if pos_tolerance_radius reached but not vel
          // tolerance
          set_best_node(new_node, current_gate_idx);
        }
      }

      if (end_distance_vel <= end_vel_distance_tolerance) {
        if (end_distance_min_time_node == NULL ||
            new_node->data.t < end_distance_min_time_node->data.t) {
          // for goal biasing if goal reached
          set_best_node(new_node, current_gate_idx);
          INFO_CYAN("improved to better time " << new_node->data.t);
          goal_reached_iter = iter;
          iter_last_impro = iter;
          goal_reached_time_ms = comp_timer.getTimeMS();
          end_distance_min_time_node = new_node;
          end_distance_min_time_dist_pos = end_distance_pos;
          end_distance_min_time_dist_vel = end_distance_vel;
        }
      }
    }
  }
}

void SST::set_best_node(TreeNode<DroneState> *new_node,
                        const int current_gate_idx) {
  // for goal biasing begin
  best_node = new_node;
  std::vector<Command> commands_to_node = get_commands_from_node(new_node);
  std::vector<std::vector<DroneState>> samples =
      get_samples_from_node(new_node);
  /*
for (size_t i = 0; i < samples.size(); i++) {
  INFO("samples " << i << " are:")
  for (size_t is = 0; is < samples[i].size(); is++) {
    INFO(samples[i][is].p.transpose())
  }
}
*/

  append_rest_of_references(commands_to_node, reference_commands);
  best_node_commands = commands_to_node;
  best_node_samples = samples;

  // for goal biasing end
}

void SST::signal(int sig) {
  this->continue_iterate = false;
  this->signal_received = sig;
  INFO_RED("signal_received " << signal_received);
  INFO_RED("iter " << this->iter);
  // while (!ended) {
  // std::cout << "wait for iteration to finish" << std::endl;
  // usleep(5000000);
  //}
}

void SST::best_near(const DroneState &state, const double dv,
                    const unsigned int gate_index, double *found_distance,
                    long unsigned int *node_index) {
  // fill flann state where to search
  // INFO("best_near begin gate_index" << gate_index);
  // INFO("flann_indexes at gate index " << flann_indexes[gate_index])
  // INFO("flann_indexes size " << flann_indexes[gate_index]->size())

  for (int var = 0; var < DroneState::SIZE_NN; ++var) {
    search_state[0][var] = state.x(var);
  }
  if (tree_parts[gate_index].flann_index == NULL) {
    INFO("null flann index")
    INFO_VAR(gate_index);
    exit(1);
  }
  int num_found = tree_parts[gate_index].flann_index->radiusSearch(
      search_state, flann_indices, flann_dists, dv * dv,
      flann::SearchParams(128));
  // INFO("num_found " << num_found)
  if (num_found > 0) {
    // find the one with minimal time
    // INFO("min time variant");
    double min_t = DBL_MAX;
    int mint_id = 0;
    // int min_nnindex = 0;
    for (int var = 0; var < num_found; ++var) {
      const long unsigned int nnindex = flann_indices[0][var];
      if (gate_index >= tree_parts.size()) {
        INFO_RED("bad gate_index " << gate_index)
      }
      if (nnindex >= tree_parts[gate_index].nodes.size()) {
        INFO_RED("bad nnindex " << nnindex)
      }
      const long unsigned int node_id =
          tree_parts[gate_index].nodes[nnindex]->id;
      // const long unsigned int node_id =
      //   node_flann_idx_to_node_idx[gate_index][nnindex];
      // INFO("tree_parts[gate_index].open_list[node_id]->data.t "
      //  << tree_parts[gate_index].open_list[node_id]->data.t)
      if (tree_parts[gate_index].open_list.count(node_id) < 1) {
        INFO_RED("node_id " << node_id << " not in openlinst")
      }
      if (tree_parts[gate_index].open_list[node_id]->data.t < min_t) {
        min_t = tree_parts[gate_index].open_list[node_id]->data.t;
        mint_id = tree_parts[gate_index].open_list[node_id]->id;
      }
    }

    *node_index = mint_id;
    *found_distance = min_t;
  } else {
    // find the nearest neighbor
    // INFO("closest variant");
    num_found = tree_parts[gate_index].flann_index->knnSearch(
        search_state, flann_indices, flann_dists, 1, flann::SearchParams(128));
    // INFO("num_found " << num_found)
    const long unsigned int nnindex = flann_indices[0][0];
    if (gate_index >= tree_parts.size()) {
      INFO_RED("bad gate_index " << gate_index)
    }
    if (nnindex >= tree_parts[gate_index].nodes.size()) {
      INFO_RED("bad nnindex " << nnindex)
    }
    const long unsigned int node_id = tree_parts[gate_index].nodes[nnindex]->id;
    const long unsigned int node_nn_id =
        tree_parts[gate_index].nodes[nnindex]->nn_id;
    // const long unsigned int node_id =
    //  node_flann_idx_to_node_idx[gate_index][nnindex];
    if (tree_parts[gate_index].open_list.count(node_id) < 1) {
      INFO_RED("node_id " << node_id << " not in openlinst")
    }
    *found_distance = tree_parts[gate_index].open_list[node_id]->data.t;
    *node_index = tree_parts[gate_index].open_list[node_id]->id;
  }
  // INFO("best_near end");
}

void SST::nearest_witness(const DroneState &state,
                          const unsigned int gate_index, double *found_distance,
                          long int *node_index) {
  flann::Matrix<float> search_state(new float[1 * DroneState::SIZE_NN], 1,
                                    DroneState::SIZE_NN);
  for (int var = 0; var < DroneState::SIZE_NN; ++var) {
    search_state[0][var] = state.x(var);
  }
  tree_parts[gate_index].flann_index_witness->knnSearch(
      search_state, flann_indices, flann_dists, 1, flann::SearchParams(128));
  const long unsigned int nnindex = flann_indices[0][0];
  const long unsigned int wit_idx =
      tree_parts[gate_index].witnesses[nnindex]->id;

  *found_distance = sqrt(flann_dists[0][0]);
  *node_index = wit_idx;
}

void SST::nearest_reference(const DroneState &state,
                            const unsigned int gate_index,
                            double *found_distance,
                            long unsigned int *found_index) {
  // INFO("nearest reference beg")
  flann::Matrix<float> nearest_reference_search_state(nearest_reference_data, 1,
                                                      3);
  for (int var = 0; var < 3; ++var) {
    nearest_reference_search_state[0][var] = state.p(var);
    // nearest_reference_search_state[0][var + 3] = state.v(var);
  }
  // INFO("filled")
  tree_parts[gate_index].flann_index_reference_pmm->knnSearch(
      nearest_reference_search_state, flann_indices, flann_dists, 1,
      flann::SearchParams(128));
  // flann_index_reference->knnSearch(nearest_reference_search_state,
  //                                  flann_indices, flann_dists, 1,
  //                                  flann::SearchParams(128));
  // INFO("searched")
  *found_index = flann_indices[0][0];
  *found_distance = sqrt(flann_dists[0][0]);
  // INFO("nearest reference end")
}

index_type SST::add_witness(TreeNode<DroneState> *new_node,
                            const unsigned int gate_index,
                            const bool newly_reached_gate) {
  // INFO_GREEN("add witness");
  Witness *new_w = new Witness();
  new_w->id = tree_parts[gate_index].witnesses.size();
  new_w->representative = NULL;
  new_w->state = new_node;
  tree_parts[gate_index].witnesses.push_back(new_w);
  // INFO("added to witness list")

  flann::Matrix<float> add_point_matrix(new float[1 * DroneState::SIZE_NN], 1,
                                        DroneState::SIZE_NN);
  for (int var = 0; var < DroneState::SIZE_NN; ++var) {
    add_point_matrix[0][var] = new_node->data.x(var);
  }
  if (newly_reached_gate) {
    // INFO("created new flan index")
    tree_parts[gate_index].flann_index_witness =
        new flann::Index<DroneDistance<float>>(add_point_matrix,
                                               flann::KDTreeIndexParams(4));
    tree_parts[gate_index].flann_index_witness->buildIndex();
    const long unsigned int flann_idx = 0;
    // witness_flann_idx_to_node_idx[gate_index][flann_idx] = new_w->id;
  } else {
    // INFO("created matrix")
    // INFO("flann_indexes_witness[gate_index]"
    //    << flann_indexes_witness[gate_index])
    // add point to flann
    // const long unsigned int flann_idx =
    // witness_flann_idx_to_node_idx[gate_index].size();
    // witness_flann_idx_to_node_idx[gate_index][flann_idx] = new_w->id;
    tree_parts[gate_index].flann_index_witness->addPoints(add_point_matrix,
                                                          2.0);
  }
  return new_w->id;
}

std::vector<Command> SST::get_commands_from_node(TreeNode<DroneState> *node) {
  std::vector<Command> commands;

  Command last_command = node->data.command;
  commands.push_back(last_command);
  double tot_time_commands = 0;
  tot_time_commands += last_command.time;
  // INFO("last_command " << last_command)
  // same command  id can happen as there is
  TreeNode<DroneState> *current_node = node->parent;
  while (current_node != NULL) { // untill reached root

    if ((last_command.id != current_node->data.command.id &&
         (last_command.type != CommandType::RISE ||
          current_node->data.command.type != CommandType::RISE)) &&
        current_node->data.command.id >= 0) {
      last_command = current_node->data.command;
      // INFO("last_command " << last_command)
      tot_time_commands += last_command.time;
      commands.push_back(last_command);
    }

    current_node = current_node->parent;
  }
  // INFO_VAR(tot_time_commands);
  // INFO_VAR(node->data.t)
  if (fabs(tot_time_commands - node->data.t) > 1e-6) {
    INFO("wrong command total times")
    INFO_VAR(tot_time_commands)
    INFO_VAR(node->data.t)
    exit(1);
  }
  std::reverse(commands.begin(), commands.end());

  return commands;
}

std::vector<std::vector<DroneState>>
SST::get_samples_from_node(TreeNode<DroneState> *node) {
  // INFO("get samples begin")
  std::vector<Command> commands = get_commands_from_node(node);
  std::vector<std::vector<DroneState>> samples;
  samples.resize(num_gates + 1);
  // INFO("samples size " << samples.size())
  DroneState current_state = root->data;
  samples[0].push_back(current_state);
  double t = 0; // keep the expansion time
  int gate_id = 0;
  double sim_for_tot = 0;
  // INFO_VAR(gate_id)
  for (int cid = 0; cid < commands.size(); ++cid) {
    Command &c = commands[cid];
    double sim_for = c.time;
    Vector<4> mot_c = c.command;
    sim_for_tot += sim_for;
    const double dt_parts = ceil(sim_for / desired_dt);
    const double dt = sim_for / dt_parts;
    DroneState last_saved_state = current_state;
    for (int dt_i = 0; dt_i < dt_parts; dt_i++) {
      DroneState next_state;
      // INFO("mot_c " << mot_c.transpose() << " current_state "<<
      // current_state);
      drone->rk4(current_state.x, dt, mot_c, next_state.x);
      // INFO("next_state " << next_state);
      next_state.t = current_state.t + dt;
      t += dt;
      current_state = next_state;

      // creating more samples

      // INFO("calc dist last saved");
      const double dist_last_saved =
          drone->distance_pos(last_saved_state, current_state);

      if (dist_last_saved > pos_tolerance_radius / 4.0 ||
          dt_i + 1 == dt_parts) {
        if (drone->distance_pos(gates[gate_id], current_state) <
            pos_tolerance_radius) {
          if (gate_id < num_gates) {
            gate_id += 1;
            // INFO("increase gate_id to " << gate_id)
          }
        }
        if (gate_id > 0 &&
            drone->distance_pos(gates[gate_id - 1], current_state) <
                pos_tolerance_radius) {
          samples[gate_id - 1].push_back(current_state);
        }
        samples[gate_id].push_back(current_state);
        last_saved_state = current_state;
      }
      if (dist_last_saved > pos_tolerance_radius) {
        INFO("can not catch saving states " << dist_last_saved)
        INFO("increase the desired dt or add state interpolation");
        exit(1);
      }

      // DEBUG part, remove later
      if (!drone->check_limits(current_state.x)) {
        INFO("out of drone limits, get samples from node");
        INFO_VAR(current_state);
        INFO_VAR(node->data);
        exit(1);
      }
    }
    if (fabs(t - sim_for_tot) > 1e-6) {
      INFO_VAR(t)
      INFO_VAR(sim_for_tot)
      ERROR_RED("not correct sim for in middle")
      exit(1);
    }
  }
  // INFO_VAR(sim_for_tot)
  if (fabs(sim_for_tot - t) > 1e-6) {
    INFO_VAR(t)
    INFO_VAR(sim_for_tot)
    ERROR_RED("not correct sim for tot")
    exit(1);
  }
  // INFO("get samples end")
  return samples;
}

void SST::append_rest_of_references(std::vector<Command> &commands_part,
                                    std::vector<Command> &reference) {
  const size_t size_commands_part = commands_part.size();
  /*
  for (size_t i = 0; i < size_commands_part; i++) {
    INFO("1commands_part " << commands_part[i])
  }
  */
  Command last_command = commands_part[size_commands_part - 1];
  commands_part.resize(reference.size());
  /*
  for (size_t i = 0; i < size_commands_part; i++) {
    INFO("2commands_part " << commands_part[i])
  }
  */

  for (size_t i = size_commands_part; i < reference.size(); i++) {
    commands_part[i] = reference[i];
  }

  // copy the rotation axis to make it continuous
  if (last_command.type == CommandType::MAX_TAU_POSITIVE ||
      last_command.type == CommandType::MAX_OMEGA_ROTATION) {
    for (size_t i = size_commands_part; i < commands_part.size(); i++) {
      if (commands_part[i].type == CommandType::RISE) {
        break;
      }
      commands_part[i].x_y_rotation_vector = last_command.x_y_rotation_vector;
    }
  }

  // DEBUG
  /*
  for (size_t i = 1; i < commands_part.size(); i++) {
    if (commands_part[i - 1].type == commands_part[i].type) {
      INFO("size_commands_part " << size_commands_part)
      INFO("same type!!!!!! " << i)
      for (size_t ri = 0; ri < reference.size(); ri++) {
        INFO(ri << " " << reference[ri])
      }

      INFO("command part")
      for (size_t ri = 0; ri < commands_part.size(); ri++) {
        INFO(ri << " " << commands_part[ri])
      }

      exit(1);
    }
  }
  */
}

std::vector<DroneState>
SST::simulate_trajectory(const std::vector<TreeNode<DroneState> *> &trajectory,
                         bool command_in_node_before) {
  std::vector<DroneState> states_all; //{trajectory[0]->data};
  if (!command_in_node_before) {
    states_all.push_back(trajectory[0]->data);
  }
  INFO_VAR(desired_dt)
  DroneState current_state = trajectory[0]->data;
  DroneState next_state = current_state;
  double t = 0;
  double t_simfor = 0;
  double non_max_acc_time = 0;
  double max_acc_time = 0;
  Command mot_c;
  double dt = 0;
  for (size_t i = 1; i < trajectory.size(); i++) {
    if ((i + 1 < trajectory.size() && trajectory[i]->data.command.id ==
                                          trajectory[i + 1]->data.command.id) ||
        (trajectory[i]->data.command.type == CommandType::RISE &&
         trajectory[i + 1]->data.command.type == CommandType::RISE)) {
      // the command continues later
      continue;
    }
    mot_c = trajectory[i]->data.command;
    if ((trajectory[i]->data.command.command.array() == drone->max_t_motor_)
            .all()) {
      max_acc_time += mot_c.time;
    } else {
      non_max_acc_time += mot_c.time;
    }
    const double sim_for = mot_c.time;
    const double dt_parts = ceil(sim_for / desired_dt);
    dt = sim_for / dt_parts;
    INFO("from " << current_state.p.transpose() << " simulate " << mot_c
                 << " for " << mot_c.time)
    INFO("dt " << dt << " dt_parts " << dt_parts);

    DroneState last_saved_state = current_state;
    for (int dt_i = 0; dt_i < dt_parts; dt_i++) {
      // add state first
      if (command_in_node_before) {
        current_state.command = mot_c;
        current_state.command.time = dt;
        current_state.a =
            (next_state.getAttitude() *
                 Vector<3>(0, 0, current_state.command.command.sum()) +
             GVEC) /
            drone->m_;
        states_all.push_back(current_state);
      }
      // INFO("t " << current_state.t << " a " << current_state.a)
      // then simulate it
      drone->rk4(current_state.x, dt, mot_c.command, next_state.x);
      // INFO("next_state " << next_state);
      next_state.t = current_state.t + dt;
      t += dt;

      if (!command_in_node_before) {
        next_state.command = mot_c;
        next_state.command.time = dt;
        next_state.a = (next_state.getAttitude() *
                            Vector<3>(0, 0, next_state.command.command.sum()) +
                        GVEC) /
                       drone->m_;
        states_all.push_back(next_state);
      }

      current_state = next_state;
      // next_state.command = mot_c;
      // next_state.command.time = dt;
    }
    t_simfor += sim_for;
    INFO("sim_for " << sim_for << " t " << t << " t_simfor " << t_simfor)
    if ((current_state.p - trajectory[i]->data.p).norm() > 0.05) {
      INFO("bad position ");
      INFO_VAR(current_state.p)
      INFO_VAR(trajectory[i]->data.p)
      exit(1);
    }
  }

  if (command_in_node_before) {
    current_state.command = mot_c;
    current_state.command.time = dt;
    current_state.a =
        (next_state.getAttitude() *
             Vector<3>(0, 0, current_state.command.command.sum()) +
         GVEC) /
        drone->m_;
    states_all.push_back(current_state);
  }

  INFO_GREEN("non_max_acc_time " << non_max_acc_time)
  INFO_GREEN("max_acc_time " << max_acc_time)
  INFO_GREEN("max_acc_time + non_max_acc_time "
             << (max_acc_time + non_max_acc_time))
  INFO_GREEN("t tot " << t)
  return states_all;
}

std::tuple<std::vector<DroneState>, std::vector<std::vector<DroneState>>>
SST::simulate_expand(const DroneState &from_state,
                     const std::vector<Command> &references,
                     const double max_time, const bool goal_bias) {
  // INFO("-------------------------------------");
  // INFO("simulate_expand begin from_state p:"
  //      << from_state.p.transpose() << " v:" << from_state.v.transpose()
  //      << " w:" << from_state.w.transpose())
  // const double desired_dt = 0.01;
  std::vector<DroneState> new_states; // to be returned with new states
  std::vector<std::vector<DroneState>> states_between;
  states_between.resize(new_states.size() + 1);

  // get random expansion time
  const double rand_expand_time = randDoubleMinMax(0.0, 1.0) * max_expand_time;
  // clip it to be minimally desired_dt
  const double wanted_to_sim_t = std::max(rand_expand_time, desired_dt);
  // limit it to be within the max time
  double simulate_for_t = std::min(wanted_to_sim_t, max_time - from_state.t);

  DroneState current_state = from_state;
  Command current_command = from_state.command;
  if (from_state.command.id < 0) {
    // in case the root is used initialize by the first reference command
    current_command = references[0];
    current_command.time = 0;
  }

  double t = 0; // keep the expansion time
  // itterate over the reference commands
  for (int i = current_command.id; i < references.size(); ++i) {
    // INFO_VAR(current_state.w.transpose())
    // INFO_VAR(current_command.x_y_rotation_vector.transpose())
    // INFO_VAR(current_command.type)
    if (t > simulate_for_t)
      break; // break if over the intended time

    double command_reference_duration = references[i].time;

    // randomize the rotation vector
    if (references[i].type == MAX_TAU_POSITIVE) {
      if (current_command.time == 0) {
        Vector<3> rotation_vector = references[i].x_y_rotation_vector;
        const Vector<2> rotation_vector_xy = rotation_vector.segment<2>(0);
        // INFO("old rot axis " <<
        // references[i].x_y_rotation_vector.transpose()
        //                      << " size "
        //                      << references[i].x_y_rotation_vector.norm())

        double rand_rotate_ang;
        if (goal_bias) {
          rand_rotate_ang =
              randDoubleMinMax(-rand_rotate_rotation_vector_ang_goal_bias,
                               rand_rotate_rotation_vector_ang_goal_bias);
        } else {
          rand_rotate_ang = randDoubleMinMax(-rand_rotate_rotation_vector_ang,
                                             rand_rotate_rotation_vector_ang);
        }
        // INFO_VAR(rand_rotate_ang)
        const Eigen::Rotation2D<Scalar> rot_m_rot_vector(rand_rotate_ang);
        // INFO_VAR(rot_m_rot_vector);
        const Vector<2> randomized_rotation_vector_xy =
            rot_m_rot_vector * rotation_vector_xy;

        rotation_vector.segment<2>(0) = randomized_rotation_vector_xy;
        // INFO_VAR(rotation_vector.transpose());
        double ang_acc_rot_vec, ang_speed_rot_vec;

        // set the command for created rotation vector
        std::tie(ang_acc_rot_vec, ang_speed_rot_vec, current_command.command,
                 command_reference_duration) =
            drone->get_max_motors_for_rotation(rotation_vector);

        current_command.x_y_rotation_vector = rotation_vector;

        if (rotation_vector(2) > PRECISION) {
          // DEBUG if only
          INFO("rotation vector is not around xy");
          exit(1);
        }
      } else {
        // do not randomize the rotation axis when in the command, get only
        // the duration
        std::tie(std::ignore, std::ignore, std::ignore,
                 command_reference_duration) =
            drone->get_max_motors_for_rotation(
                current_command.x_y_rotation_vector);
      }
    }

    // we need to get the command_reference_duration and the command
    if (references[i].type == MAX_TAU_NEGATIVE) {
      if (current_command.time == 0) {
        // set both the command and time
        std::tie(std::ignore, std::ignore, current_command.command,
                 command_reference_duration) =
            drone->get_max_motors_for_rotation(
                -current_command.x_y_rotation_vector);
        // INFO("set MAX_TAU_NEGATIVE command_reference_duration "
        //      << command_reference_duration << " for rot vector "
        //      << -current_command.x_y_rotation_vector.transpose())
      } else {
        // set only the time
        std::tie(std::ignore, std::ignore, std::ignore,
                 command_reference_duration) =
            drone->get_max_motors_for_rotation(
                -current_command.x_y_rotation_vector);
      }
    }

    // INFO_VAR(current_command.time)
    double scale = 1.0; // scale for the time duration of some commands
    if (references[i].type == MAX_OMEGA_ROTATION ||
        references[i].type == RISE) {
      if (goal_bias) {
        scale += randDoubleMinMax(-ref_time_deviation_goal_bias,
                                  ref_time_deviation_goal_bias);
      } else {
        scale += randDoubleMinMax(-ref_time_deviation, ref_time_deviation);
      }
    }
    const double want_to_simulate_command = command_reference_duration * scale;

    // substract already simulated command time from wanted simulation time
    double sim_for = want_to_simulate_command - current_command.time;
    if (references[i].type == MAX_OMEGA_ROTATION ||
        references[i].type == RISE) {
      // clip the sim_for by simulate_for_t only, do not do it in ang acc as
      // that one reaches omega max
      sim_for = std::min(sim_for, simulate_for_t - t);
    }

    // INFO_VAR(sim_for)
    if (sim_for > 0) {
      const Vector<4> mot_c = current_command.command;
      // INFO("current_command " << current_command)
      // INFO("mot_c " << mot_c.transpose());
      const double dt_parts = ceil(sim_for / desired_dt);
      const double dt = sim_for / dt_parts;
      // INFO("dt " << dt << " dt_parts " << dt_parts);

      DroneState last_saved_state = current_state;
      for (int dt_i = 0; dt_i < dt_parts; dt_i++) {
        DroneState next_state;
        // INFO("mot_c " << mot_c.transpose() << " current_state "<<
        // current_state);
        drone->rk4(current_state.x, dt, mot_c, next_state.x);
        // INFO("next_state " << next_state);
        next_state.t = current_state.t + dt;
        t += dt;
        current_command.time += dt;
        current_state = next_state;

        // creating more samples
        const double dist_last_saved =
            drone->distance_pos(last_saved_state, current_state);
        if (dist_last_saved > pos_tolerance_radius / 4.0) {
          current_state.command = current_command;
          states_between.back().push_back(current_state);
          last_saved_state = current_state;
          // INFO("add to states_between")
        }
        if (dist_last_saved > pos_tolerance_radius) {
          INFO("can not catch saving states " << dist_last_saved)
          INFO("increase the desired dt or add state interpolation");
          exit(1);
        }

        // DEBUG part, remove later
        if (!drone->check_limits(current_state.x)) {
          INFO("out of drone limits");
          exit(1);
        }
      }

      current_state.command = current_command;
      new_states.push_back(current_state);
      states_between.resize(new_states.size() + 1);

      // int is = current_command.type;
      // if ((current_state.w.norm() > 0.1 and
      //      (is == 2 or is == 3 or is == 6 or is == 7)) or
      //     (current_state.w.norm() < 0.1 and
      //      (is == 0 or is == 1 or is == 5 or is == 9))) {
      //   INFO("vrrrr");
      //   INFO("current command id " << is);
      //   INFO("current_state.w is" << current_state.w);
      //   INFO("current_state after" << current_state);
      //   exit(1);
      // }

      // for (int nsi = 0; nsi < new_states.size(); ++nsi) {
      //  INFO("ns after add " << nsi << " " << new_states[nsi]);
      //}
    }
    // exit(1);
    if (i + 1 < references.size()) {
      // switch to new command
      // INFO("switch to next command " << i + 1)
      Command last_command = current_command;
      current_command = references[i + 1];
      if (!isfinite(current_command.command(0))) {
        INFO("not finite current command")
        exit(1);
      }
      current_command.time = 0;
      // randomize the const rotation part motor commands
      if (current_command.type == MAX_OMEGA_ROTATION) {
        const Scalar val = current_command.command(0);
        // const Scalar scaled_val =
        //   std::min(val * randDoubleMinMax(0.8, drone->max_t_motor_ / val),
        //            drone->max_t_motor_);
        // const Scalar scaled_val = randDoubleMinMax(
        //  std::max(val, drone->min_t_motor_), drone->max_t_motor_);
        const Scalar scaled_val = drone->max_t_motor_;
        if (!isfinite(scaled_val)) {
          INFO("not finite scaled val")
          INFO_VAR(val)
          INFO_VAR(scaled_val)
          exit(1);
        }
        for (int cmid = 0; cmid < 4; ++cmid) {
          current_command.command(cmid) = scaled_val;
        }
      }
      // coppy the rotation vector randomized in MAX_TAU_POSITIVE to the
      if (current_command.type == MAX_OMEGA_ROTATION ||
          current_command.type == MAX_TAU_NEGATIVE) {
        current_command.x_y_rotation_vector = last_command.x_y_rotation_vector;
      }
    }
  }
  // INFO("-------------------------------------");
  return {new_states, states_between};
}

bool SST::is_collisions_between(const DroneState &from, const DroneState &to,
                                const std::vector<DroneState> &states_between) {
  if (!check_collisions_) {
    return false;
  }
  // INFO("is_collisions_between")
  // INFO("from " << from)
  // INFO_VAR(from.p.transpose())
  // INFO_VAR(to.p.transpose())
  // INFO_VAR(states_between.size())
  const double clearance_to = map->getClearence(to.p);
  if (!std::isfinite(clearance_to) || clearance_to < min_clearance_) {
    return true;
  }

  for (size_t i = 0; i < states_between.size(); i++) {
    // INFO_VAR(states_between[i].p.transpose())
    const double clearance = map->getClearence(states_between[i].p);
    if (!std::isfinite(clearance) || clearance < min_clearance_) {
      return true;
    }
  }
  return false;
}

void SST::test_hower(DroneState start) {
  // test hover
  Vector<4> hover;
  hover << G * drone->m_ / 4.0, G * drone->m_ / 4.0, G * drone->m_ / 4.0,
      G * drone->m_ / 4.0;
  DroneState current_state = start;
  INFO(current_state);
  double dt = 0.05;
  for (int var = 0; var < 100; ++var) {
    DroneState next_state;
    drone->rk4(current_state.x, dt, hover, next_state.x);
    next_state.t = current_state.t + dt;
    current_state = next_state;
    INFO(current_state);
  }
}

/*
return the min time for pointmass model to get between two states using amax
and gravity in z can be used for filtering out the useless samples and not
expanding those
- it is like the heurisitc in A*
*/
Scalar SST::get_pointmass_time_to_gate(const DroneState &from,
                                       const DroneState &to,
                                       const Scalar amax) {
  const Vector<3> max_acc(amax, amax, amax);
  const Vector<3> min_acc(-amax, -amax, -amax - G);
  const Vector<3> from_v = from.v;
  const Vector<3> from_p = from.p;
  const Vector<3> to_p = to.p;
  // const Vector<3> to_v = to.v;

  // double max_time = 0;
  // for (size_t i = 0; i < 3; i++) {
  //   const Tr1D tr = one_dim_double_integrator_two_acc(
  //     from_p(i), from_v(i), to_p(i), to_v(i), max_acc(i), min_acc(i), i,
  //     false);
  //   INFO("i tr " << tr)
  //   if (tr.exists) {
  //     const double time = tr.time();
  //     if (time > max_time) {
  //       max_time = time;
  //     }
  //   } else {
  //     INFO_RED("tr does not exists")
  //     exit(1);
  //   }
  // }

  Vector<3> diff_p = from_p - to_p;
  // // INFO("diff_p bef " << diff_p.transpose())
  // // the target does not have to be visited preciselly, thus make the diff_p
  // // closer to zero by the tolerance radius
  double max_time = 0;

  for (size_t i = 0; i < 3; i++) {
    if (diff_p(i) < 0) {
      // diff was negative so make it closer to zero by tolerance radius, but
      // maximally 0
      diff_p(i) = std::min(diff_p(i) + pos_tolerance_radius, 0.0);
    } else {
      // diff was positive so make it closer to zero by tolerance radius, but
      // maximally 0
      diff_p(i) = std::max(diff_p(i) - pos_tolerance_radius, 0.0);
    }

    // 0 = diff_p + v0*t +- 0.5*a*t^2

    if (diff_p(i) > 0) {
      double min_axis_time = std::numeric_limits<double>::max();
      const double d_pos = from_v(i) * from_v(i) - 2.0 * max_acc(i) * diff_p(i);
      if (d_pos > 0) {
        const double t1_a_pos = (-from_v(i) + sqrt(d_pos)) / max_acc(i);
        const double t2_a_pos = (-from_v(i) - sqrt(d_pos)) / max_acc(i);
        if (t1_a_pos >= 0) {
          min_axis_time = std::min(min_axis_time, t1_a_pos);
        }
        if (t2_a_pos >= 0) {
          min_axis_time = std::min(min_axis_time, t2_a_pos);
        }
      }
      //   (-from_v + D_a_pos.cwiseSqrt()).cwiseProduct(max_acc.cwiseInverse());

      const double d_neg = from_v(i) * from_v(i) - 2.0 * min_acc(i) * diff_p(i);
      if (d_neg > 0) {
        const double t1_a_neg = (-from_v(i) + sqrt(d_neg)) / min_acc(i);
        const double t2_a_neg = (-from_v(i) - sqrt(d_neg)) / min_acc(i);
        if (t1_a_neg >= 0) {
          min_axis_time = std::min(min_axis_time, t1_a_neg);
        }
        if (t2_a_neg >= 0) {
          min_axis_time = std::min(min_axis_time, t2_a_neg);
        }
      }
      // INFO(i << " min_axis_time " << min_axis_time)
      max_time = std::max(max_time, min_axis_time);
    }
  }

  // // INFO("diff_p aft " << diff_p.transpose())
  // // INFO_VAR(pos_tolerance_radius)
  // // INFO_VAR(max_acc.transpose())
  // // INFO_VAR(from_p.transpose())
  // // INFO_VAR(from_v.transpose())
  // // INFO_VAR(to_p.transpose())

  // const Vector<3> D_a_pos =
  //   from_v.cwiseProduct(from_v) - 2 * max_acc.cwiseProduct(diff_p);

  // const Vector<3> D_a_neg =
  //   from_v.cwiseProduct(from_v) - 2 * min_acc.cwiseProduct(diff_p);

  // // INFO_VAR(D_a_pos.transpose())
  // // INFO_VAR(D_a_neg.transpose())
  // // INFO_VAR(D_a_pos.cwiseSqrt().transpose())
  // // INFO_VAR(D_a_neg.cwiseSqrt().transpose())
  // // INFO_VAR((-from_v + D_a_pos.cwiseSqrt()).transpose())
  // // INFO_VAR((-from_v + D_a_neg.cwiseSqrt()).transpose())
  // // INFO_VAR(max_acc.cwiseInverse().transpose())
  // // INFO_VAR((-from_v +
  // // D_a_pos.cwiseSqrt()).cwiseProduct(max_acc.cwiseInverse()))
  // const Vector<3> t1_a_pos =
  //   (-from_v + D_a_pos.cwiseSqrt()).cwiseProduct(max_acc.cwiseInverse());
  // const Vector<3> t2_a_pos =
  //   (-from_v - D_a_pos.cwiseSqrt()).cwiseProduct(max_acc.cwiseInverse());
  // const Vector<3> t1_a_neg =
  //   (-from_v + D_a_neg.cwiseSqrt()).cwiseProduct(min_acc.cwiseInverse());
  // const Vector<3> t2_a_neg =
  //   (-from_v - D_a_neg.cwiseSqrt()).cwiseProduct(min_acc.cwiseInverse());
  // Matrix<3, 4> times;
  // times << t1_a_pos, t2_a_pos, t1_a_neg, t2_a_neg;
  // // INFO_VAR(times)
  // times = (times.array().isFinite()).select(times, 0);
  // // INFO_VAR(times)
  // const Scalar max_time =
  //   times.maxCoeff();  // max time per axis and omiting nans

  // // INFO_VAR(times)
  // // INFO_VAR(max_time)
  // if (!std::isfinite(max_time)) {
  //   INFO_VAR(times);
  //   INFO_VAR(t1_a_pos)
  //   INFO_VAR(t2_a_pos)
  //   INFO_VAR(t1_a_neg)
  //   INFO_VAR(t2_a_neg)
  //   INFO_VAR(max_time)
  //   exit(1);
  // }
  return max_time;
}

std::pair<Vector<3>, double>
SST::getPositionInPathPart(const path_with_length<Vector<3>> path,
                           const double portion) {
  if (portion < 0 || portion >= 1.0) {
    INFO("portion has to be between 0 and 1")
    exit(1);
  }
  double dist_from_start = 0;
  const double target_length = portion * path.length;

  for (size_t i = 1; i < path.plan.size(); i++) {
    const Vector<3> vec_from_to = (path.plan[i]->data - path.plan[i - 1]->data);
    const double current_lenght = vec_from_to.norm();

    const double next_dist_from_start = dist_from_start + current_lenght;

    if (target_length < next_dist_from_start) {
      const double length_part_current = target_length - dist_from_start;

      const Vector<3> pos =
          path.plan[i - 1]->data +
          vec_from_to * (length_part_current / current_lenght);
      const double angle = atan2(vec_from_to(1), vec_from_to(0));
      return {pos, angle};
    }
    dist_from_start = next_dist_from_start;
  }
  const int last_idx = path.plan.size() - 1;
  const Vector<3> vec_last =
      (path.plan[last_idx]->data - path.plan[last_idx - 1]->data);
  const double angle = atan2(vec_last(1), vec_last(0));
  return {path.plan.back()->data, angle};
}

std::pair<Vector<3>, double>
SST::getPositionInPathClosest(const path_with_length<Vector<3>> path,
                              Vector<3> position) {
  double portion = 0;
  if (portion < 0 || portion >= 1.0) {
    INFO("portion has to be between 0 and 1")
    exit(1);
  }
  double dist_from_start = 0;
  const double target_length = portion * path.length;

  for (size_t i = 1; i < path.plan.size(); i++) {
    const Vector<3> vec_from_to = (path.plan[i]->data - path.plan[i - 1]->data);
    const double current_lenght = vec_from_to.norm();

    const double next_dist_from_start = dist_from_start + current_lenght;

    if (target_length < next_dist_from_start) {
      const double length_part_current = target_length - dist_from_start;

      const Vector<3> pos =
          path.plan[i - 1]->data +
          vec_from_to * (length_part_current / current_lenght);
      const double angle = atan2(vec_from_to(1), vec_from_to(0));
      return {pos, angle};
    }
    dist_from_start = next_dist_from_start;
  }
  const int last_idx = path.plan.size() - 1;
  const Vector<3> vec_last =
      (path.plan[last_idx]->data - path.plan[last_idx - 1]->data);
  const double angle = atan2(vec_last(1), vec_last(0));
  return {path.plan.back()->data, angle};
}

std::vector<path_with_length<Vector<3>>>
SST::splitPathPart(const path_with_length<Vector<3>> path,
                   const double portion) {
  if (portion < 0 || portion >= 1.0) {
    INFO("portion has to be between 0 and 1")
    exit(1);
  }

  if (fabs(path.calc_path_length() - path.length) > 0.01) {
    INFO("original path lenght wrong");
    exit(1);
  }
  std::vector<path_with_length<Vector<3>>> splited;
  double dist_from_start = 0;
  const double target_length = portion * path.length;

  for (size_t i = 1; i < path.plan.size(); i++) {
    const Vector<3> vec_from_to = (path.plan[i]->data - path.plan[i - 1]->data);
    const double current_lenght = vec_from_to.norm();

    const double next_dist_from_start = dist_from_start + current_lenght;

    if (target_length < next_dist_from_start) {
      const double length_part_current = target_length - dist_from_start;
      INFO_VAR(length_part_current)
      INFO_VAR((length_part_current / current_lenght))
      const Vector<3> pos =
          path.plan[i - 1]->data +
          vec_from_to * (length_part_current / current_lenght);

      INFO_VAR(path.length)
      INFO("add len " << (path.plan[i - 1]->data - pos).norm())
      INFO("target_length " << target_length - dist_from_start)

      INFO("plan_old:")
      for (size_t pi = 0; pi < path.plan.size(); pi++) {
        INFO(path.plan[pi]->data.transpose())
      }

      path_with_length<Vector<3>> before;
      before.length = target_length;
      HeapNode<Vector<3>> *pos_middle = new HeapNode<Vector<3>>(pos);
      before.plan = std::vector<HeapNode<Vector<3>> *>(path.plan.begin(),
                                                       path.plan.begin() + i);
      before.plan.push_back(pos_middle);

      path_with_length<Vector<3>> after;
      after.length = path.length - target_length;
      after.plan = std::vector<HeapNode<Vector<3>> *>(path.plan.begin() + i,
                                                      path.plan.end());
      after.plan.insert(after.plan.begin(), pos_middle);

      INFO("before:")
      for (size_t pi = 0; pi < before.plan.size(); pi++) {
        INFO(before.plan[pi]->data.transpose())
      }

      INFO("after:")
      for (size_t pi = 0; pi < after.plan.size(); pi++) {
        INFO(after.plan[pi]->data.transpose())
      }

      if (fabs(before.length - before.calc_path_length()) > 0.01 ||
          fabs(after.length - after.calc_path_length()) > 0.01) {
        ERROR_RED("path length does not match")
        INFO_VAR(before.length)
        INFO_VAR(before.calc_path_length())
        INFO_VAR(after.length)
        INFO_VAR(after.calc_path_length())
        INFO_VAR((before.calc_path_length() + after.calc_path_length()))
        exit(1);
      }
      return {before, after};
    }
    dist_from_start = next_dist_from_start;
  }

  return splited;
}
