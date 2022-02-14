
#include "vel_search_graph.hpp"

#include <cfloat>

#include "timer.hpp"

std::shared_ptr<BaseMap> VelSearchGraph::map_;
double VelSearchGraph::collision_distance_check_;
double VelSearchGraph::min_clearance_;
bool VelSearchGraph::check_collisions_;

VelSearchGraph::VelSearchGraph(
    const Drone *drone, std::shared_ptr<BaseMap> map,
    const bool check_collisions, const double collision_distance_check,
    const double min_clearance, const double max_yaw_pitch_ang,
    const double precision_yaw_pitch_ang,
    const double yaw_pitch_cone_angle_boundary,
    const double min_velocity_size_boundary, const double min_velocity_size,
    const double max_velocity_size, const double precision_velocity_size,
    const std::string output_folder)
    : drone_(drone), max_yaw_pitch_ang_(max_yaw_pitch_ang),
      precision_yaw_pitch_ang_(precision_yaw_pitch_ang),
      yaw_pitch_cone_angle_boundary_(yaw_pitch_cone_angle_boundary),
      min_velocity_size_boundary_(min_velocity_size_boundary),
      min_velocity_size_(min_velocity_size),
      max_velocity_size_(max_velocity_size),
      precision_velocity_size_(precision_velocity_size),
      output_folder_(output_folder) {
  // : start_(start),
  //   end_(end),
  //   gates_(gates),
  //   gates_orientations_(gates_orientations),

  map_ = map;
  collision_distance_check_ = collision_distance_check;
  min_clearance_ = min_clearance;
  check_collisions_ = check_collisions;
}

VelocitySearchResult VelSearchGraph::find_velocities_in_positions(
    const std::vector<TrajectoryPoint> &gates_waypoints,
    const bool check_collisions_during_search,
    const bool end_free) { //
  int num_calls_gd_acc = 0;

  std::vector<Vector<3>> found_gates_speeds;
  std::vector<double> found_gates_times;
  const int gates_size = gates_waypoints.size();
  found_gates_speeds.resize(
      gates_size - 1); // also allow optimizing the end but omit the start
  found_gates_times.resize(gates_size - 1);

  std::vector<std::vector<Vector<2>>> gates_yaw_pitch_size_ranges;
  gates_yaw_pitch_size_ranges.resize(
      gates_size - 1, {Vector<2>(-max_yaw_pitch_ang_, max_yaw_pitch_ang_),
                       Vector<2>(-max_yaw_pitch_ang_, max_yaw_pitch_ang_),
                       Vector<2>(min_velocity_size_, max_velocity_size_)});

  std::vector<Vector<3>> gate_position_samples;
  gate_position_samples.resize(gates_size);
  // gate_position_samples[0] = start.p;

  std::vector<double> gates_orientations_deg;
  gates_orientations_deg.resize(gates_size);
  // gates_orientations_deg[0] = start_angle_deg;
  for (size_t i = 0; i < gates_size; i++) {
    gate_position_samples[i] = gates_waypoints[i].position;
    gates_orientations_deg[i] = gates_waypoints[i].angle_deg;
    INFO(i << " pos " << gates_waypoints[i].position.transpose() << " ang_deg "
           << gates_waypoints[i].angle_deg)
  }
  // gate_position_samples[gates_size + 1] = end.p;
  // gates_orientations_deg[gates_size + 1] = end_angle_deg;

  std::vector<std::vector<std::tuple<Vector<3>, Vector<3>, Vector<3>>>>
      gate_velocity_samples;
  gate_velocity_samples.resize(gates_size);
  gate_velocity_samples[0].push_back(
      {gates_waypoints[0].velocity, Vector<3>::Zero(), Vector<3>::Zero()});
  // shortest time structure for samples, add also the start
  std::vector<std::vector<std::pair<int, double>>> shortest_samples_times;
  shortest_samples_times.resize(gate_velocity_samples.size());
  shortest_samples_times[0].push_back({-1, 0});

  const int num_vel_samples_yaw_pitch_ang =
      ceil(2 * max_yaw_pitch_ang_ / precision_yaw_pitch_ang_) +
      1; // 2* range+zero
  const double vel_samples_yaw_pitch_ang_step =
      num_vel_samples_yaw_pitch_ang > 1
          ? 2 * max_yaw_pitch_ang_ /
                ((double)(num_vel_samples_yaw_pitch_ang - 1))
          : 0;

  const int num_vel_samples_size =
      ceil((max_velocity_size_ - min_velocity_size_) /
           precision_velocity_size_) +
      1; // from 0 to max velocity size

  const double vel_samples_size_step =
      num_vel_samples_size > 1 ? (max_velocity_size_ - min_velocity_size_) /
                                     ((double)(num_vel_samples_size - 1))
                               : 0;

  // INFO_VAR(num_vel_samples_yaw_pitch_ang)
  // INFO_VAR(vel_samples_yaw_pitch_ang_step)
  // INFO_VAR(num_vel_samples_size)
  // INFO_VAR(vel_samples_size_step)
  // INFO_VAR(shortest_samples_times.size())

  int sample_num_gates = gates_size - 2;
  if (end_free) {
    sample_num_gates += 1;
  } else {
    gate_velocity_samples[gates_size - 1].push_back(
        {gates_waypoints.back().velocity, Vector<3>::Zero(),
         Vector<3>::Zero()});
    shortest_samples_times[gate_velocity_samples.size() - 1].push_back(
        {-1, DBL_MAX}); // add end
  }

  Timer timer;
  timer.start();

  const int num_iter_improve = 5;

  // iteration over improevment of velocities
  for (size_t iterimp = 0; iterimp < num_iter_improve; iterimp++) {
    // loop samples 1....gatesize becase 0 is start and gatesize+1 is end
    for (size_t gid = 1; gid <= sample_num_gates; gid++) {
      // INFO("gid " << gid << " out of " << sample_num_gates)
      const int num_samples_gate =
          num_vel_samples_yaw_pitch_ang * num_vel_samples_yaw_pitch_ang *
          num_vel_samples_size; // zero size does not need yaw-pitch samples

      // get current min-max angles(relative to gate yaw) and velocity sizes
      const Vector<2> min_max_yaw = gates_yaw_pitch_size_ranges[gid - 1][0];
      const Vector<2> min_max_pitch = gates_yaw_pitch_size_ranges[gid - 1][1];
      const Vector<2> min_max_size = gates_yaw_pitch_size_ranges[gid - 1][2];

      // angle step sizes
      const double gate_vel_samples_size_step =
          (min_max_size(1) - min_max_size(0)) /
          ((double)(num_vel_samples_size - 1));
      const double gate_vel_samples_yaw_step =
          (min_max_yaw(1) - min_max_yaw(0)) /
          ((double)(num_vel_samples_yaw_pitch_ang - 1));
      const double gate_vel_samples_pitch_step =
          (min_max_pitch(1) - min_max_pitch(0)) /
          ((double)(num_vel_samples_yaw_pitch_ang - 1));

      // INFO_VAR(gid)
      // INFO_VAR(gates.size())
      const double gate_yaw =
          gates_orientations_deg[gid]; //  gates[gid - 1].angle_deg;

      gate_velocity_samples[gid].resize(num_samples_gate);

      shortest_samples_times[gid].resize(num_samples_gate, {-1, DBL_MAX});
      const std::pair<int, double> not_reached = std::make_pair(-1, DBL_MAX);
      std::fill(shortest_samples_times[gid].begin(),
                shortest_samples_times[gid].end(), not_reached);

      // save samples values
      int i = 0;
      for (size_t z_s = 0; z_s < num_vel_samples_size; z_s++) {
        const double vel_size =
            min_max_size(0) + z_s * gate_vel_samples_size_step;
        // INFO("\tvel_size " << vel_size)
        for (size_t x_s = 0; x_s < num_vel_samples_yaw_pitch_ang; x_s++) {
          const double yaw = min_max_yaw(0) + x_s * gate_vel_samples_yaw_step;
          // INFO("\t\tyaw " << yaw)
          for (size_t y_s = 0; y_s < num_vel_samples_yaw_pitch_ang; y_s++) {
            const double pitch =
                min_max_pitch(0) + y_s * gate_vel_samples_pitch_step;
            // INFO("\t\t\tpitch " << pitch)
            Matrix<3, 3> m;
            // m = Eigen::AngleAxisd(pitch * M_PI / 180.0, Vector<3>::UnitY()) *
            //     Eigen::AngleAxisd((gate_yaw + yaw) * M_PI / 180.0,
            //                       Vector<3>::UnitZ());
            m = Eigen::AngleAxisd((gate_yaw + yaw) * M_PI / 180.0,
                                  Vector<3>::UnitZ()) *
                Eigen::AngleAxisd(pitch * M_PI / 180.0, Vector<3>::UnitY());
            const Vector<3> vec = m * Vector<3>::UnitX() * vel_size;
            const Vector<3> vec_yaw_pitch_size(yaw, pitch, vel_size);
            const Vector<3> vec_yaw_pitch_size_idx(x_s, y_s, z_s);

            // INFO("gate " << gid << " gate yaw " << gate_yaw << " vel vec "
            //             << vec.transpose())
            // INFO("i " << i << " size " << gate_velocity_samples[gid].size())
            std::get<0>(gate_velocity_samples[gid][i]) = vec;
            std::get<1>(gate_velocity_samples[gid][i]) = vec_yaw_pitch_size;
            std::get<2>(gate_velocity_samples[gid][i]) = vec_yaw_pitch_size_idx;

            i++;
          }
        }
      }
    }

    // loop from the first gate gid_to = 1
    for (size_t gid_to = 1; gid_to < gates_size; gid_to++) {
      // get locally the parts
      INFO("-------------------------------------------------")
      INFO("gid_to " << gid_to << " from "
                     << gate_position_samples[gid_to - 1].transpose() << " to "
                     << gate_position_samples[gid_to].transpose());

      const std::vector<std::pair<int, double>> &from_shortest_samples_dists =
          shortest_samples_times[gid_to - 1];
      std::vector<std::pair<int, double>> &to_shortest_samples_dists =
          shortest_samples_times[gid_to];
      const std::vector<std::tuple<Vector<3>, Vector<3>, Vector<3>>>
          &from_samples = gate_velocity_samples[gid_to - 1];
      const std::vector<std::tuple<Vector<3>, Vector<3>, Vector<3>>>
          &to_samples = gate_velocity_samples[gid_to];

      // get the positions of the gates/start/end
      const Vector<3> &from_p = gate_position_samples[gid_to - 1];
      const Vector<3> &to_p = gate_position_samples[gid_to];

      // INFO("from p " << from_p.transpose())
      // INFO("to p " << to_p.transpose())
      double min_dist_to_gate = DBL_MAX;

      // loop for indexes of the samples in the gate (gid_to-1)
      for (int idx_from = 0; idx_from < from_shortest_samples_dists.size();
           idx_from++) {
        const double &time_from = from_shortest_samples_dists[idx_from].second;
        const Vector<3> &from_v = std::get<0>(from_samples[idx_from]);
        // INFO("from " << idx_from << " vel " << from_v.transpose()
        //              << " time_from " << time_from)
        if (time_from == DBL_MAX) {
          continue;
        }

        // loop for indexes of the samples in the gate gid_to
        for (int idx_to = 0; idx_to < to_shortest_samples_dists.size();
             idx_to++) {
          // INFO("idx_to " << idx_to)
          // std::pair<int, double> shortest_to_with_collision =
          //   to_shortest_samples_dists[idx_to];
          std::pair<int, double> &shortest_to =
              to_shortest_samples_dists[idx_to];

          // get the velocitions in particular samples

          const Vector<3> &to_v = std::get<0>(to_samples[idx_to]);

          DroneState from_state;
          from_state.p = from_p;
          from_state.v = from_v;
          DroneState to_state;
          to_state.p = to_p;
          to_state.v = to_v;

          const TrMaxAcc3D tr_max_acc =
              calc_max_acc_thrust(from_state, to_state, drone_, 0.001);
          num_calls_gd_acc += 1;

          if (tr_max_acc.exists()) {
            // INFO("test collision free")
            // VelocitySearchCollision cf = isCollisionFree(tr_max_acc);
            if (!check_collisions_during_search ||
                isCollisionFree(tr_max_acc).collision_free) {
              const double time_between = tr_max_acc.time();
              const double time_tot = time_between + time_from;

              if (time_tot < min_dist_to_gate) {
                min_dist_to_gate = time_tot;
              }
              if (time_tot < shortest_to.second) {
                shortest_to.first = idx_from;
                shortest_to.second = time_tot;
              }
            }
          } else {
            // INFO("to " << idx_to << " vel " << to_v.transpose() << " time "
            //            << DBL_MAX)
            // INFO_RED(tr_max_acc)
          }
        }
      }

      if (min_dist_to_gate == DBL_MAX) {
        INFO("there is no connection to gate "
             << gid_to << " with position "
             << gate_position_samples[gid_to].transpose())
        INFO_VAR(from_p.transpose())
        INFO_VAR(to_p.transpose())
        // INFO_VAR(gates_orientations[gid_to])
        // INFO_VAR(gates_orientations[gid_to - 1])
        exit(1);
      }
    }

    // find the shortest among the end ones
    double shortest_time = DBL_MAX;
    int end_best_idx = -1; // end idx is 0
    const int endi = shortest_samples_times.size() - 1;
    // INFO_VAR(endi)
    for (size_t i = 0; i < shortest_samples_times[endi].size(); i++) {
      const double time = shortest_samples_times[endi][i].second;
      if (time < shortest_time) {
        shortest_time = time;
        end_best_idx = i;
      }
    }

    int prev_sample_idx = end_best_idx;
    for (int g_id = endi; g_id > 0; g_id--) {
      found_gates_speeds[g_id - 1] =
          std::get<0>(gate_velocity_samples[g_id][prev_sample_idx]);

      Vector<3> indexes =
          std::get<2>(gate_velocity_samples[g_id][prev_sample_idx]);

      for (size_t i = 0; i < 3; i++) {
        const Vector<2> current_range =
            gates_yaw_pitch_size_ranges[g_id - 1][i];
        // INFO("yaw_pitch_cone_angle_boundary_ "
        //      << yaw_pitch_cone_angle_boundary_)
        const double range_size_half =
            (current_range(1) - current_range(0)) / 2.0;
        if (indexes(i) == 0) {
          // move the range of minmax down
          // INFO("select smaller sample " << g_id << " " << i)
          gates_yaw_pitch_size_ranges[g_id - 1][i] =
              Vector<2>(current_range(0) - range_size_half,
                        current_range(1) - range_size_half);
          if (i == 2) {
            // limit velocity size to be bigger than min_velocity_size_boundary_
            gates_yaw_pitch_size_ranges[g_id - 1][i] =
                gates_yaw_pitch_size_ranges[g_id - 1][i].cwiseMax(
                    min_velocity_size_boundary_);
          } else {
            // limit yaw pitch change from gate direction to be within
            // -yaw_pitch_cone_angle_boundary_ and
            // yaw_pitch_cone_angle_boundary_
            gates_yaw_pitch_size_ranges[g_id - 1][i] =
                gates_yaw_pitch_size_ranges[g_id - 1][i]
                    .cwiseMax(-yaw_pitch_cone_angle_boundary_)
                    .cwiseMin(yaw_pitch_cone_angle_boundary_);
          }
        } else if (indexes(i) == 2) {
          // move the range of minmax up
          // INFO("select bigger sample " << g_id << " " << i)
          gates_yaw_pitch_size_ranges[g_id - 1][i] =
              Vector<2>(current_range(0) + range_size_half,
                        current_range(1) + range_size_half);
          if (i < 2) {
            // limit yaw pitch change from gate direction to be within
            // -yaw_pitch_cone_angle_boundary_ and
            // yaw_pitch_cone_angle_boundary_
            gates_yaw_pitch_size_ranges[g_id - 1][i] =
                gates_yaw_pitch_size_ranges[g_id - 1][i]
                    .cwiseMax(-yaw_pitch_cone_angle_boundary_)
                    .cwiseMin(yaw_pitch_cone_angle_boundary_);
          }
        } else {
          // make smaller range around current sample
          // INFO("increase precision " << g_id << " " << i)
          gates_yaw_pitch_size_ranges[g_id - 1][i] =
              Vector<2>(current_range(0) + range_size_half / 2.0,
                        current_range(1) - range_size_half / 2.0);
        }
      }

      prev_sample_idx = shortest_samples_times[g_id][prev_sample_idx].first;
    }

    INFO(iterimp << "end state best time is " << shortest_time);

    /*
    // to DEBUG only - save the samples during iterations
    double time_sum_iter = 0;
    std::vector<TrMaxAcc3D> trajectories_to_save_iters;
    DroneState to_state;
    to_state.p = gates_[0].p;
    to_state.v = found_gates_speeds[0];
    TrMaxAcc3D tr_max_acc_start =
      calc_max_acc_thrust(start_, to_state, drone_, 0.001);
    trajectories_to_save_iters.push_back(tr_max_acc_start);
    time_sum_iter += tr_max_acc_start.time();

    if (!tr_max_acc_start.exists()) {
      INFO("should exists from start!!!!");
      exit(1);
    }
    for (size_t i = 1; i < found_gates_speeds.size(); i++) {
      DroneState from_state_b;
      from_state_b.p = gate_position_samples[i];
      from_state_b.v = found_gates_speeds[i - 1];
      DroneState to_state_b;
      to_state_b.p = gate_position_samples[i + 1];
      to_state_b.v = found_gates_speeds[i];
      TrMaxAcc3D tr_max_between =
        calc_max_acc_thrust(from_state_b, to_state_b, drone_, 0.001);
      trajectories_to_save_iters.push_back(tr_max_between);

      time_sum_iter += tr_max_between.time();
      if (!tr_max_between.exists()) {
        INFO("should exists between!!!! " << i);
        exit(1);
      }
    }
    if (fabs(time_sum_iter - shortest_time) > 0.0001) {
      INFO("time sum does not equal to the one found");
    }
    std::stringstream ss;
    ss << "iter_" << iterimp << "_samples.csv";
    save_track_trajectory(trajectories_to_save_iters, shortest_time, ss.str());
    */
  }

  // now that path is found, test collisions
  // paths_between_gates_

  timer.stop();

  // INFO("shortest time " << shortest_samples_times.size())
  //   INFO_VAR(gates_size + 1)
  //   INFO("index " << shortest_samples_times[gates_size + 1][0].first << "
  //   "
  //                 << shortest_samples_times[gates_size + 1][0].second)

  // find the shortest among the end ones
  double shortest_time = DBL_MAX;
  int end_best_idx = -1; // end idx is 0
  const int endi = shortest_samples_times.size() - 1;
  INFO_VAR(endi)
  for (size_t i = 0; i < shortest_samples_times[endi].size(); i++) {
    const double time = shortest_samples_times[endi][i].second;
    if (time < shortest_time) {
      shortest_time = time;
      end_best_idx = i;
    }
  }

  int prev_sample_idx = end_best_idx;
  for (int g_id = endi; g_id > 0; g_id--) {
    found_gates_speeds[g_id - 1] =
        std::get<0>(gate_velocity_samples[g_id][prev_sample_idx]);
    found_gates_times[g_id - 1] =
        shortest_samples_times[g_id][prev_sample_idx].second;

    prev_sample_idx = shortest_samples_times[g_id][prev_sample_idx].first;
  }

  INFO("end state best time is " << shortest_time);

  // INFO("endi " << endi)

  INFO_GREEN("num_calls_gd_acc " << num_calls_gd_acc)
  INFO_GREEN("time required gc calls " << timer.getTimeMS() << "ms")

  std::vector<TrMaxAcc3D> trajectories;

  double time_sum = 0;
  INFO_VAR(found_gates_speeds.size());
  INFO_VAR(gate_position_samples.size());
  INFO_VAR(gates_waypoints.size());

  INFO_VAR(gates_waypoints[0].position.transpose());
  INFO_VAR(gates_waypoints[0].velocity.transpose());

  DroneState from_start_state;
  from_start_state.p = gates_waypoints[0].position;
  from_start_state.v = gates_waypoints[0].velocity;
  DroneState to_state;
  to_state.p = gates_waypoints[1].position;
  to_state.v = found_gates_speeds[0];
  TrMaxAcc3D tr_max_acc_start =
      calc_max_acc_thrust(from_start_state, to_state, drone_, 0.001);
  trajectories.push_back(tr_max_acc_start);
  time_sum += tr_max_acc_start.time();

  if (!tr_max_acc_start.exists()) {
    INFO("should exists from start!!!!");
    exit(1);
  }

  for (size_t i = 2; i < gates_waypoints.size(); i++) {
    DroneState from_state_b;
    from_state_b.p = gates_waypoints[i - 1].position;
    from_state_b.v = found_gates_speeds[i - 2];
    DroneState to_state_b;
    to_state_b.p = gates_waypoints[i].position;
    to_state_b.v = found_gates_speeds[i - 1];
    TrMaxAcc3D tr_max_between =
        calc_max_acc_thrust(from_state_b, to_state_b, drone_, 0.001);
    trajectories.push_back(tr_max_between);

    time_sum += tr_max_between.time();
    if (!tr_max_between.exists()) {
      INFO("should exists between!!!! " << i);
      exit(1);
    }
  }

  INFO_GREEN("time_sum " << time_sum)

  if (fabs(time_sum - shortest_time) > 0.0001) {
    INFO("time sum does not equal to the one found");
    INFO_VAR(time_sum)
    INFO_VAR(shortest_time)
    exit(1);
  } else {
    INFO_GREEN("time equals")
  }

  for (size_t i = 0; i < trajectories.size(); i++) {
    // INFO("tr i " << i)
    const double max_time = trajectories[i].time();
    for (size_t axi = 0; axi < 3; axi++) {
      Tr1D scaled = one_dim_double_integrator_lim_vel(
          trajectories[i].get_axis(axi), max_time);
      trajectories[i].set_by_axis(axi, scaled);
    }
    // INFO(trajectories[i])
  }

  save_track_trajectory(trajectories, shortest_time,
                        output_folder_ + "samples_pmm.csv");

  // save_track_trajectory_equidistant(trajectories,
  // shortest_time,"samples_equidistant.csv");

  // std::vector<TrMaxAcc3D> single = {trajectories[0]};
  // save_track_trajectory(single, trajectories[0].time(), "samples.csv");
  // save_track_trajectory_equidistant(single, trajectories[0].time(),
  //                                   "samples_equidistant.csv");

  INFO_VAR(found_gates_speeds.size())
  INFO_VAR(gate_position_samples.size())

  INFO_CYAN("start pos " << gates_waypoints.front().position.transpose()
                         << " speed "
                         << gates_waypoints.front().velocity.transpose()
                         << " time " << 0)
  // INFO_VAR(gates_waypoints.size())
  // for (size_t i = 1; i < gates_waypoints.size(); i++) {
  //   INFO("gates_waypoints[" << i << "] " << gates_waypoints[i].is_gate
  //                           << " pos " << gates_waypoints[i].position << "
  //                           vel "
  //                           << gates_waypoints[i].velocity)
  // }

  for (int i = 1; i < gates_waypoints.size() - 1; i++) {
    // INFO(i)
    // INFO_VAR(gates_waypoints.size())
    // INFO_VAR((i < (((int)gates.size()) - 1)))
    if (!gates_waypoints[i].is_gate) {
      INFO_CYAN("waypoint " << i << " pos "
                            << gate_position_samples[i].transpose() << " speed "
                            << found_gates_speeds[i - 1].transpose() << " time "
                            << found_gates_times[i - 1])
    } else {
      INFO_CYAN("gate " << i << " pos " << gate_position_samples[i].transpose()
                        << " speed " << found_gates_speeds[i - 1].transpose()
                        << " time " << found_gates_times[i - 1])
    }
  }
  INFO_CYAN("end pos " << gates_waypoints.back().position.transpose()
                       << " speed " << found_gates_speeds.back().transpose()
                       << " time " << shortest_time)

  VelocitySearchResult res;
  res.time = shortest_time;
  res.found_gates_speeds = found_gates_speeds;
  res.trajectories = trajectories;
  res.gates_waypoints = gates_waypoints;

  return res;
}

void VelSearchGraph::save_track_trajectory_equidistant(
    const std::vector<TrMaxAcc3D> &trajectories, const double trajectories_time,
    std::string filename) {
  std::vector<std::vector<DroneState>> samples;
  samples.resize(1);
  const double ds_desired = 0.250;
  double trajectory_length = 0;
  // get the trajectories length
  for (size_t i = 0; i < trajectories.size(); i++) {
    const double ttime = trajectories[i].timemin();
    const double traj_length = trajectories[i].get_length(0, ttime);
    trajectory_length += traj_length;
  }

  int num_samples = ceil(trajectory_length / ds_desired);
  double ds = trajectory_length / ((double)(num_samples - 1));

  DroneState beginstate = trajectories[0].state_in_time(0);
  beginstate.command = Command(0, Vector<4>(0, 0, 0, 0), CommandType::NONE, 0,
                               Vector<3>(0, 0, 0));
  beginstate.t = 0;
  samples[0].push_back(beginstate);

  INFO_VAR(trajectory_length);

  int tr_id = 0;
  double tr_start_time = 0;
  double tr_end_time = trajectories[0].timemin();

  double lower_time = 0;
  double upper_time = 0;
  double dist_first = 0;
  double time_middle = 0;
  double t_current = 0;
  double t_from_start = 0;
  double accumulated_s = 0;
  for (size_t i = 1; i <= num_samples; i++) {
    const double s = i * ds;
    double target_ds = s - accumulated_s;
    // INFO("target_ds " << target_ds)

    // INFO("get length between " << t << " and " << tr_end_time)
    double dist_to_end = trajectories[tr_id].get_length(t_current, tr_end_time);
    // INFO("dist_to_end " << dist_to_end)
    if (dist_to_end < target_ds) {
      tr_id++;
      if (tr_id >= trajectories.size()) {
        break;
      }
      t_current = 0;
      t_from_start += tr_end_time;
      tr_end_time = trajectories[tr_id].timemin();
      target_ds -= dist_to_end;
      accumulated_s += dist_to_end;
    }

    lower_time = t_current;
    upper_time = tr_end_time;

    do {
      time_middle = (lower_time + upper_time) / 2.0;
      dist_first = trajectories[tr_id].get_length(t_current, time_middle);

      if (dist_first > target_ds) {
        // soulution between lower_time and time_middle
        // INFO("bellow")
        upper_time = time_middle;
      } else {
        // soulution between time_middle and upper_time
        // INFO("above")
        lower_time = time_middle;
      }
    } while (fabs(dist_first - target_ds) > 0.0000000001);

    // add also samples of acc switch
    std::vector<double> acc_switch_times;
    for (size_t axi = 0; axi < 3; axi++) {
      const double t1 = trajectories[tr_id].get_axis_switch_time(axi);
      // const double t1 = trajectories[tr_id].get_axis(axi).t1;
      if (t1 > t_current && t1 < time_middle) {
        acc_switch_times.push_back(t1);
      }
    }
    std::sort(acc_switch_times.begin(), acc_switch_times.end());

    // converged
    t_current = time_middle;
    accumulated_s += dist_first;

    // INFO("converged to time " << t_current);

    if (t_current < 0 || t_current > trajectories[tr_id].timemin()) {
      INFO("bad time");
      exit(1);
    }

    DroneState dronestate = trajectories[tr_id].state_in_time(t_current);
    if (fabs(dronestate.p(1)) > 30) {
      INFO("bad distance")
      exit(1);
    }
    dronestate.command =
        Command(t_from_start + t_current, Vector<4>(0, 0, 0, 0),
                CommandType::NONE, 0, Vector<3>(0, 0, 0));
    dronestate.t = t_from_start + t_current;
    dronestate.w(0) = accumulated_s; // hack

    const double dist_to_last = (samples[0].back().p - dronestate.p).norm();
    if (fabs(dist_to_last - ds) > 0.01) {
      INFO("bad distance change in save_track_trajectory_equidistant "
           << dist_to_last)
      INFO("sample " << i << " out of " << num_samples)
      INFO_VAR(samples[0].size())
      exit(1);
    }
    samples[0].push_back(dronestate);
  }

  DroneState::saveSamplesToFile(filename, samples);

  INFO("velocity search graph ended")
}

void VelSearchGraph::save_track_trajectory(
    const std::vector<TrMaxAcc3D> &trajectories, const double trajectories_time,
    std::string filename) {
  std::vector<std::vector<DroneState>> samples;
  samples.resize(1);
  const double dt_desired = 0.01;
  const int num_samples = ceil(trajectories_time / dt_desired);

  samples[0].resize(num_samples + 1);
  const double dt = trajectories_time / ((double)(num_samples));
  int tr_id = 0;
  double tr_start_time = 0;
  double tr_end_time = trajectories[0].time();
  for (size_t i = 0; i <= num_samples; i++) {
    // INFO(i << " num_samples " << num_samples)
    const double t = i * dt;
    // INFO("t " << t)
    if (t > tr_end_time) {
      // INFO("switch")
      tr_id++;
      if (tr_id >= trajectories.size()) {
        break;
      }
      tr_start_time = tr_end_time;
      tr_end_time = tr_end_time + trajectories[tr_id].time();
    }
    // INFO("t - tr_start_time " << (t - tr_start_time))
    // INFO("tr_id " << tr_id << " trajectories size " << trajectories.size())
    DroneState ds = trajectories[tr_id].state_in_time(t - tr_start_time);
    ds.command = Command(t, Vector<4>(0, 0, 0, 0), CommandType::NONE, 0,
                         Vector<3>(0, 0, 0));
    ds.t = t;
    samples[0][i] = ds;
  }

  DroneState::saveSamplesToFile(filename, samples);
}

VelocitySearchCollision
VelSearchGraph::isCollisionFree(const VelocitySearchResult &trajectory) {
  // INFO("test collision free trajectory begin")
  // INFO_RED(
  //   "here should be tested only starting from already known nun collision "
  //   "trajectory")
  VelocitySearchCollision vsc;
  for (size_t i = 0; i < trajectory.trajectories.size(); i++) {
    vsc = VelSearchGraph::isCollisionFree(trajectory.trajectories[i]);
    if (!vsc.collision_free) {
      vsc.trajectory_index = i;
      return vsc;
    }
  }
  return vsc;
}

VelocitySearchCollision
VelSearchGraph::isCollisionFree(const TrMaxAcc3D &trajectory) {
  // INFO("test collision free trajectory part begin")
  VelocitySearchCollision vsc;
  bool collision_free = true;
  if (!check_collisions_) {
    return vsc;
  }
  std::vector<Vector<3>> samples =
      sampleTrajectory(trajectory, collision_distance_check_);
  // INFO("have " << samples.size() << " samples")
  for (size_t i = 0; i < samples.size(); ++i) {
    const double clearance = map_->getClearence(samples[i]);
    // INFO("clearance " << clearance)
    // INFO_VAR(min_clearance_)
    if (!std::isfinite(clearance) || clearance < min_clearance_) {
      // INFO("collision")
      vsc.collision_free = false;
      vsc.position = samples[i];
      return vsc;
    }
  }

  return vsc;
}

std::vector<Vector<3>>
VelSearchGraph::sampleTrajectory(const TrMaxAcc3D &trajectory,
                                 const double ds_desired) {
  std::vector<Vector<3>> samples;
  // INFO("sample trajectory begin")

  // get the trajectories time and length
  const double ttime = trajectory.timemin();
  // INFO_VAR(ttime)
  const double trajectory_length = trajectory.get_length(0, ttime);
  // INFO_VAR(trajectory_length)

  // define num samples and ds
  int num_samples = ceil(trajectory_length / ds_desired);
  double ds = trajectory_length / ((double)(num_samples - 1));
  // INFO_VAR(num_samples)
  // INFO_VAR(ds)

  DroneState beginstate = trajectory.state_in_time(0);
  samples.push_back(beginstate.p);

  double tr_start_time = 0;
  double tr_end_time = ttime;

  double lower_time = 0;
  double upper_time = 0;
  double dist_first = 0;
  double time_middle = 0;
  double t_current = 0;
  double t_from_start = 0;
  double accumulated_s = 0;
  for (size_t i = 1; i < num_samples; i++) {
    const double s = i * ds;
    double target_ds = s - accumulated_s;

    // INFO("i " << i << " s " << s << " target_ds " << target_ds)
    lower_time = t_current;
    upper_time = tr_end_time;

    do {
      time_middle = (lower_time + upper_time) / 2.0;
      dist_first = trajectory.get_length(t_current, time_middle);

      if (dist_first > target_ds) {
        // soulution between lower_time and time_middle
        // INFO("bellow")
        upper_time = time_middle;
      } else {
        // soulution between time_middle and upper_time
        // INFO("above")
        lower_time = time_middle;
      }
    } while (fabs(dist_first - target_ds) > 0.001);

    /*
        // add also samples of acc switch
        std::vector<double> acc_switch_times;
        for (size_t axi = 0; axi < 3; axi++) {
          const double t1 = trajectory.get_axis_switch_time(axi);
          // const double t1 = trajectory.get_axis(axi).t1;
          if (t1 > t_current && t1 < time_middle) {
            acc_switch_times.push_back(t1);
          }
        }
        std::sort(acc_switch_times.begin(), acc_switch_times.end());
    */

    // converged
    t_current = time_middle;
    accumulated_s += dist_first;

    // INFO("converged to time " << t_current);

    if (t_current < 0 || t_current > trajectory.timemin()) {
      INFO("bad time");
      exit(1);
    }

    DroneState dronestate = trajectory.state_in_time(t_current);

    const double dist_to_last = (samples.back() - dronestate.p).norm();
    if (fabs(dist_to_last - ds) > ds) {
      INFO("bad distance change in sampleTrajectory " << dist_to_last)
      INFO("sample " << i << " out of " << num_samples)
      INFO_VAR(samples[0].size());
      INFO_VAR(target_ds);
      INFO_VAR(t_current)
      INFO_VAR(ttime)
      INFO_VAR(trajectory)
      INFO_VAR(dist_first)
      INFO_VAR(target_ds)
      INFO_VAR(fabs(dist_first - target_ds))
      exit(1);
    }
    samples.push_back(dronestate.p);
  }

  // INFO("sample trajectory end")
  return samples;
}