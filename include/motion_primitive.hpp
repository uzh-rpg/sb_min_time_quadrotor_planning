/*
 * motion_primitive.hpp
 *
 *  Created on: Mar 7, 2021
 *      Author: Robert Penicka
 */

#pragma once

#include <tuple>
#include <vector>

#include "point_rotation3d.hpp"
#include "point_speed3d.hpp"

struct Primitive;

class MotionPrimitive {
 public:
  static Primitive acc_primitive(const DroneState &from_state,
                                 const DroneState &to_state,
                                 const Drone *drone);

  static Primitive fix_rotation_continuity(Primitive &old, const Drone *drone);

  static void check_primitive(Primitive &pr, bool check_quaternions = false);

  static TrRMaxAcc3D calc_rot_trajectories(const Quaternion &q_from,
                                           const Quaternion &q_to,
                                           const DroneState &state_in_switch,
                                           const Vector<3> omega_after,
                                           const Drone *drone);
  static std::vector<Command> get_motor_commands(const Primitive primitive,
                                                 Drone *drone);
  static std::vector<Command> get_motor_command_rot(const TrRMaxAcc3D rot,
                                                    const int c_id,
                                                    Drone *drone);
  static Command get_motor_command_trans(const TrMaxAcc3D rot, const int c_id,
                                         const Drone *drone);
  static std::vector<std::vector<DroneState>> get_samples(
    Primitive primitive, std::vector<std::tuple<int, int>> &gate_primitives,
    const double desired_num_samples, const double rotation_sample_mult_ratio,
    Drone *drone);
  static std::vector<std::vector<DroneState>> get_samples_plain(
    Primitive primitive,
    std::vector<std::tuple<int, int>> &gate_primitive_sizes,
    const double desired_dt, Drone *drone);
};

typedef struct Primitive {
  std::vector<TrMaxAcc3D> translations;
  std::vector<TrRMaxAcc3D> rotations;
  Scalar time;
  static Primitive connect_primitives(Primitive &first, Primitive &second,
                                      const Drone *drone) {
    // INFO("connect_primitives begin");

    // INFO("connect1 " << first.rotations.size() << " "
    //                 << " trans " << first.translations.size())
    // INFO("connect2 " << second.rotations.size() << " "
    //                 << " trans " << second.translations.size())
    Primitive res;
    res.translations = first.translations;
    res.translations.insert(res.translations.end(), second.translations.begin(),
                            second.translations.end());

    res.rotations.insert(res.rotations.end(), first.rotations.begin(),
                         first.rotations.end() - 1);
    const DroneState last_rot_start = first.rotations.back().get_start_state();
    const Quaternion last_rot_start_q = last_rot_start.getAttitude();
    const DroneState first_rot_end = second.rotations.front().get_end_state();
    const Quaternion first_rot_end_q = first_rot_end.getAttitude();
    TrRMaxAcc3D rot_between = MotionPrimitive::calc_rot_trajectories(
      last_rot_start_q, first_rot_end_q, last_rot_start, Vector<3>::Zero(),
      drone);
    rot_between.gate_idx = first.rotations.back().gate_idx;
    res.rotations.push_back(rot_between);
    res.rotations.insert(res.rotations.end(), second.rotations.begin() + 1,
                         second.rotations.end());
    res.time = res.calc_time();
    // INFO("res " << res.rotations.size() << " "
    //           << " trans " << res.translations.size())
    // INFO("connect_primitives end");
    return res;
  }

  Scalar calc_time() {
    Scalar time = 0;
    for (int var = 0; var < translations.size(); ++var) {
      time += translations[var].time();
    }
    for (int var = 0; var < rotations.size(); ++var) {
      time += rotations[var].time();
    }
    return time;
  }

  void setGateId(int gate_id) {
    for (int var = 0; var < translations.size(); ++var) {
      time += translations[var].gate_idx = gate_id;
    }
    for (int var = 0; var < rotations.size(); ++var) {
      time += rotations[var].gate_idx = gate_id;
    }
  }
} Primitive;
