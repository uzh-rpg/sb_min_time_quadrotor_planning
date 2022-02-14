/*
 * point_speed3d.h
 *
 *  Created on: Oct 23, 2020
 *      Author: Robert Penicka
 */

#pragma once

#include <math.h>

#include <eigen3/Eigen/Eigen>

#include "common.hpp"
#include "drone.hpp"
#include "point_speed3d.hpp"

// typedef struct PointSpeed2D PointSpeed2D;
typedef struct TrR1D TrR1D;
typedef struct TrRMaxAcc3D TrRMaxAcc3D;

TrR1D calc_limited_acc_lim_vel_rot1d(const double ps, const double vs,
                                     const double pe, const double ve,
                                     const double amax, const double vmax,
                                     const int i);

TrRMaxAcc3D calc_pitch_roll(const Quaternion &q_from, const Quaternion &q_to,
                            const Vector<3> from_omegas,
                            const Vector<3> to_omegas,
                            const Vector<3> from_state_p,
                            const Vector<3> from_state_v, const Drone *drone,
                            const double rot_acc, const double rot_vel);

typedef struct TrR1D {
  TrR1D() {
    exists = false;
    t1 = t2 = t3 = 0;
    p0 = p1 = p2 = p3 = 0;
    v0 = v1 = v3 = 0;
    i = 0;
    a = 0;
  }
  std::tuple<Scalar, Scalar, Scalar> state_in_time(const Scalar time_in_tr) {
    Scalar pos, vel, acc;
    // INFO("rot state_in_time " << time_in_tr)

    if (time_in_tr <= t1) {
      // const acc part with self.a
      // print("a")
      pos = p0 + v0 * time_in_tr + 0.5 * a * time_in_tr * time_in_tr;
      vel = v0 + a * time_in_tr;
      acc = a;
    } else if (time_in_tr <= t1 + t2) {
      const Scalar time_part = (time_in_tr - t1);
      pos = p1 + v1 * time_part;
      vel = v1;
      acc = 0;
    } else if (time_in_tr <= t1 + t2 + t3) {
      const Scalar time_part = (time_in_tr - t1 - t2);
      pos = p2 + v1 * time_part - 0.5 * a * time_part * time_part;
      vel = v1 - a * time_part;
      acc = -a;
    } else {
      // return the last state
      pos = p3;
      vel = v3;
      acc = -a;
    }
    return {pos, vel, acc};
  }
  bool exists;
  double t1, t2, t3;
  double p0, p1, p2, p3;
  double v0, v1, v3;
  double a;
  int i;
} TrR1D;


std::ostream &operator<<(std::ostream &o, const TrR1D &f);

typedef struct TrRMaxAcc3D {
  TrRMaxAcc3D() {}
  TrRMaxAcc3D(TrR1D rotation_, Vector<3> rotation_axis_, Quaternion q_from_,
              Vector<3> from_state_p_, Vector<3> from_state_v_) {
    rotation = rotation_;
    rotation_axis = rotation_axis_.normalized();
    q_from = q_from_;
    from_state_p = from_state_p_;
    from_state_v = from_state_v_;
  }
  void set_by_axis(int i, TrR1D tr) {
    switch (i) {
      case 0:
        rotation = tr;
        break;
      default:
        exit(1);
    }
  }
  bool exists() const { return rotation.exists; }
  double time() const { return rotation.t1 + rotation.t2 + rotation.t3; }
  void set_from_state(Vector<3> from_state_p_, Vector<3> from_state_v_) {
    from_state_p = from_state_p_;
    from_state_v = from_state_v_;
  }
  DroneState state_in_time(const Scalar time_in_tr) {
    // INFO("state_in_time " << time_in_tr);
    DroneState state;

    Scalar time_tr = time();

    auto [theta_pos, theta_vel, theta_acc] = rotation.state_in_time(time_in_tr);

    // INFO_VAR(theta_pos)
    // INFO_VAR(theta_vel)
    // INFO_VAR(theta_acc)

    state.x.segment<DroneState::IDX::NPOS>(DroneState::IDX::POS) =
      from_state_p;  //+ time_in_tr * self.from_state_v
    state.x.segment<DroneState::IDX::NVEL>(DroneState::IDX::VEL) = from_state_v;
    //#q_target_thrust =
    // myquaternion.qBetweenVec(np.array([[0,0,1]]).T,np.array([[x_acc,y_acc,z_acc]]).T)
    // INFO("rotation_axis " << rotation_axis.transpose());
    //#INFO("theta_pos",theta_pos)
    const Vector<3> q_now_vec_part = rotation_axis * sin(theta_pos / 2.0);
    const Scalar q_now_real = cos(theta_pos / 2.0);
    //#INFO("q_now_real",q_now_real)
    //#INFO("q_now_vec_part",q_now_vec_part)
    const Quaternion q_now(q_now_real, q_now_vec_part(0), q_now_vec_part(1),
                           q_now_vec_part(2));
    const Quaternion q = q_from * q_now;
    state.qx(0) = q.w();
    state.qx(1) = q.x();
    state.qx(2) = q.y();
    state.qx(3) = q.z();
    // INFO("state.qx " << state.qx.transpose());
    state.x.segment<DroneState::IDX::NOME>(DroneState::IDX::OME) =
      theta_vel * rotation_axis;
    //#state.w = np.array([[r_vel, p_vel, 0]]).T  # angular speed
    return state;
  }
  DroneState get_end_state() {  // INFO("end state")
    return state_in_time(time());
  }
  DroneState get_start_state() {  // INFO("start state")
    return state_in_time(0);
  }

  int gate_idx;
  TrR1D rotation;
  Vector<3> rotation_axis;
  Quaternion q_from;
  Vector<3> from_state_p;
  Vector<3> from_state_v;
} TrRMaxAcc3D;

std::ostream &operator<<(std::ostream &o, const TrRMaxAcc3D &f);
std::ostream &operator<<(std::ostream &o, const Quaternion &q);
