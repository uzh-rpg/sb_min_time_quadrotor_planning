/*
 * point_rotation3d.cpp
 *
 *  Created on: Oct 23, 2020
 *      Author: robert
 */

#include "point_rotation3d.hpp"

#include <algorithm>
#include <iostream>
#include <limits>

#define PRECISION_ROT3D (1.0e-4)

TrR1D calc_limited_acc_lim_vel_rot1d(const double ps, const double vs,
                                     const double pe, const double ve,
                                     const double amax, const double vmax,
                                     const int i) {
  TrR1D tr;
  tr.i = i;
  tr.p0 = ps;
  tr.p3 = pe;
  tr.v0 = vs;
  tr.v3 = ve;
  tr.exists = true;
  double a = amax;
  const double v = std::copysign(vmax, pe - ps);

  if (fabs(pe - ps) < PRECISION_ROT3D) {
    if (vs != ve) {
      INFO("can not change pos with not v diff");
      tr.a = 0.0;
      tr.exists = false;
      return tr;
    } else {
      tr.a = 0;
      tr.p0 = tr.p1 = tr.p2 = tr.p3 = ps;
      tr.v0 = tr.v1 = tr.v3 = vs;
      tr.exists = true;
      return tr;
    }
  }

  if (v - vs < 0) {
    a = -amax;
  }
  INFO_VAR(a)
  if (2 * a * pe - 2 * a * ps + pow(ve, 2) - 2 * pow(v, 2) + pow(vs, 2) >= 0) {
    const double t1 = (v - vs) / a;
    const double t2 =
      (2 * a * pe - 2 * a * ps + pow(ve, 2) - 2 * pow(v, 2) + pow(vs, 2)) /
      (2 * a * v);
    const double t3 = (-ve + v) / a;
    const double p1 = ps + t1 * vs + 0.5 * t1 * t1 * (a);
    const double p2 = p1 + t2 * v;

    if (t1 < 0 or t2 < 0 or t3 < 0) {
      INFO(t1 << t2 << t3);
      exit(1);
    }
    tr.t1 = t1;
    tr.t2 = t2;
    tr.t3 = t3;
    tr.p1 = p1;
    tr.p2 = p2;
    tr.v1 = v;
    tr.a = a;
  } else {
    tr.exists = false;
    Tr1D unlimited_vel =
      one_dim_double_integrator_two_acc(ps, vs, pe, ve, amax, -amax, 0);

    INFO_RED("has to calculate max acc rotation without reaching max vel "
             << unlimited_vel)
    // INFO_VAR(v)
    // exit(1);
    if (fabs(unlimited_vel.v1) <= fabs(v)) {
      tr.t1 = unlimited_vel.t1;
      tr.t2 = unlimited_vel.t2;
      tr.t3 = unlimited_vel.t3;
      tr.p1 = unlimited_vel.p1;
      tr.p2 = unlimited_vel.p2;
      tr.v1 = unlimited_vel.v1;
      tr.a = unlimited_vel.a1;
      // INFO_VAR(unlimited_vel)
      // exit(1);
      tr.exists = true;
    } else {
      INFO(unlimited_vel)
      INFO("rotation does not exists rot");
      INFO("ps:" << ps << ";vs:" << vs << ";pe:" << pe << ";ve:" << ve
                 << ";amax:" << amax << ";vmax:" << vmax);
      exit(1);
    }
  }
  return tr;
}

TrRMaxAcc3D calc_pitch_roll(const Quaternion &q_from, const Quaternion &q_to,
                            const Vector<3> from_omegas,
                            const Vector<3> to_omegas,
                            const Vector<3> from_state_p,
                            const Vector<3> from_state_v, const Drone *drone,
                            const double rot_acc, const double rot_vel) {
  // INFO("q_from" << q_from.w() << " " << q_from.vec().transpose());
  // INFO("q_to" << q_to.w() << " " << q_to.vec().transpose());
  // INFO("from_omegas" << from_omegas.transpose());
  // INFO("to_omegas" << to_omegas.transpose());

  const Quaternion q_from_to = q_from.inverse() * q_to;
  // INFO("q_from_to" << q_from_to.w() << " " << q_from_to.vec().transpose());
  const AngleAxis ang_ax(q_from_to);

  const Vector<3> rot_vec = ang_ax.axis();
  const Scalar rot_angle = ang_ax.angle();
  // INFO("rot_vec " << rot_vec.transpose());
  /*
   const Vector<3> from_omegas = from_state.w;
   const Vector<3> state_bef_p = from_state.p;
   const Vector<3> state_bef_v = from_state.v;
   const Vector<3> to_omegas = to_state.w;
  */
  const Scalar from_omega = rot_vec.transpose() * from_omegas;
  const Scalar to_omega = rot_vec.transpose() * to_omegas;

  if (fabs(from_omega) > PRECISION_ROT3D || fabs(to_omega) > PRECISION_ROT3D) {
    ERROR("from_omega " << from_omega);
    ERROR("to_omega " << to_omega);
    ERROR("from_omega or to_omega are not zero");
  }


  TrR1D tr_rot = calc_limited_acc_lim_vel_rot1d(0, from_omega, rot_angle,
                                                to_omega, rot_acc, rot_vel, 0);
  // TrR1D rotation_, Vector<3> rotation_axis_, Quaternion q_from_,
  //              Vector<3> from_state_p_, Vector<3> from_state_v_
  TrRMaxAcc3D result(tr_rot, rot_vec, q_from, from_state_p, from_state_v);
  return result;
}


std::ostream &operator<<(std::ostream &o, const TrR1D &f) {
  o << "maxrot: t1:" << f.t1 << ";t2:" << f.t2 << ";t3:" << f.t3
    << ";exists:" << f.exists << ";a:" << f.a << ";i:" << f.i << "\n";
  o << "      : p0:" << f.p0 << ";p1:" << f.p1 << ";p2:" << f.p2
    << ";p3:" << f.p3 << "\n";
  o << "      : v0:" << f.v0 << ";v1:" << f.v1 << ";v3:" << f.v3 << "t_tot"
    << (f.t1 + f.t2 + f.t3);
  return o;
}

std::ostream &operator<<(std::ostream &o, const TrRMaxAcc3D &f) {
  o << "maxacc3d: t:" << f.time() << ";exists:" << f.exists();
  o << "\n\trotation: " << f.rotation;
  o << "\n\tr_axis: " << f.rotation_axis.transpose();
  return o;
}

std::ostream &operator<<(std::ostream &o, const Quaternion &q) {
  o << q.w() << " " << q.vec().transpose();
  return o;
}
