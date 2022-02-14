/*
 * point_rotation3d.h
 *
 *  Created on: Oct 23, 2020
 *      Author: Robert Penicka
 */

#pragma once
#include <math.h>

#include <algorithm>
#include <cfloat>
#include <eigen3/Eigen/Eigen>
#include <tuple>
#include <unordered_map>

#define CONVERGENCE_PREC (1E-3)

#include "common.hpp"
#include "drone.hpp"

// typedef struct PointSpeed2D PointSpeed2D;
typedef struct Tr1D Tr1D;
typedef struct TrMaxAcc3D TrMaxAcc3D;

double one_dim_double_integrator_min_acc(const double ps, const double vs,
                                         const double pe, const double ve);
// Tr1D one_dim_double_integrator(const double ps, const double vs,
//                                const double pe, const double ve,
//                                const double amax, const int i,
//                                const bool keep_acc_sign = false);

Tr1D one_dim_double_integrator_two_acc(const double ps, const double vs,
                                       const double pe, const double ve,
                                       const double a1, const double a2,
                                       const int i,
                                       const bool keep_acc_sign = false);

Tr1D one_dim_double_integrator_lim_vel(const double ps, const double vs,
                                       const double pe, const double ve,
                                       const double amax, const double vmax,
                                       const int i);

Tr1D one_dim_double_integrator_lim_vel(const Tr1D in, const double tot);

Tr1D one_dim_double_integrator_lim_vel_known_time(
  const double ps, const double vs, const double pe, const double ve,
  const double t, const double amax, const int i);

TrMaxAcc3D calc_max_acc_thrust(
  const DroneState &from_state, const DroneState &to_state, const Drone *drone,
  const double convergence_precision = CONVERGENCE_PREC);

void reproject_to_sphere(Vector<3> &new_thrust_acc, std::vector<bool> &fixed,
                         const Vector<3> &req_thrust_acc_min,
                         const Vector<3> &req_thrust_acc_max,
                         const Vector<3> &acc_req, const Vector<3> &t_times,
                         const double &a_max);

inline void get_time_avg_min(Scalar &tavg, Scalar &tmin, int &max_time_idx,
                             int &min_time_idx, const Scalar &tmax,
                             const Vector<3> &t_times,
                             const Vector<3> &gradients_scaled);

Scalar get_bodyacc_size_for_max_thrust(const Vector<3> &body_acc_dir,
                                       const Scalar a_max_thrust);

void test_acc_space(const Drone *drone);
void save_times_per_acc(const DroneState &from, const DroneState &to);
std::tuple<double, Vector<3>, Vector<3>,
           std::vector<std::pair<Vector<3>, double>>, int>
test_samling_thrust(const DroneState &from_state, const DroneState &to_state,
                    const double a_max);

template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const &matrix) const {
    // Note that it is oblivious to the storage
    // order of Eigen matrix (column- or
    // row-major). It will give you the same hash
    // value for two different matrices if they
    // are the transpose of each other in
    // different storage order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
              (seed >> 2);
    }
    return seed;
  }
};

typedef struct Tr1D {
  Tr1D() {
    exists = false;
    t1 = t2 = t3 = 0;
    p0 = p1 = p2 = p3 = 0;
    v0 = v1 = v3 = 0;
    i = 0;
    a1 = 0;
    a2 = 0;
    dt_da = 0;
  }
  double time() const { return t1 + t2 + t3; }
  std::tuple<Scalar, Scalar, Scalar> state_in_time(
    const Scalar time_in_tr) const {
    // INFO("state_in_time tr begin " << i << " " << time_in_tr)
    // INFO_VAR((t1 + t2 + t3))
    // INFO_VAR((time_in_tr < t1 + t2 + t3))
    Scalar pos, vel, acc;

    if (time_in_tr < t1) {
      // const acc part with self.a
      // INFO("a")
      pos = p0 + v0 * time_in_tr + 0.5 * a1 * time_in_tr * time_in_tr;
      vel = v0 + a1 * time_in_tr;
      acc = a1;
    } else if (time_in_tr < t1 + t2) {
      // const vel part with self.a
      // INFO("b")
      const Scalar time_part = (time_in_tr - t1);
      pos = p1 + v1 * time_part;
      vel = v1;
      acc = 0.0;
    } else if (time_in_tr < t1 + t2 + t3) {
      // INFO("c")
      // const vel part with self.a
      const Scalar time_part = (time_in_tr - t1 - t2);
      pos = p2 + v1 * time_part + 0.5 * a2 * time_part * time_part;
      vel = v1 + a2 * time_part;
      acc = a2;
    } else {
      // return the last state
      pos = p3;
      vel = v3;
      acc = a2;
    }
    if (t1 == 0 && t2 == 0 && t3 > 0) {
      acc = a2;
    } else if (t1 > 0 and t2 == 0 and t3 == 0) {
      acc = a1;
    }

    return {pos, vel, acc};
  }
  std::tuple<Tr1D, Tr1D> split_in_time(const Scalar time_in_tr) {
    auto [pos, vel, acc] = state_in_time(time_in_tr);
    //#print("get_time",self.get_time())
    //#print("time_in_tr",time_in_tr)
    Tr1D bef;
    bef.exists = true;
    bef.i = i;
    bef.a1 = a1;
    bef.a2 = a2;
    Tr1D aft;
    aft.exists = true;
    aft.i = i;
    aft.a1 = a1;
    aft.a2 = a2;

    INFO("split time_in_tr=" << time_in_tr << " self.t1=" << t1 << " diff "
                             << (time_in_tr - t1));
    if (time_in_tr <= t1) {  //} - EQUALITY_ERROR) {
      //  INFO("split 1 time_in_tr=" << time_in_tr << " self.t1=" << t1 << "
      //  diff "
      //                           << (time_in_tr - t1));
      INFO("split 1")
      bef.t1 = time_in_tr;
      bef.t2 = bef.t3 = .0;
      bef.t3 = .0;
      bef.p0 = p0;
      bef.p1 = bef.p2 = bef.p3 = pos;
      bef.v0 = v0;
      bef.v1 = bef.v3 = vel;

      aft.t1 = t1 - time_in_tr;
      aft.t2 = t2;
      aft.t3 = t3;
      aft.p0 = pos;
      aft.p1 = p1;
      aft.p2 = p2;
      aft.p3 = p3;
      aft.v0 = vel;
      aft.v1 = v1;
      aft.v3 = v3;
    } else if (time_in_tr <= t1 + t2) {
      INFO("split 2");
      bef.t1 = t1;
      bef.t2 = time_in_tr - t1;
      bef.t3 = .0;
      bef.p0 = p0;
      bef.p1 = p1;
      bef.p2 = bef.p3 = pos;
      bef.v0 = v0;
      bef.v1 = v1;
      bef.v3 = vel;

      aft.t1 = .0;
      aft.t2 = (t1 + t2) - time_in_tr;
      aft.t3 = t3;
      aft.p0 = aft.p1 = pos;
      aft.p2 = p2;
      aft.p3 = p3;
      aft.v0 = aft.v1 = vel;
      aft.v3 = v3;

    } else if (time_in_tr <= t1 + t2 + t3) {
      INFO("split 3");
      bef.t1 = t1;
      bef.t2 = t2;
      bef.t3 = time_in_tr - (t1 + t2);
      bef.p0 = p0;
      bef.p1 = p1;
      bef.p2 = p2;
      bef.p3 = pos;
      bef.v0 = v0;
      bef.v1 = v1;
      bef.v3 = vel;

      aft.t1 = aft.t2 = .0;
      aft.t3 = (t1 + t2 + t3) - time_in_tr;
      aft.p0 = aft.p1 = aft.p2 = pos;
      aft.p3 = p3;
      aft.v0 = aft.v1 = vel;
      aft.v3 = v3;
    } else {
      //#print("time",time_in_tr,"is outside of
      // the primitive") #quit()
      bef.t1 = t1;
      bef.t2 = t2;
      bef.t3 = t3;
      bef.p0 = p0;
      bef.p1 = p1;
      bef.p2 = p2;
      bef.p3 = p3;
      bef.v0 = v0;
      bef.v1 = v1;
      bef.v3 = v3;

      aft.t1 = aft.t2 = aft.t3 = .0;
      aft.p0 = aft.p1 = aft.p3 = pos;
      aft.v0 = aft.v1 = aft.v3 = vel;
    }
    return {bef, aft};
  }
  void save_to_file(std::string filename) {
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (myfile.is_open()) {
      myfile << "a1,a2,t1,t2,t3,p0,p1,p2,p3,v0,v1,"
                "v3,exists,dt_da,axis"
             << std::endl;
      myfile << a1 << "," << a2 << "," << t1 << "," << t2 << "," << t3 << ","
             << p0 << "," << p1 << "," << p2 << "," << p3 << "," << v0 << ","
             << v1 << "," << v3 << "," << exists << "," << dt_da << "," << i;
      myfile.close();
    }
  }
  bool exists;
  double t1, t2, t3;
  double p0, p1, p2, p3;
  double v0, v1, v3;
  double a1, a2;
  double dt_da;
  int i;
} Rr1D;


std::ostream &operator<<(std::ostream &o, const Tr1D &f);

typedef struct TrMaxAcc3D {
  TrMaxAcc3D() {}
  TrMaxAcc3D(Tr1D x_, Tr1D y_, Tr1D z_) {
    x = x_;
    y = y_;
    z = z_;
  }
  TrMaxAcc3D(Tr1D x_, Tr1D y_, Tr1D z_,
             std::unordered_map<Vector<3>, Quaternion, matrix_hash<Vector<3>>>
               q_for_acc_) {
    x = x_;
    y = y_;
    z = z_;
    q_for_acc = q_for_acc_;
  }
  void set_by_axis(const int i, Tr1D tr) {
    switch (i) {
      case 0:
        x = tr;
        break;
      case 1:
        y = tr;
        break;
      case 2:
        z = tr;
        break;
      default:
        exit(1);
    }
  }
  Tr1D &get_axis(const int i) {
    switch (i) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      default:
        exit(1);
    }
  }
  double get_axis_switch_time(const int i) const {
    switch (i) {
      case 0:
        return x.t1;
      case 1:
        return y.t1;
      case 2:
        return z.t1;
      default:
        exit(1);
    }
  }
  // Vector<3> get_acc() { return Vector<3>(x.a, y.a, z.a); }
  bool exists() const { return x.exists && y.exists && z.exists; }
  double time() const { return std::max({x.time(), y.time(), z.time()}); }
  double timemin() const { return std::min({x.time(), y.time(), z.time()}); }

  double get_length(const double tfrom, const double tto) const {
    // time switches in t1 for x,y,z
    // INFO("get_length call")
    std::vector<double> switch_times = {tfrom, tto};
    for (size_t i = 0; i < 3; i++) {
      const double switch_time = get_axis_switch_time(i);
      // INFO(i << " switch_time " << switch_time)
      if (switch_time > tfrom and switch_time < tto) {
        // sorted insert
        for (size_t swid = 1; swid < switch_times.size(); swid++) {
          if (switch_time < switch_times[swid]) {
            // INFO("add switch time " << switch_time)
            switch_times.insert(switch_times.begin() + swid, switch_time);
            break;
          }
        }
      }
    }

    // INFO("time() " << time())
    double ds = 0;
    for (size_t i = 1; i < switch_times.size(); i++) {
      const double tfrom_part = switch_times[i - 1];
      const double tto_part = switch_times[i];
      // INFO("tfrom_part " << tfrom_part << " tto_part " << tto_part)
      // const DroneState state_beg_part = this->state_in_time(tfrom_part);
      const auto [x_pos, x_vel, x_acc] = x.state_in_time(tfrom_part);
      const auto [y_pos, y_vel, y_acc] = y.state_in_time(tfrom_part);
      const auto [z_pos, z_vel, z_acc] = z.state_in_time(tfrom_part);
      const Vector<3> pos(x_pos, y_pos, z_pos);
      const Vector<3> vel(x_vel, y_vel, z_vel);
      const Vector<3> acc(x_acc, y_acc, z_acc);
      // INFO("pos " << pos.transpose() << " vel " << vel.transpose() << " acc "
      //             << acc.transpose())

      ds += get_length_const_a(0, tto_part - tfrom_part, pos, vel, acc);
    }
    // INFO("ds " << ds)
    return ds;
  }

  double get_length_const_a(const double tfrom, const double tto,
                            const Vector<3> p, const Vector<3> v,
                            const Vector<3> a) const {
    const double &sx = p(0);
    const double &sy = p(1);
    const double &sz = p(2);

    const double &vx = v(0);
    const double &vy = v(1);
    const double &vz = v(2);

    const double &ax = a(0);
    const double &ay = a(1);
    const double &az = a(2);

    // INFO("p " << p.transpose())
    // INFO("v " << v.transpose())
    // INFO("a " << a.transpose())

    const double ax_pow2 = ax * ax;
    const double ay_pow2 = ay * ay;
    const double az_pow2 = az * az;
    // const double tfrom_pow2 = tfrom * tfrom;
    // const double tfrom_pow3 = tfrom_pow2 * tfrom;
    // const double tfrom_pow4 = tfrom_pow3 * tfrom;
    // const double tto_pow2 = tto * tto;
    // const double tto_pow3 = tto_pow2 * tto;
    // const double tto_pow4 = tto_pow3 * tto;
    const double vx_pow2 = vx * vx;
    const double vy_pow2 = vy * vy;
    const double vz_pow2 = vz * vz;

    double t = tto;
    double t_pow2 = t * t;
    // INFO("t  " << t << " t_pow2 " << t_pow2)
    double tmp = sqrt(ax_pow2 * t_pow2 + ay_pow2 * t_pow2 + az_pow2 * t_pow2 +
                      2 * ax * t * vx + vx_pow2 + 2 * ay * t * vy + vy_pow2 +
                      2 * az * t * vz + vz_pow2);
    double logpart =
      log(ax_pow2 * t + ay_pow2 * t + az_pow2 * t + ax * vx + ay * vy +
          az * vz + sqrt(ax_pow2 + ay_pow2 + az_pow2) * tmp);
    if (vx == 0 && vy == 0 && vz == 0 && t == 0) {
      logpart = 0;
    }
    const double ds_to =
      (0.5) *
      (tmp *
         (t + (ax * vx + ay * vy + az * vz) / (ax_pow2 + ay_pow2 + az_pow2)) +
       (1.0 / pow(ax_pow2 + ay_pow2 + az_pow2, 1.5)) *
         ((az_pow2 * (vx_pow2 + vy_pow2) - 2.0 * ax * az * vx * vz -
           2.0 * ay * vy * (ax * vx + az * vz) + ay_pow2 * (vx_pow2 + vz_pow2) +
           ax_pow2 * (vy_pow2 + vz_pow2)) *
          logpart));


    t = tfrom;
    t_pow2 = t * t;
    // INFO("t  " << t << " t_pow2 " << t_pow2)
    tmp = sqrt(ax_pow2 * t_pow2 + ay_pow2 * t_pow2 + az_pow2 * t_pow2 +
               2 * ax * t * vx + vx_pow2 + 2 * ay * t * vy + vy_pow2 +
               2 * az * t * vz + vz_pow2);
    logpart = log(ax_pow2 * t + ay_pow2 * t + az_pow2 * t + ax * vx + ay * vy +
                  az * vz + sqrt(ax_pow2 + ay_pow2 + az_pow2) * tmp);
    if (vx == 0 && vy == 0 && vz == 0 && t == 0) {
      logpart = 0;
    }

    const double ds_from =
      (0.5) *
      (tmp *
         (t + (ax * vx + ay * vy + az * vz) / (ax_pow2 + ay_pow2 + az_pow2)) +
       (1.0 / pow(ax_pow2 + ay_pow2 + az_pow2, 1.5)) *
         ((az_pow2 * (vx_pow2 + vy_pow2) - 2.0 * ax * az * vx * vz -
           2.0 * ay * vy * (ax * vx + az * vz) + ay_pow2 * (vx_pow2 + vz_pow2) +
           ax_pow2 * (vy_pow2 + vz_pow2)) *
          logpart));

    const double ds = ds_to - ds_from;

    // INFO("ds_to  " << ds_to << " ds_from  " << ds_from << " ds " << ds)

    if (!isfinite(ds) || ds < 0) {
      INFO("non finite ds or bellow zero")
      INFO("p " << p.transpose())
      INFO("v " << v.transpose())
      INFO("a " << a.transpose())
      exit(1);
    }
    return ds;
  }

  DroneState state_in_time(const Scalar time_in_tr) const {
    // INFO("state_in_time point speed " << time_in_tr)
    DroneState ds;
    auto [x_pos, x_vel, x_acc] = x.state_in_time(time_in_tr);
    auto [y_pos, y_vel, y_acc] = y.state_in_time(time_in_tr);
    auto [z_pos, z_vel, z_acc] = z.state_in_time(time_in_tr);
    // INFO("axises found")

    ds.p(0) = x_pos;
    ds.p(1) = y_pos;
    ds.p(2) = z_pos;

    ds.v(0) = x_vel;
    ds.v(1) = y_vel;
    ds.v(2) = z_vel;

    ds.a(0) = x_acc;
    ds.a(1) = y_acc;
    ds.a(2) = z_acc;

    // INFO_VAR(ds.a)
    // G = np.array([[ 0.0, 0.0, Drone.G_const
    // ]]).T acc =  np.array([[x_acc, y_acc,
    // z_acc]]).T

    Vector<3> cur_acc(x_acc, y_acc, z_acc);
    // INFO_VAR(cur_acc.transpose())
    if (q_for_acc.find(cur_acc) != q_for_acc.end()) {
      const Quaternion new_q_to = q_for_acc[cur_acc];
      ds.qx(0) = new_q_to.w();
      ds.qx(1) = new_q_to.x();
      ds.qx(2) = new_q_to.y();
      ds.qx(3) = new_q_to.z();
    } else {
      Vector<3> thrust_acc = Vector<3>(x_acc, y_acc, z_acc) - GVEC;
      Quaternion q_target_thrust =
        Quaternion::FromTwoVectors(Vector<3>(0, 0, 1), thrust_acc);
      auto [new_q_to, q_xy] =
        decompose_xy_z(Quaternion(1, 0, 0, 0), q_target_thrust);
      ds.qx(0) = new_q_to.w();
      ds.qx(1) = new_q_to.x();
      ds.qx(2) = new_q_to.y();
      ds.qx(3) = new_q_to.z();
    }
    ds.w(0) = ds.w(1) = ds.w(2) = 0;  // angular speed
    return ds;
  }
  DroneState get_end_state() {  // INFO("end state")
    return state_in_time(time());
  }
  DroneState get_start_state() {  // INFO("start state")
    return state_in_time(0);
  }
  std::tuple<TrMaxAcc3D, TrMaxAcc3D> split_in_time(const Scalar time_in_tr) {
    auto [x_bef, x_aft] = x.split_in_time(time_in_tr);
    auto [y_bef, y_aft] = y.split_in_time(time_in_tr);
    auto [z_bef, z_aft] = z.split_in_time(time_in_tr);
    TrMaxAcc3D bef(x_bef, y_bef, z_bef, q_for_acc);
    TrMaxAcc3D aft(x_aft, y_aft, z_aft, q_for_acc);

    return {bef, aft};
  }

  std::tuple<TrMaxAcc3D, TrMaxAcc3D> split_acc_switch_by_axis(
    const int axis_id) {
    Scalar sw_time = get_axis_switch_time(axis_id);
    auto [x_bef, x_aft] = x.split_in_time(sw_time);
    auto [y_bef, y_aft] = y.split_in_time(sw_time);
    auto [z_bef, z_aft] = z.split_in_time(sw_time);
    TrMaxAcc3D bef(x_bef, y_bef, z_bef, q_for_acc);
    TrMaxAcc3D aft(x_aft, y_aft, z_aft, q_for_acc);

    return {bef, aft};
  }

  void add_quaternion_for_time(const Scalar time_in_tr, const Quaternion q) {
    auto [x_pos, x_vel, x_acc] = x.state_in_time(time_in_tr);
    auto [y_pos, y_vel, y_acc] = y.state_in_time(time_in_tr);
    auto [z_pos, z_vel, z_acc] = z.state_in_time(time_in_tr);
    Vector<3> acc(x_acc, y_acc, z_acc);
    q_for_acc[acc] = q;
  }
  mutable std::unordered_map<Vector<3>, Quaternion, matrix_hash<Vector<3>>>
    q_for_acc;
  Tr1D x;
  Tr1D y;
  Tr1D z;
  int gate_idx;
} TrMaxAcc3D;
std::ostream &operator<<(std::ostream &o, const TrMaxAcc3D &f);
