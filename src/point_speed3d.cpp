/*
 * point_speed3d.cpp
 *
 *  Created on: Oct 23, 2020
 *      Author: robert
 */

#include "point_speed3d.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <sstream>

#define PRECISION_TRAN3D (1.0e-8)
#define PRECISION_TRANS3D (1E-4)
#define ZERO_ACC (1E-3)
#define MIN_REQUIRED_ACC (0.5)

#define ALLOWED_DIFF_TIMES_RATIO (0.0001)
#define NUM_ITERS (30)
#define PRINT_DEBUG (false)
#define MIN_ACC_REQ (0.01)

double one_dim_double_integrator_min_acc(const double ps, const double vs,
                                         const double pe, const double ve) {
  // required acc to be able to fulfill the task
  // i.e. accelerate between required velocities and in the same time not
  // overshoot the position derived from
  // t_change_vel = (ve-vs)/amax
  // and pe-ps = vs*t_change_vel + 0.5*amax*t_change_vel**2
  // from that what is the amax....
  // INFO("oddiminacc s:" << ps << ";vs:" << vs << ";pe:" << pe << ";ve:" <<
  // ve);
  if (fabs(pe - ps) < PRECISION_TRANS3D) {
    return MIN_ACC_REQ;
  } else {
    const double pow_ve2 = ve * ve;
    const double pow_vs2 = vs * vs;
    if (fabs(pow_ve2 - pow_vs2) < PRECISION_TRANS3D) {
      return std::copysign(MIN_REQUIRED_ACC, pe - ps);
    } else {
      const double a_min_req = (-pow_ve2 + pow_vs2) / (2 * (-pe + ps));
      return a_min_req;
    }
  }
}

Tr1D one_dim_double_integrator_two_acc(const double ps, const double vs,
                                       const double pe, const double ve,
                                       const double a1_in, const double a2_in,
                                       const int i, const bool keep_acc_sign) {
  // INFO_COND(PRINT_DEBUG, "oddi two acc ps->"
  //                          << ps << ",vs->" << vs << ",pe->" << pe << ",ve->"
  //                          << ve << ",a1->" << a1_in << ",a2->" << a2_in
  //                          << ";i:" << i << ";keep_acc_sign:" <<
  //                          keep_acc_sign);

  Tr1D tr;
  tr.i = i;
  tr.p0 = ps;
  tr.p3 = pe;
  tr.v0 = vs;
  tr.v3 = ve;

  if (fabs(a1_in) <= PRECISION_TRANS3D && fabs(a2_in) <= PRECISION_TRANS3D) {
    tr.exists = false;
    return tr;
  }

  const double pow_ve2 = ve * ve;
  const double pow_vs2 = vs * vs;

  if (fabs(pe - ps) < PRECISION_TRANS3D && fabs(ve - vs) < PRECISION_TRANS3D) {
    tr.a1 = a1_in;
    tr.a2 = a2_in;
    tr.p0 = tr.p1 = tr.p2 = tr.p3 = ps;
    tr.v0 = tr.v1 = tr.v3 = vs;
    tr.exists = true;
    return tr;
  }


  // INFO("a " << a);
  double t1 = DBL_MAX;
  double t2 = DBL_MAX;
  double dt_da = DBL_MAX;
  // INFO_VAR(a_test)
  double used_acc1 = a1_in;
  double used_acc2 = a2_in;
  std::vector<std::pair<double, double>> test_acc_vec = {{a1_in, a2_in},
                                                         {a2_in, a1_in}};
  if (keep_acc_sign) {
    test_acc_vec = {{a1_in, a2_in}};
  }
  for (auto [a1, a2] : test_acc_vec) {
    // INFO("test a " << a)

    const double pow_a2_2 = a2 * a2;
    const double pow_a1_2 = a1 * a1;
    const double pow_a1_3 = pow_a1_2 * a1;

    const double tst1 = sqrt(
      (-a1 + a2) * (2 * a1 * a2 * (pe - ps) - a1 * pow_ve2 + a2 * pow_vs2));
    const double tst2 = sqrt(
      (a1 - a2) * (a1 * (-2 * a2 * pe + 2 * a2 * ps + pow_ve2) - a2 * pow_vs2));

    // case 1
    const double t1_1 = (-(a1 * vs) + a2 * vs + tst1) / (a1 * (a1 - a2));
    const double t2_1 = -((-(a1 * ve) + a2 * ve + tst2) / ((a1 - a2) * a2));

    // case 2
    const double t1_2 = -((a1 * vs - a2 * vs + tst1) / (a1 * (a1 - a2)));
    const double t2_2 = (a1 * ve - a2 * ve + tst2) / ((a1 - a2) * a2);

    // INFO("a1 " << a1 << " a2 " << a2)
    // INFO("t1_1 " << t1_1 << " t2_1 " << t2_1 << " sum " << (t1_1 + t2_1));
    // INFO("t1_2 " << t1_2 << " t2_2 " << t2_2 << " sum " << (t1_2 + t2_2));


    if (isfinite(t1_1) and isfinite(t2_1) and t1_1 > -PRECISION_TRAN3D and
        t2_1 > -PRECISION_TRAN3D and t1_1 + t2_1 < t1 + t2) {
      t1 = std::max(t1_1, 0.0);
      t2 = std::max(t2_1, 0.0);

      // dt/da == gradient we can use to optimize time
      const double d_t_da1 = (a1 * (2 * a2 * (pe - ps) - pow_ve2 - pow_vs2) +
                              2 * vs * (a2 * vs + tst1)) /
                             (2 * pow_a1_2 * tst1);
      const double d_t_da2 = (2 * a1 * (a2 * (-pe + ps) + pow_ve2) -
                              a2 * (pow_ve2 + pow_vs2) - 2 * ve * tst2) /
                             (2 * pow_a2_2 * tst1);

      // INFO("d_t_da1 " << d_t_da1 << " d_t_da2 " << d_t_da2 << " for a1 " <<
      // a1
      //                 << " a2 " << a2 << " for i " << i)
      if (i < 2) {
        dt_da = std::copysign(d_t_da1, -a1) +
                std::copysign(d_t_da2, -a1);  // gradient with respect to a1
      } else {
        dt_da = std::copysign(d_t_da1, -1) + std::copysign(d_t_da2, -1);
      }
      used_acc1 = a1;
      used_acc2 = a2;
    }
    if (isfinite(t1_2) and isfinite(t2_2) and t1_2 > -PRECISION_TRAN3D and
        t2_2 > -PRECISION_TRAN3D and
        (t1 == DBL_MAX || t1_2 + t2_2 < t1 + t2)) {
      t1 = std::max(t1_2, 0.0);
      t2 = std::max(t2_2, 0.0);


      // dt/da == gradient we can use to optimize time
      const double d_t_da1 =
        (a1 * (-2 * a2 * pe + 2 * a2 * ps + pow_ve2 + pow_vs2) +
         2 * vs * (-(a2 * vs) + tst1)) /
        (2 * pow_a1_2 * tst1);
      const double d_t_da2 = (2 * a1 * a2 * (pe - ps) - 2 * a1 * pow_ve2 +
                              a2 * (pow_ve2 + pow_vs2) - 2 * ve * tst2) /
                             (2 * pow_a2_2 * tst1);

      // INFO("d_t_da1 " << d_t_da1 << " d_t_da2 " << d_t_da2 << " for a1 " <<
      // a1
      //                 << " a2 " << a2 << " for i " << i)

      if (i < 2) {
        dt_da = std::copysign(d_t_da1, -a1) +
                std::copysign(d_t_da2, -a1);  // gradient with respect to a1
      } else {
        dt_da = std::copysign(d_t_da1, -1) +
                std::copysign(d_t_da2, -1);  // a1 always positive
      }
      used_acc1 = a1;
      used_acc2 = a2;
    }
  }
  //}


  if (t1 >= 0 and t1 != DBL_MAX) {
    // INFO("used_acc " << used_acc << " t1 " << t1 << " t2 " << t2 << " sum "
    //                  << (t1 + t2))
    tr.exists = true;
    tr.t1 = t1;
    tr.t2 = 0;
    tr.t3 = t2;
    tr.p1 = ps + t1 * vs + 0.5 * used_acc1 * t1 * t1;
    tr.p2 = tr.p1;
    tr.v1 = vs + used_acc1 * t1;
    tr.a1 = used_acc1;
    tr.a2 = used_acc2;
    tr.dt_da = std::isfinite(dt_da) ? dt_da : 0;
    // INFO(tr.dt_da);
    const double ve_tst = tr.v1 + tr.a2 * t2;
    const double pe_tst = tr.p2 + t2 * tr.v1 + 0.5 * tr.a2 * t2 * t2;
    if (fabs(ve_tst - ve) > PRECISION_TRANS3D ||
        fabs(pe_tst - pe) > PRECISION_TRANS3D) {
      INFO_RED("wrong ve or pe oddi two acc");
      INFO_RED("ve_tst " << ve_tst << " ve " << ve);
      INFO_RED("pe_tst " << pe_tst << " pe " << pe);
      INFO_RED("t1 " << t1 << " t2 " << t2 << " a1 " << used_acc1 << " a2 "
                     << used_acc2)
      tr.exists = false;
      return tr;
    }

    /*
        if (fabs(tr.dt_da) > tr.t1 + tr.t3) {
          if (i < 2) {
            tr.dt_da = std::copysign((tr.t1 + tr.t3) / 10.0, -tr.a1);
          } else {
            tr.dt_da = std::copysign((tr.t1 + tr.t3) / 10.0, -1);
          }
        }
    */

  } else {
    tr.exists = false;
    // INFO_RED("not found oddi two");
  }

  return tr;
}

Tr1D one_dim_double_integrator_lim_vel(const Tr1D in, const double tot) {
  if (in.time() == tot) {
    return in;
  }

  const double &ps = in.p0;
  const double &pe = in.p3;
  const double &vs = in.v0;
  const double &ve = in.v3;
  const double &a1 = in.a1;
  const double &a2 = in.a2;
  const int &i = in.i;
  const double pow_ve2 = ve * ve;
  const double pow_vs2 = vs * vs;

  const double pow_tot2 = tot * tot;

  const double part = (2 * a1 * pe - 2 * a2 * pe - 2 * a1 * ps + 2 * a2 * ps -
                       2 * a1 * tot * ve + 2 * a2 * tot * vs);
  const double sqrt_part = sqrt(
    part * part - 4 * a1 * a2 * pow_tot2 * (pow_ve2 - 2 * ve * vs + pow_vs2));
  const double scale1 =
    (-2 * a1 * pe + 2 * a2 * pe + 2 * a1 * ps - 2 * a2 * ps +
     2 * a1 * tot * ve - 2 * a2 * tot * vs - sqrt_part) /
    (2 * a1 * a2 * pow_tot2);
  const double scale2 =
    (-2 * a1 * pe + 2 * a2 * pe + 2 * a1 * ps - 2 * a2 * ps +
     2 * a1 * tot * ve - 2 * a2 * tot * vs + sqrt_part) /
    (2 * a1 * a2 * pow_tot2);

  // INFO_VAR(scale1)
  // INFO_VAR(scale2)
  Tr1D tr;
  if (scale1 < 1.0 && scale1 > -1.0) {
    tr = one_dim_double_integrator_two_acc(ps, vs, pe, ve, a1 * scale1,
                                           a2 * scale1, i, true);
  } else if (scale2 < 1.0 && scale2 > -1.0) {
    tr = one_dim_double_integrator_two_acc(ps, vs, pe, ve, a1 * scale2,
                                           a2 * scale2, i, true);
  } else {
    if (fabs(scale1 - 1.0) < 1e-10 || fabs(scale2 - 1.0) < 1e-10) {
      tr = in;
      INFO_VAR(tot - in.time())
      if (tr.t1 > 0) {
        tr.t1 += tot - tr.time();
      } else if (tr.t3 > 0) {
        tr.t3 += tot - tr.time();
      } else {
        INFO("error scale time")

        exit(1);
      }
      INFO_VAR(tot - tr.time())
    }
    INFO_VAR(scale1)
    INFO_VAR(scale2)

    // exit(1);
  }
  // INFO("diff time " << tr.time() - tot)
  return tr;
}

Tr1D one_dim_double_integrator_lim_vel(const double ps, const double vs,
                                       const double pe, const double ve,
                                       const double amax, const double vmax,
                                       const int i) {
  double a = amax;
  if (vmax - vs < 0) {
    a = -amax;
  }
  if (2 * a * pe - 2 * a * ps + pow(ve, 2) - 2 * pow(vmax, 2) + pow(vs, 2) >=
      0) {
    const double t1 = (vmax - vs) / a;
    const double t2 =
      (2 * a * pe - 2 * a * ps + pow(ve, 2) - 2 * pow(vmax, 2) + pow(vs, 2)) /
      (2 * a * vmax);
    const double t3 = (-ve + vmax) / a;
    const double p1 = ps + t1 * vs + 0.5 * t1 * t1 * (a);
    const double p2 = p1 + t2 * vmax;

    Tr1D tr;
    if (t1 < 0 or t2 < 0 or t3 < 0) {
      tr.exists = false;
      exit(1);
    } else {
      // return t1, t2, t3, p1, p2, vmax, a
      tr.exists = true;
      tr.t1 = t1;
      tr.t2 = t2;
      tr.t3 = t3;
      tr.p1 = p1;
      tr.p2 = p2;
      tr.v1 = vmax;
      tr.a1 = a;
      tr.a2 = -a;
    }
    return tr;
  } else {
    return one_dim_double_integrator_two_acc(ps, vs, pe, ve, amax, -amax, i);
  }
}

Tr1D one_dim_double_integrator_lim_vel_known_time(
  const double ps, const double vs, const double pe, const double ve,
  const double t, const double amax, const int i) {
  //# vmax = -vmax

  double t1 = -1;
  double t2 = -1;
  double t3 = -1;
  double p1 = NAN;
  double p2 = NAN;
  double v1 = NAN;
  double a = amax;
  double a_used = a;

  double tst1 = pow(a, 2) * (-4 * a * pe + 4 * a * ps + pow(a, 2) * pow(t, 2) +
                             2 * a * t * ve - pow(ve, 2) + 2 * a * t * vs +
                             2 * ve * vs - pow(vs, 2));
  if (tst1 >= 0) {
    const double t1_1 = (a * t + ve - vs - sqrt(tst1) / a) / (2 * a);
    const double t2_1 =
      sqrt(pow(a, 2) *
           (-4 * a * pe + 4 * a * ps + pow(a, 2) * pow(t, 2) + 2 * a * t * ve -
            pow(ve, 2) + 2 * a * t * vs + 2 * ve * vs - pow(vs, 2))) /
      pow(a, 2);
    const double t3_1 =
      (2 * pow(a, 2) * t - 2 * a * ve + 2 * a * vs -
       sqrt(pow(-2 * pow(a, 2) * t + 2 * a * ve - 2 * a * vs, 2) -
            8 * pow(a, 2) *
              (2 * a * pe - 2 * a * ps - 2 * a * t * ve + pow(ve, 2) -
               2 * ve * vs + pow(vs, 2)))) /
      (4 * pow(a, 2));
    if (t1_1 >= 0 and t2_1 >= 0 and t3_1 >= 0) {
      t1 = t1_1;
      t2 = t2_1;
      t3 = t3_1;
      a_used = a;
    }

    const double t1_2 = (a * t + ve - vs + sqrt(tst1) / a) / (2 * a);
    const double t2_2 =
      -(sqrt(pow(a, 2) * (-4 * a * pe + 4 * a * ps + pow(a, 2) * pow(t, 2) +
                          2 * a * t * ve - pow(ve, 2) + 2 * a * t * vs +
                          2 * ve * vs - pow(vs, 2))) /
        pow(a, 2));
    const double t3_2 =
      (2 * pow(a, 2) * t - 2 * a * ve + 2 * a * vs +
       sqrt(pow(-2 * pow(a, 2) * t + 2 * a * ve - 2 * a * vs, 2) -
            8 * pow(a, 2) *
              (2 * a * pe - 2 * a * ps - 2 * a * t * ve + pow(ve, 2) -
               2 * ve * vs + pow(vs, 2)))) /
      (4 * pow(a, 2));
    if (t1_2 >= 0 and t2_2 >= 0 and t3_2 >= 0) {
      t1 = t1_2;
      t2 = t2_2;
      t3 = t3_2;
      a_used = a;
    }
  }

  a = -amax;
  double tst2 = pow(a, 2) * (-4 * a * pe + 4 * a * ps + pow(a, 2) * pow(t, 2) +
                             2 * a * t * ve - pow(ve, 2) + 2 * a * t * vs +
                             2 * ve * vs - pow(vs, 2));
  if (tst2 >= 0) {
    const double t1_3 = (a * t + ve - vs - sqrt(tst2) / a) / (2 * a);
    const double t2_3 =
      sqrt(pow(a, 2) *
           (-4 * a * pe + 4 * a * ps + pow(a, 2) * pow(t, 2) + 2 * a * t * ve -
            pow(ve, 2) + 2 * a * t * vs + 2 * ve * vs - pow(vs, 2))) /
      pow(a, 2);
    const double t3_3 =
      (2 * pow(a, 2) * t - 2 * a * ve + 2 * a * vs -
       sqrt(pow(-2 * pow(a, 2) * t + 2 * a * ve - 2 * a * vs, 2) -
            8 * pow(a, 2) *
              (2 * a * pe - 2 * a * ps - 2 * a * t * ve + pow(ve, 2) -
               2 * ve * vs + pow(vs, 2)))) /
      (4 * pow(a, 2));
    if (t1_3 >= 0 and t2_3 >= 0 and t3_3 >= 0) {
      t1 = t1_3;
      t2 = t2_3;
      t3 = t3_3;
      a_used = a;
    }

    const double t1_4 = (a * t + ve - vs + sqrt(tst2) / a) / (2 * a);
    const double t2_4 =
      -(sqrt(pow(a, 2) * (-4 * a * pe + 4 * a * ps + pow(a, 2) * pow(t, 2) +
                          2 * a * t * ve - pow(ve, 2) + 2 * a * t * vs +
                          2 * ve * vs - pow(vs, 2))) /
        pow(a, 2));
    const double t3_4 =
      (2 * pow(a, 2) * t - 2 * a * ve + 2 * a * vs +
       sqrt(pow(-2 * pow(a, 2) * t + 2 * a * ve - 2 * a * vs, 2) -
            8 * pow(a, 2) *
              (2 * a * pe - 2 * a * ps - 2 * a * t * ve + pow(ve, 2) -
               2 * ve * vs + pow(vs, 2)))) /
      (4 * pow(a, 2));
    if (t1_4 >= 0 and t2_4 >= 0 and t3_4 >= 0) {
      t1 = t1_4;
      t2 = t2_4;
      t3 = t3_4;
      a_used = a;
    }
  }

  Tr1D tr;
  if (t1 > 0) {
    v1 = vs + a_used * t1;
    //# print("v1",v1)
    p1 = ps + t1 * vs + 0.5 * a_used * pow(t1, 2);
    p2 = p1 + t2 * v1;
    // t1, t2, t3, p1, p2, v1, a_used
    tr.exists = true;
    tr.t1 = t1;
    tr.t2 = t2;
    tr.t3 = t3;
    tr.p1 = p1;
    tr.p2 = p2;
    tr.v1 = v1;
    tr.a1 = a_used;
    tr.a2 = -a_used;
  } else {
    //# return None
    INFO("ajajaaaaj2");
    tr.exists = false;
    exit(1);
  }

  return tr;
  // return t1, t2, t3, p1, p2, v1, a_used
}

Scalar get_bodyacc_size_for_max_thrust(const Vector<3> &body_acc_dir,
                                       const Scalar a_max_thrust) {
  // So we know the Gravity acc and the body minimal bodyacc direction and
  // correcposnding req_thrust_acc. We would like to find the body acc size
  // that maximizes thrust acc

  // knowns are, body_a dir, G dir and size, thrust_a size
  // question is how large is body a with maximal thrust_a size

  // we can use cosine equation with angle between body acc dir and G
  // a_max^2 = |G|^2 + |body_a|^2 - 2*|G|*|body_a|*cos(bodya_G_angle)

  // angle between the G and body acc (body_axx = Thrust_acc+ G)
  const Scalar cos_cang =
    body_acc_dir.normalized().transpose() * GVEC.normalized();

  // from cosine equation we get discriminant
  const Scalar discrim =
    4 * (G * G) * (cos_cang * cos_cang) - 4 * G * G +
    4 * a_max_thrust * a_max_thrust;  // a_max is the thrust acc
  const Scalar a1 = (2 * G * cos_cang + sqrt(discrim)) / 2.0;
  const Scalar a2 = (2 * G * cos_cang - sqrt(discrim)) / 2.0;

  // length of the final body acceleration initially in direction of the
  // required thrust where the
  const Scalar alen = a1 > 0 ? a1 : a2;
  // INFO("a1 " << a1 << " a2" << a2)
  return alen;
}

TrMaxAcc3D calc_max_acc_thrust(const DroneState &from_state,
                               const DroneState &to_state, const Drone *drone,
                               const double convergence_precision) {
  static int maxsuccess_iters = 0;
  static int maxunimproved_iters = 0;
  // optimize the thrust such that maxtime(x,y,z) is minimal

  // TODO:
  // - check what happen if min req acc is set for two axises, the remaining
  // acc size on the remaining axis should acieve smaller time than both the
  // first two

  INFO_COND(PRINT_DEBUG, "-------------------------------------------");
  INFO_COND(PRINT_DEBUG, "calc_max_acc_thrust begin")
  INFO_COND(PRINT_DEBUG, "from_state p:" << from_state.p.transpose()
                                         << " v:" << from_state.v.transpose());
  INFO_COND(PRINT_DEBUG, "to_state p:" << to_state.p.transpose()
                                       << " v:" << to_state.v.transpose());


  TrMaxAcc3D tr_max_acc;
  Vector<3> t_times(0.0, 0.0, 0.0);
  Vector<3> gradients(0.0, 0.0, 0.0);

  // first get the minimal required acc per axis
  Vector<3> acc_req(MIN_ACC_REQ, MIN_ACC_REQ, MIN_ACC_REQ);

  for (int i = 0; i < 3; ++i) {
    const double vs = from_state.v(i);
    const double ve = to_state.v(i);
    const double ps = from_state.p(i);
    const double pe = to_state.p(i);
    acc_req(i) = one_dim_double_integrator_min_acc(ps, vs, pe, ve);
    if (acc_req(i) < 0) {
      acc_req(i) = std::min(acc_req(i), -MIN_ACC_REQ);
    } else {
      acc_req(i) = std::max(acc_req(i), MIN_ACC_REQ);
    }


    Tr1D tr_above = one_dim_double_integrator_two_acc(
      ps, vs, pe, ve, acc_req(i) * 1.1, -acc_req(i) * 1.1, i);
    if (tr_above.exists) {
      acc_req(i) = std::copysign(acc_req(i), tr_above.a1);
      t_times(i) = tr_above.time();
      gradients(i) = tr_above.dt_da;
    } else {
      INFO("non existing min acc should not happen i:" << i)
      INFO("acc_req(i) " << acc_req(i))
      INFO(tr_above)
      exit(1);
    }
    Tr1D tr_bellow = one_dim_double_integrator_two_acc(
      ps, vs, pe, ve, acc_req(i) * 0.9, -acc_req(i) * 0.9, i);
    if (tr_bellow.exists) {
      INFO_COND(PRINT_DEBUG, "tr bellow exists " << i << " a1 " << tr_bellow.a1
                                                 << " a2 " << tr_bellow.a2)
      acc_req(i) = std::copysign(MIN_ACC_REQ, acc_req(i));
    } else {
      INFO("bellow does not exists")
    }
  }

  acc_req(2) = fabs(acc_req(2));


  INFO_COND(PRINT_DEBUG,
            "acc_req " << acc_req.transpose() << " norm " << acc_req.norm());

  const double a_max = 4 * drone->max_t_motor_ / drone->m_;
  // INFO("a_max " << a_max)
  const Vector<3> req_max_thrust_acc =
    acc_req.cwiseAbs() +
    Vector<3>(0, 0,
              G);  // we need abs of the az + g to be within the limit....
  INFO_COND(PRINT_DEBUG, "req_max_thrust_acc abs "
                           << req_max_thrust_acc.transpose() << " norm "
                           << req_max_thrust_acc.norm() << " a_max " << a_max);
  // INFO("amax " << a_max)
  // unfeasible if over the max thrust of quad

  if (req_max_thrust_acc.norm() > a_max) {
    INFO("req_max_thrust_acc above amax")
  }

  if (req_max_thrust_acc.norm() > a_max) {
    tr_max_acc.x.exists = false;
    tr_max_acc.y.exists = false;
    tr_max_acc.z.exists = false;
    INFO_COND_COLOR(PRINT_DEBUG, OUTPUT_BLUE,
                    "req_max_thrust_acc above limit "
                      << req_max_thrust_acc.norm() << " a_max " << a_max);

    return tr_max_acc;
  }

  // add G to min thrust acc
  Vector<3> req_thrust_acc_min;
  Vector<3> req_thrust_acc_max;
  for (size_t i = 0; i < 3; i++) {
    if (i == 2) {
      // z
      if (acc_req(i) > 0) {
        req_thrust_acc_min(i) = acc_req(i) + G;
        req_thrust_acc_max(i) = a_max;
      } else {
        req_thrust_acc_max(i) = acc_req(i) + G;
        req_thrust_acc_min(i) = -a_max;
      }
    } else {
      // x, y
      if (acc_req(i) > 0) {
        req_thrust_acc_min(i) = acc_req(i);
        req_thrust_acc_max(i) = a_max;
      } else {
        req_thrust_acc_max(i) = acc_req(i);
        req_thrust_acc_min(i) = -a_max;
      }
    }
  }

  INFO_COND(PRINT_DEBUG,
            "req_thrust_acc_max " << req_thrust_acc_max.transpose())
  INFO_COND(PRINT_DEBUG,
            "req_thrust_acc_min " << req_thrust_acc_min.transpose())
  // INFO_VAR(acc_req.transpose())


  Vector<3> thrust_acc = acc_req - GVEC;
  INFO_COND(PRINT_DEBUG, "thrust_acc non scaled " << thrust_acc.transpose())
  // scale the thrust acc to max value
  std::vector<bool> fixed_init{false, false, false};
  reproject_to_sphere(thrust_acc, fixed_init, req_thrust_acc_min,
                      req_thrust_acc_max, acc_req, t_times, a_max);

  INFO_COND(PRINT_DEBUG, "thrust_acc scaled " << thrust_acc.transpose())

  if (fabs(thrust_acc.norm() - a_max) > 0.01) {
    INFO("bad thrust acc norm")
    exit(1);
  }

  // for debbuging checking bound
  for (size_t i = 0; i < 3; i++) {
    if (thrust_acc(i) < req_thrust_acc_min(i) ||
        thrust_acc(i) > req_thrust_acc_max(i)) {
      ERROR_RED("out of thrust acc bounds in beggining, thrust is "
                << thrust_acc.transpose())
      INFO("req_thrust_acc_max " << req_thrust_acc_max.transpose())
      INFO("req_thrust_acc_min " << req_thrust_acc_min.transpose())
      INFO("from_state p:" << from_state.p.transpose()
                           << " v:" << from_state.v.transpose());
      INFO("to_state p:" << to_state.p.transpose()
                         << " v:" << to_state.v.transpose());
      exit(1);
    }
  }


  double min_tdiff_min_max = std::numeric_limits<double>::max();

  int iter_unimproved_max = 0;
  int last_improved_iter = 0;

  double tmax = std::numeric_limits<double>::max();
  double tmin, tavg;
  double tmax_old = tmax;


  Vector<3> gradients_old = gradients;
  Vector<3> t_times_old = t_times;
  double dalph = 0;
  double dalph_old = dalph;
  // Vector<3> dalph(0.0, 0.0, 0.0);
  // Vector<3> dalph_old = dalph;
  double default_min_time_change_decay = 1.0;
  double min_time_change_decay = default_min_time_change_decay;
  double decay_decrease_min_time_swap = 1.0;
  double decay_decrease_max_time_swap = 0.8;
  double decay_increase_tmax = 0.9;
  double decay_decrease_three_constrained = 0.2;
  int max_time_idx, max_time_idx_old = 0;
  int min_time_idx, min_time_idx_old = 0;
  double min_tmax_iters = tmax;
  TrMaxAcc3D min_tmax_iters_tr;

  bool converged = true;
  const int num_iter_opt = NUM_ITERS;
  int iter = 0;
  for (; iter < num_iter_opt; iter++) {
    INFO_COND(PRINT_DEBUG, "------------------")
    INFO_COND(PRINT_DEBUG, "iter " << iter)

    converged = true;

    // copy to old vars
    t_times_old = t_times;
    tmax_old = tmax;
    gradients_old = gradients;
    max_time_idx_old = max_time_idx;
    min_time_idx_old = min_time_idx;
    dalph_old = dalph;

    // check if thrust_acc is max
    if (fabs(thrust_acc.norm() - a_max) > 0.01) {
      INFO("bad thrust_dir_size " << thrust_acc.norm() << " vs " << a_max);
      INFO_VAR(iter)
      INFO_VAR(thrust_acc.norm())
      INFO_VAR(a_max)
      exit(1);
    }

    thrust_acc =
      thrust_acc.cwiseMin(req_thrust_acc_max).cwiseMax(req_thrust_acc_min);

    const Vector<3> body_acc = thrust_acc + GVEC;
    const Vector<3> body_acc_down = -thrust_acc + GVEC;
    // acc_req = req_thrust_acc + GVEC;
    for (int i = 0; i < 3; ++i) {
      /*
      if (fabs(body_acc(i)) < fabs(acc_req(i)) &&
          fabs(body_acc(i) - acc_req(i)) < 0.001) {
        body_acc(i) = acc_req(i);
      }
    */

      const double max_acc = body_acc(i);
      const double min_acc = body_acc_down(i);
      const double vs = from_state.v(i);
      const double ve = to_state.v(i);
      const double ps = from_state.p(i);
      const double pe = to_state.p(i);

      const Tr1D tr =
        one_dim_double_integrator_two_acc(ps, vs, pe, ve, max_acc, min_acc, i);
      // INFO(i << " dtda " << tr.dt_da)

      if (!tr.exists) {
        ERROR_RED("iterated to nonexisting one_dim_double_integrator");
        INFO("axis " << i)
        INFO_VAR(tr.a1)
        INFO_VAR(tr.a2)
        INFO_VAR(max_acc)

        INFO_VAR(req_thrust_acc_max.transpose())
        INFO_VAR(req_thrust_acc_min.transpose())
        INFO_VAR(acc_req.transpose())
        INFO_VAR(acc_req.norm())
        INFO_VAR(body_acc.transpose())
        INFO_VAR(thrust_acc.transpose())
        INFO_VAR(thrust_acc.norm())
        INFO_VAR(a_max)
        INFO_VAR(acc_req.transpose())
        exit(1);
      }

      t_times(i) = tr.time();
      gradients(i) = tr.dt_da;
      tr_max_acc.set_by_axis(i, tr);

      // if (i == 2) {
      //   gradients(i) = copysign(gradients(i), -1);
      // }

      // the best a may change sign
      if (i != 2 && acc_req(i) * tr.a1 < 0) {
        thrust_acc(i) = tr.a1;
        acc_req(i) = copysign(acc_req(i), tr.a1);
        const double tmp_acc_max = req_thrust_acc_max(i);
        const double tmp_acc_min = req_thrust_acc_min(i);
        req_thrust_acc_max(i) = -tmp_acc_min;
        req_thrust_acc_min(i) = -tmp_acc_max;
      }
    }

    // use gradient inverse, i.e. da/dt
    Vector<3> gradients_scaled = gradients;  //.cwiseInverse();
    // gradients_scaled.cwiseMax(a_max / 4.0).cwiseMin(-a_max / 4.0);

    tmax = t_times.maxCoeff();

    get_time_avg_min(tavg, tmin, max_time_idx, min_time_idx, tmax, t_times,
                     gradients_scaled);


    INFO_COND_COLOR(PRINT_DEBUG, OUTPUT_CYAN, "tmax " << tmax)
    INFO_COND_COLOR(
      PRINT_DEBUG, OUTPUT_CYAN,
      "t_times " << t_times(0) << " " << t_times(1) << " " << t_times(2))
    INFO_COND(PRINT_DEBUG, "gradients " << gradients(0) << " " << gradients(1)
                                        << " " << gradients(2))
    INFO_COND(PRINT_DEBUG, "gradients_scaled init "
                             << gradients_scaled(0) << " "
                             << gradients_scaled(1) << " "
                             << gradients_scaled(2))
    INFO_COND(PRINT_DEBUG, "thrust_acc " << thrust_acc(0) << " "
                                         << thrust_acc(1) << " "
                                         << thrust_acc(2))
    INFO_COND(PRINT_DEBUG, "tmin " << tmin)


    Vector<3> t_times_diff = (t_times_old - t_times);
    t_times_diff = t_times_diff.cwiseAbs().cwiseMax(
      0.0005);  // cap it not tu be too close to 0 for gradient amplification
    Vector<3> gradients_diff =
      (gradients_old - gradients).cwiseAbs().cwiseMax(0.0005);

    INFO_COND(PRINT_DEBUG, "t_times_diff " << t_times_diff.transpose())
    bool on_limit = true;


    // setting gradient to zero if on the limit of acc
    while (on_limit) {
      on_limit = false;
      for (int i = 0; i < 3; i++) {
        if (t_times(i) == 0) {
          gradients_scaled(i) = 0;
        }

        if (gradients_scaled(i) != 0) {
          if ((acc_req(i) >= 0 &&
               fabs(thrust_acc(i) - req_thrust_acc_min(i)) < 0.001 &&
               t_times(i) < tavg) ||
              (acc_req(i) <= 0 &&
               fabs(thrust_acc(i) - req_thrust_acc_max(i)) < 0.001 &&
               t_times(i) < tavg)) {
            // on the acc limit while wanting to go further (acc_req different
            // sing from gradient)

            gradients_scaled(i) = 0;

            INFO_COND_COLOR(
              PRINT_DEBUG, OUTPUT_RED,
              "is on limit of descent, do not consider for  avg.... " << i)
            INFO_COND(PRINT_DEBUG, "acc_req " << acc_req.transpose())


            on_limit = true;
            break;
          }
        }
      }
      get_time_avg_min(tavg, tmin, max_time_idx, min_time_idx, tmax, t_times,
                       gradients_scaled);
    }


    if (iter - last_improved_iter > iter_unimproved_max) {
      iter_unimproved_max = iter - last_improved_iter;
    }
    const double tdiff_min_max = tmax - tmin;
    if (tdiff_min_max < min_tdiff_min_max) {
      min_tdiff_min_max = tdiff_min_max;
      last_improved_iter = iter;
    }


    // end convergence if gradient is zero of >1 axises
    int num_on_limit = 0;
    for (size_t i = 0; i < 3; i++) {
      if (gradients_scaled(i) == 0) {
        num_on_limit++;
      }
    }
    if (num_on_limit > 1) {
      converged = true;
      break;
    }

    // scale the gradient based on time diff to last and signed distance from
    // average
    for (int i = 0; i < 3; i++) {
      double dist_t_avg = fabs(tavg - t_times(i));
      if (t_times(i) > tavg) {
        // INFO("inverse for i " << i)
        gradients_scaled(i) = -gradients_scaled(i) * dist_t_avg /
                              (fabs(t_times_diff(i)) * gradients_diff(i));
      } else {
        // INFO("do not inverse for i " << i)
        gradients_scaled(i) = gradients_scaled(i) * dist_t_avg /
                              (fabs(t_times_diff(i)) * gradients_diff(i));
      }
    }


    INFO_COND(PRINT_DEBUG, "gradients_scaled bef " << gradients_scaled(0) << " "
                                                   << gradients_scaled(1) << " "
                                                   << gradients_scaled(2))
    INFO_COND(PRINT_DEBUG,
              "tmax " << tmax << " tavg " << tavg << " tmin " << tmin)

    // scale alpha angle by distance from average and the dacay
    dalph = min_time_change_decay * (fabs(tmax - tavg) / tmax);
    if (dalph == 0) {
      converged = true;
      break;
    }
    dalph = std::min(0.6, dalph);  // limit the angle
    INFO_COND(PRINT_DEBUG, "dalph " << dalph)

    // so now we have gradients_scaled and need to make them
    // dot(gradients_scaled,thrust_acc_u)==0 and
    // gradient_tang_plane.dot(thrust_acc_u) == 0 &&
    // gradient_tang_plane.norm
    // == thrust acc
    // so we will fix the zeros first

    gradients_scaled = gradients_scaled.normalized() * a_max;

    INFO_COND(PRINT_DEBUG, "gradients_scaled normalized "
                             << gradients_scaled(0) << " "
                             << gradients_scaled(1) << " "
                             << gradients_scaled(2))

    // the ones that reached the limit are fixed by force == their scale can
    // not be changed
    std::vector<double> fixed_tang_plane_force = {false, false, false};

    double dot_fixed = 0;
    double g_len_fixed = a_max * a_max;
    std::vector<double> fixed_tang_plane = {false, false, false};
    std::vector<int> nf_ids;
    Vector<3> gradient_tang_plane(0, 0, 0);
    for (size_t i = 0; i < 3; i++) {
      if (gradients_scaled(i) == 0) {
        // we have to make acc = acc*cos(alph) + gtp*cos(alph) too keep the
        // acc at the same value
        INFO_COND_COLOR(PRINT_DEBUG, OUTPUT_MAGENTA, "on limit for idx " << i)
        fixed_tang_plane[i] = true;
        fixed_tang_plane_force[i] = true;
        gradient_tang_plane(i) =
          (thrust_acc(i) - cos(dalph) * thrust_acc(i)) / sin(dalph);

        dot_fixed -= gradient_tang_plane(i) * thrust_acc(i);
        g_len_fixed -= gradient_tang_plane(i) * gradient_tang_plane(i);

      } else {
        nf_ids.push_back(i);
      }
    }


    if (fixed_tang_plane[0] + fixed_tang_plane[1] + fixed_tang_plane[2] == 1) {
      //  now the gradient_tang_plane non fixed parts are set to be
      //  solutions to the
      // system of equations
      // 0 = a_0*g_0 + a_1*g_1 + a_2*g_2
      // a_max^2 = g_0^2 + g_1^2 + g_2^2
      const double acc0_pow2 = (thrust_acc(nf_ids[0]) * thrust_acc(nf_ids[0]));
      const double acc1_pow2 = (thrust_acc(nf_ids[1]) * thrust_acc(nf_ids[1]));
      const double c = -g_len_fixed + (dot_fixed * dot_fixed) / (acc0_pow2);
      const double b = -2.0 * dot_fixed * thrust_acc(nf_ids[1]) / acc0_pow2;
      const double a = (acc1_pow2 + acc0_pow2) / acc0_pow2;
      const double disc = b * b - 4.0 * a * c;
      const double g1_first = (-b + sqrt(disc)) / (2.0 * a);
      const double g1_second = (-b - sqrt(disc)) / (2.0 * a);

      const double g0_first =
        (dot_fixed - thrust_acc(nf_ids[1]) * g1_first) / thrust_acc(nf_ids[0]);
      const double g0_second =
        (dot_fixed - thrust_acc(nf_ids[1]) * g1_second) / thrust_acc(nf_ids[0]);

      // first solution of the quadratic function is the correct one == the
      // one with same sign as gradient scaled


      if (g1_first * gradients_scaled(nf_ids[1]) >= 0 &&
          g0_first * gradients_scaled(nf_ids[0]) >= 0) {
        gradient_tang_plane(nf_ids[1]) = g1_first;
        gradient_tang_plane(nf_ids[0]) = g0_first;
      } else if (g1_second * gradients_scaled(nf_ids[1]) >= 0 &&
                 g0_second * gradients_scaled(nf_ids[0]) >= 0) {
        gradient_tang_plane(nf_ids[1]) = g1_second;
        gradient_tang_plane(nf_ids[0]) = g0_second;
      } else {
        if (max_time_idx == nf_ids[0] &&
            (g0_first * gradients_scaled(nf_ids[0]) >= 0 ||
             g0_second * gradients_scaled(nf_ids[0]) >= 0)) {
          // select the one solution that preserves sign of max_time_idx
          // INFO("here1")
          if (g0_first * gradients_scaled(nf_ids[0]) >= 0) {
            gradient_tang_plane(nf_ids[1]) = g1_first;
            gradient_tang_plane(nf_ids[0]) = g0_first;
          } else if (g0_second * gradients_scaled(nf_ids[0]) >= 0) {
            gradient_tang_plane(nf_ids[1]) = g1_second;
            gradient_tang_plane(nf_ids[0]) = g0_second;
          }
        } else if (max_time_idx == nf_ids[1] &&
                   (g1_first * gradients_scaled(nf_ids[1]) >= 0 ||
                    g1_second * gradients_scaled(nf_ids[1]) >= 0)) {
          // select the one solution that preserves sign of max_time_idx
          // INFO("here2")
          if (g1_first * gradients_scaled(nf_ids[1]) >= 0) {
            gradient_tang_plane(nf_ids[1]) = g1_first;
            gradient_tang_plane(nf_ids[0]) = g0_first;
          } else if (g1_second * gradients_scaled(nf_ids[1]) >= 0) {
            gradient_tang_plane(nf_ids[1]) = g1_second;
            gradient_tang_plane(nf_ids[0]) = g0_second;
          }
        } else if (g1_first * gradients_scaled(nf_ids[1]) >= 0 ||
                   g0_first * gradients_scaled(nf_ids[0]) >= 0) {
          // select any solution that preserves the sign
          // INFO("here3")
          gradient_tang_plane(nf_ids[1]) = g1_first;
          gradient_tang_plane(nf_ids[0]) = g0_first;
        } else if (g1_second * gradients_scaled(nf_ids[1]) >= 0 ||
                   g0_second * gradients_scaled(nf_ids[0]) >= 0) {
          // select any solution that preserves the sign
          // INFO("here4")
          gradient_tang_plane(nf_ids[1]) = g1_second;
          gradient_tang_plane(nf_ids[0]) = g0_second;
        } else {
          INFO("there is no max among the nonfixed???")
          INFO("gradients_scaled normalized " << gradients_scaled(0) << " "
                                              << gradients_scaled(1) << " "
                                              << gradients_scaled(2))
          INFO("gradients " << gradients(0) << " " << gradients(1) << " "
                            << gradients(2))
          INFO("non existing same sign gradient plane")
          INFO_VAR(g1_first)
          INFO_VAR(g1_second)
          INFO_VAR(gradients_scaled(nf_ids[1]))

          INFO_VAR(tavg);
          INFO_VAR(g0_first)
          INFO_VAR(g0_second)
          INFO_VAR(gradients_scaled(nf_ids[0]))
          INFO_VAR(tavg);
          INFO_VAR(disc)

          INFO_VAR(req_thrust_acc_max.transpose())
          INFO_VAR(req_thrust_acc_min.transpose())

          INFO_VAR(req_max_thrust_acc.transpose())
          INFO_VAR(req_max_thrust_acc.norm())
          INFO_VAR(a_max)

          INFO("t_times " << t_times(0) << " " << t_times(1) << " "
                          << t_times(2))
          INFO("t_times-tavg " << t_times(0) - tavg << " " << t_times(1) - tavg
                               << " " << t_times(2) - tavg)
          INFO("t_times_diff " << t_times_diff.transpose())
          INFO("thrust_acc " << thrust_acc(0) << " " << thrust_acc(1) << " "
                             << thrust_acc(2) << " norm " << thrust_acc.norm()
                             << " a_max " << a_max)
          INFO_VAR(nf_ids[0])
          INFO_VAR(nf_ids[1])
          INFO_VAR(max_time_idx)
          INFO_VAR(min_time_idx)
          exit(1);
        }
      }


    } else {
      // fix max first
      // INFO("thrust_acc norm " << thrust_acc.norm())
      gradient_tang_plane =
        gradients_scaled -
        gradients_scaled.normalized().dot(thrust_acc.normalized()) * thrust_acc;
      gradient_tang_plane = gradient_tang_plane.normalized() * a_max;

      // limit the dalph to be within the scale of gradient to tangent
      // projection
      double angle_between = acos(
        gradient_tang_plane.normalized().dot(gradients_scaled.normalized()));
      if (M_PI_2 - angle_between > 0) {
        dalph = std::min(dalph, M_PI_2 - angle_between);
        INFO_COND_COLOR(PRINT_DEBUG, OUTPUT_RED,
                        "!!!!!!!! limiting dalph to " << dalph)
      }
      // INFO_MAGENTA("angle_between " << angle_between)
    }

    INFO_COND(PRINT_DEBUG, "gradient_tang_plane dot_equal "
                             << gradient_tang_plane.transpose() << " norm "
                             << gradient_tang_plane.norm())


    INFO_COND(PRINT_DEBUG, "acc_req " << acc_req(0) << " " << acc_req(1) << " "
                                      << acc_req(2))


    INFO_COND(PRINT_DEBUG, "gradient_tang_plane "
                             << gradient_tang_plane.transpose() << " norm "
                             << gradient_tang_plane.norm())

    Vector<3> new_thrust_acc =
      thrust_acc * cos(dalph) + gradient_tang_plane * sin(dalph);


    INFO_COND(PRINT_DEBUG, "new_thrust_acc new "
                             << new_thrust_acc(0) << " " << new_thrust_acc(1)
                             << " " << new_thrust_acc(2) << " norm "
                             << new_thrust_acc.norm() << " a_max " << a_max)


    // INFO("thrust_acc_tst size " << new_thrust_acc.norm() << " a_max "
    //                             << a_max)

    // projecting to constrained acc
    std::vector<bool> fixed{false, false, false};
    reproject_to_sphere(new_thrust_acc, fixed, req_thrust_acc_min,
                        req_thrust_acc_max, acc_req, t_times, a_max);


    const int num_fixed = fixed[0] + fixed[1] + fixed[2];

    if (max_time_idx != max_time_idx_old && iter > 0) {
      min_time_change_decay *= decay_decrease_max_time_swap;
      INFO_COND_COLOR(PRINT_DEBUG, OUTPUT_GREEN,
                      "changed max time axis decay " << min_time_change_decay)
    } else if (num_fixed >= 3) {
      min_time_change_decay *= decay_decrease_three_constrained;
      INFO_COND_COLOR(PRINT_DEBUG, OUTPUT_GREEN,
                      "decrease decay num fixed " << min_time_change_decay)
    } else if (tmax - tmax_old > 0) {
      min_time_change_decay *= decay_increase_tmax;
      INFO_COND_COLOR(PRINT_DEBUG, OUTPUT_GREEN,
                      "decrease decay tmax increased " << min_time_change_decay)
    }


    thrust_acc = new_thrust_acc;

    if (fabs(thrust_acc.norm() - a_max) > 0.01) {
      INFO("2!bad thrust_dir_size " << thrust_acc.norm() << " vs " << a_max);
      INFO_VAR(iter)
      INFO_VAR(thrust_acc.norm())
      INFO_VAR(a_max)
      exit(1);
    }


    const double max_time_tst = t_times.maxCoeff();
    const double min_time_tst = t_times.minCoeff();
    if (max_time_tst - min_time_tst > ALLOWED_DIFF_TIMES_RATIO * tmax) {
      converged = false;
    } else {
      INFO_COND_COLOR(PRINT_DEBUG, OUTPUT_BLUE,
                      "converged in " << iter << " itterations")

      // INFO_BLUE("converged in " << iter << " itterations")
    }

    // for (int var = 0; var < 3; ++var) {
    //   Tr1D &tr = tr_max_acc.get_axis(var);
    //   std::stringstream ss;
    //   ss << "axis_profiles/iter_" << iter << "_axis_" << var
    //      << "sampled_profile.csv";
    //   tr.save_to_file(ss.str());
    // }

    if (converged) {
      INFO_COND(PRINT_DEBUG, "converged to tmax " << tmax << " acc "
                                                  << thrust_acc.transpose())
      INFO_COND(PRINT_DEBUG, "min time recorded " << min_tmax_iters)
      break;
    }
  }


  // INFO("here we are")

  if (!converged) {
  } else {
    if (min_tmax_iters < tmax) {
      INFO_COND_COLOR(PRINT_DEBUG, OUTPUT_GREEN,
                      "change result to min tmax one find during itterations")
      tr_max_acc = min_tmax_iters_tr;
    }
  }


  const double max_time_tst = t_times.maxCoeff();
  double min_time_tst = t_times.minCoeff();
  while (min_time_tst == 0) {
    for (size_t i = 0; i < 3; i++) {
      if (t_times(i) == 0) {
        t_times(i) = max_time_tst;
        tr_max_acc.get_axis(i).t1 = max_time_tst;
        min_time_tst = t_times.minCoeff();
        break;
      }
    }
  }

  if (max_time_tst - min_time_tst > ALLOWED_DIFF_TIMES_RATIO * tmax) {
    tr_max_acc.x.exists = false;
    tr_max_acc.y.exists = false;
    tr_max_acc.z.exists = false;
  }

  return tr_max_acc;
}


inline void reproject_to_sphere(Vector<3> &new_thrust_acc,
                                std::vector<bool> &fixed,
                                const Vector<3> &req_thrust_acc_min,
                                const Vector<3> &req_thrust_acc_max,
                                const Vector<3> &acc_req,
                                const Vector<3> &t_times, const double &a_max) {
  bool within_constrints = false;
  while (!within_constrints) {
    within_constrints = true;
    if (fabs(new_thrust_acc.norm() - a_max) > 0.0001) {
      within_constrints = false;
    }
    // Vector<3> new_thrust_acc = thrust_acc;
    double fixed_len_squared = a_max * a_max;
    double non_fixed_len_squared = 0;
    for (size_t i = 0; i < 3; i++) {
      // INFO("i " << i)
      if (t_times(i) == 0) {
        // t_times(i) == 0 means no acc optimization needed and set to min
        // required one is possible
        if (!fixed[i]) {
          within_constrints = false;
        }
        if (acc_req(i) > 0) {
          new_thrust_acc(i) = req_thrust_acc_min(i);
          fixed[i] = true;
          fixed_len_squared -= req_thrust_acc_min(i) * req_thrust_acc_min(i);
        } else {
          new_thrust_acc(i) = req_thrust_acc_max(i);
          fixed[i] = true;
          fixed_len_squared -= req_thrust_acc_max(i) * req_thrust_acc_max(i);
        }

      } else {
        if (acc_req(i) < 0) {
          if (new_thrust_acc(i) > req_thrust_acc_max(i)) {
            within_constrints = false;
          }
          if (new_thrust_acc(i) > req_thrust_acc_max(i) || fixed[i]) {
            new_thrust_acc(i) = req_thrust_acc_max(i);
            fixed[i] = true;
            fixed_len_squared -= req_thrust_acc_max(i) * req_thrust_acc_max(i);
            // INFO("fixing acc " << i)
          } else {
            non_fixed_len_squared += new_thrust_acc(i) * new_thrust_acc(i);
          }
        } else {
          if (new_thrust_acc(i) < req_thrust_acc_min(i)) {
            within_constrints = false;
          }
          if (new_thrust_acc(i) < req_thrust_acc_min(i) || fixed[i]) {
            new_thrust_acc(i) = req_thrust_acc_min(i);
            fixed[i] = true;
            fixed_len_squared -= req_thrust_acc_min(i) * req_thrust_acc_min(i);
            //  INFO("fixing acc " << i)
          } else {
            non_fixed_len_squared += new_thrust_acc(i) * new_thrust_acc(i);
          }
        }
      }
    }
    INFO_COND(PRINT_DEBUG,
              "fixed " << fixed[0] << " " << fixed[1] << " " << fixed[2])

    INFO_COND(PRINT_DEBUG, "bef new_thrust_acc " << new_thrust_acc(0) << " "
                                                 << new_thrust_acc(1) << " "
                                                 << new_thrust_acc(2))

    if (non_fixed_len_squared != 0) {
      double add_acc = 1.0;
      if (!fixed[2]) {
        non_fixed_len_squared =
          non_fixed_len_squared - 2 * (new_thrust_acc(2) - G) * G - G * G;
        // INFO("non_fixed_len_squared " << non_fixed_len_squared)

        const double c = -fixed_len_squared + G * G;
        const double b = 2 * (new_thrust_acc(2) - G) * G;
        const double a = non_fixed_len_squared;
        add_acc = (-b + sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
        // const double add_acc2 = (-b - sqrt(b * b - 4.0 * a * c)) / (2.0 *
        // a); INFO("add_acc " << add_acc) INFO("add_acc " << add_acc2)
      } else {
        add_acc = sqrt(fixed_len_squared / non_fixed_len_squared);
      }

      // INFO("fixed_len_squared " << fixed_len_squared)
      // INFO("scale_non_fixed " << scale_non_fixed)
      for (size_t i = 0; i < 3; i++) {
        if (!fixed[i]) {
          if (i == 2) {
            new_thrust_acc(i) = (new_thrust_acc(i) - G) * add_acc + G;
          } else {
            new_thrust_acc(i) = new_thrust_acc(i) * add_acc;
          }
        }
      }
    } else {
      double scale_non_fixed = 1.0;
      scale_non_fixed =
        sqrt((a_max * a_max) / ((a_max * a_max) - fixed_len_squared));
      // INFO("2scale_non_fixed " << scale_non_fixed)
      for (size_t i = 0; i < 3; i++) {
        new_thrust_acc(i) = new_thrust_acc(i) * (scale_non_fixed);
      }
    }

    /*
        if (scale_non_fixed < 1.0) {
          within_constrints = false;
        }
        */
    INFO_COND(PRINT_DEBUG, "aft new_thrust_acc " << new_thrust_acc(0) << " "
                                                 << new_thrust_acc(1) << " "
                                                 << new_thrust_acc(2))
  }
  // INFO("reproject_to_sphere end")
}


void get_time_avg_min(Scalar &tavg, Scalar &tmin, int &max_time_idx,
                      int &min_time_idx, const Scalar &tmax,
                      const Vector<3> &t_times,
                      const Vector<3> &gradients_scaled) {
  tmin = DBL_MAX;
  tavg = 0;
  Scalar tavg_num = 0;
  for (size_t i = 0; i < 3; i++) {
    if (t_times(i) != 0 && gradients_scaled(i) != 0) {
      tavg += t_times(i);
      tavg_num += 1.0;
      if (t_times(i) < tmin) {
        tmin = t_times(i);
        min_time_idx = i;
      }
      if (t_times(i) == tmax) {
        max_time_idx = i;
      }
    }
  }
  tavg /= tavg_num;
  if (max_time_idx == -1 || min_time_idx == -1) {
    INFO_RED("wrong max_time_idx or min_time_idx " << max_time_idx << " "
                                                   << min_time_idx)
    //   INFO("from_state p:" << from_state.p.transpose()
    //                        << " v:" << from_state.v.transpose());
    //   INFO("to_state p:" << to_state.p.transpose()
    //                      << " v:" << to_state.v.transpose());
    INFO("t_times " << t_times.transpose())
    exit(1);
  }
}


std::ostream &operator<<(std::ostream &o, const Rr1D &f) {
  o << "maxacc: t1:" << f.t1 << ";t2:" << f.t2 << ";t3:" << f.t3
    << ";exists:" << f.exists << ";a1:" << f.a1 << ";a2:" << f.a2
    << ";i:" << f.i << "\n";
  o << "      : p0:" << f.p0 << ";p1:" << f.p1 << ";p2:" << f.p2
    << ";p3:" << f.p3 << "\n";
  o << "      : v0:" << f.v0 << ";v1:" << f.v1 << ";v3:" << f.v3 << "t_tot"
    << (f.t1 + f.t2 + f.t3);
  return o;
}

std::ostream &operator<<(std::ostream &o, const TrMaxAcc3D &f) {
  o << "maxacc3d: t:" << f.time() << ";exists:" << f.exists();
  o << "\n\tx: " << f.x;
  o << "\n\ty: " << f.y;
  o << "\n\tz: " << f.z;
  return o;
}


void test_acc_space(const Drone *drone) {
  std::vector<std::pair<DroneState, DroneState>> start_end_states;

  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.8, 6.8, 1.2);
    from_state.v = Vector<3>(-18, -18, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-4, 5.5, 1.2);
    to_state.v = Vector<3>(-17, 17, 0);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.5, -6.0, 3.2);
    from_state.v = Vector<3>(-16, -18.0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-3.9, -6.0, 2.2);
    to_state.v = Vector<3>(-13, -18.1, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-5, 4.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-0.9, -1.27, 3.48);
    to_state.v = Vector<3>(9.6225, -2.58819, 0.84186);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(23, 3, 0.868);
    from_state.v = Vector<3>(4.82963, -1.2941, 0);
    DroneState to_state;
    to_state.p = Vector<3>(15, 3.01, 0.8681);
    to_state.v = Vector<3>(2.7232, 4.19335, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p =
      Vector<3>(-4.078791909208521, -6.233218162831177, 3.3552067922757387);
    from_state.v =
      Vector<3>(-7.465698842967558, 1.56343810643077, -2.717844981012354);
    DroneState to_state;
    to_state.p =
      Vector<3>(-4.481316085189184, -5.9469612103032885, 1.2303806566203637);
    to_state.v =
      Vector<3>(5.314808055640459, 0.7874650340109357, -5.126558119286457);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(-5, 4.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-0.9, -1.27, 3.48);
    to_state.v = Vector<3>(8.3403, -0.448594, -1.92551);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(-0.9, -1.27, 3.48);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(9.09, 6.26, 1.08);
    to_state.v = Vector<3>(4.31514, -0.313904, -1.23735);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(-4.48, -5.94, 1.05);
    from_state.v = Vector<3>(4.22604, 1.53909, -0.147577);
    DroneState to_state;
    to_state.p = Vector<3>(4.45, -0.8, 1.09);
    to_state.v = Vector<3>(0.60916, 8.97808, 0.151881);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(9.09, 6.26, 1.08);
    from_state.v = Vector<3>(1.41038, -0.413456, 0.299786);
    DroneState to_state;
    to_state.p = Vector<3>(9.27, -3.46, 1.17);
    to_state.v = Vector<3>(-1.05605, -2.78155, -0.384369);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(4.45, -0.8, 1.09);
    from_state.v = Vector<3>(-16, -2, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-2.65, 6.51, 1.3);
    to_state.v = Vector<3>(2, 4, 0);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(-5, 4.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-0.9, -1.27, 3.48);
    to_state.v = Vector<3>(11.5933, 1.87721, -2.46422);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-5, 4.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-0.9, -1.27, 3.48);
    to_state.v = Vector<3>(10.0435, -2.7176, -1.41152);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(-4.48, -5.94, 1.05);
    from_state.v = Vector<3>(11.5911, -3.10583, 0);
    DroneState to_state;
    to_state.p = Vector<3>(4.45, -0.8, 1.09);
    to_state.v = Vector<3>(1.03847, 2.80074, -0.278257);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-5, 4.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-0.9, -1.27, 3.48);
    to_state.v = Vector<3>(11.5933, 1.87721, -2.46422);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(-1.1, -1.6, 3.6);
    from_state.v = Vector<3>(-18, -18, 0);
    DroneState to_state;
    to_state.p = Vector<3>(9.2, 6.6, 1.2);
    to_state.v = Vector<3>(-18, -18, 0);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(9.27, -3.46, 1.17);
    from_state.v = Vector<3>(-12, 14, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-4.5, -6.14, 3.4);
    to_state.v = Vector<3>(8, 16, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(9.09, 6.26, 1.08);
    from_state.v = Vector<3>(-12, -18, 0);
    DroneState to_state;
    to_state.p = Vector<3>(9.27, -3.46, 1.17);
    to_state.v = Vector<3>(12, -12, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-0.9, -1.27, 3.48);
    from_state.v = Vector<3>(-12, 8, 0);
    DroneState to_state;
    to_state.p = Vector<3>(9.09, 6.26, 1.08);
    to_state.v = Vector<3>(18, 12, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(9.09, 6.26, 1.08);
    from_state.v = Vector<3>(-14, -18, 0);
    DroneState to_state;
    to_state.p = Vector<3>(9.27, -3.46, 1.17);
    to_state.v = Vector<3>(-14, 8, 0);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(-4.48, -5.94, 1.05);
    from_state.v = Vector<3>(6, 6, 0);
    DroneState to_state;
    to_state.p = Vector<3>(4.45, -0.8, 1.09);
    to_state.v = Vector<3>(6, 6, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(9.2, 6.6, 1.2);
    from_state.v = Vector<3>(-17, -18, 0);
    DroneState to_state;
    to_state.p = Vector<3>(9.2, -4, 1.2);
    to_state.v = Vector<3>(18, -18, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(9.2, 6.6, 1.2);
    from_state.v = Vector<3>(-18, -18, 0);
    DroneState to_state;
    to_state.p = Vector<3>(9.2, -4, 1.2);
    to_state.v = Vector<3>(-18, -18, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.7, 0.0, 1.2);
    from_state.v = Vector<3>(-18, -18, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-1.1, -1.6, 3.6);
    to_state.v = Vector<3>(-18, 18, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.7, 0.0, 1.2);
    from_state.v = Vector<3>(-18, -18, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-1.1, -1.6, 3.6);
    to_state.v = Vector<3>(-18, -18, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.7, 0.0, 1.2);
    from_state.v = Vector<3>(-18, -15, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-1.1, -1.6, 3.6);
    to_state.v = Vector<3>(17, 16, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.8, 6.8, 1.2);
    from_state.v = Vector<3>(-18, -7, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-4, 5.5, 1.2);
    to_state.v = Vector<3>(17, -1, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.8, 6.8, 1.2);
    from_state.v = Vector<3>(-18, -8, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-4, 5.5, 1.2);
    to_state.v = Vector<3>(17, -2, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(0, 8.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-2.8, 6.8, 1.2);
    to_state.v = Vector<3>(-11, -8, 0);
    start_end_states.push_back({from_state, to_state});
  }


  {
    DroneState from_state;
    from_state.p = Vector<3>(-4, 5.5, 1.2);
    from_state.v = Vector<3>(-2, -15, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-2.7, 0, 1.2);
    to_state.v = Vector<3>(2, -9, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(0, 8.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-2.8, 6.8, 1.2);
    to_state.v = Vector<3>(12, -2, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(0, 8.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-2.8, 6.8, 1.2);
    to_state.v = Vector<3>(-2, -3, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(0, 8.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-2.8, 6.8, 1.2);
    to_state.v = Vector<3>(-13, -5, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(0, 8.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-2.8, 6.8, 1.2);
    to_state.v = Vector<3>(-4, 4, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(0, 8.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-2.8, 6.8, 1.2);
    to_state.v = Vector<3>(-1, -10, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(0, 8.5, 1.2);
    from_state.v = Vector<3>(0, 0, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-2.8, 6.8, 1.2);
    to_state.v = Vector<3>(-13, -3, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.8, 6.8, 1.2);
    from_state.v = Vector<3>(-18, -2, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-4, 5.5, 1.2);
    to_state.v = Vector<3>(16, -2, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.8, 6.8, 1.2);
    from_state.v = Vector<3>(-18, -14, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-4, 5.5, 1.2);
    to_state.v = Vector<3>(17, 12, 0);
    start_end_states.push_back({from_state, to_state});
  }

  {
    DroneState from_state;
    from_state.p = Vector<3>(-2.8, 6.8, 1.2);
    from_state.v = Vector<3>(-7, -8, 0);
    DroneState to_state;
    to_state.p = Vector<3>(-4, 5.5, 1.2);
    to_state.v = Vector<3>(-5, -7, 0);
    start_end_states.push_back({from_state, to_state});
  }


  /*saving time per acc and its plotting end*/

  for (size_t sid = 0; sid < start_end_states.size(); sid++) {
    INFO("-----------------------------------")


    auto [from_state, to_state] = start_end_states[sid];

    // save_times_per_acc(from_state, to_state);


    // Vector<3> min_calc_body_a = min_calc_acc;


    double a_max = 4.0 * drone->max_t_motor_ / drone->m_;

    // test what i will find here
    INFO("bef")
    TrMaxAcc3D tst = calc_max_acc_thrust(from_state, to_state, drone, 0.001);
    INFO("aft")
    const double tst_time = tst.exists() ? tst.time() : DBL_MAX;
    // const double tst_time = 0;
    INFO_CYAN("GD time is " << tst_time)


    auto [min_calc, min_calc_acc, min_calc_times, data_acc_time, num_accepted] =
      test_samling_thrust(from_state, to_state, a_max);

    INFO_GREEN("search time is " << min_calc)
    INFO_VAR(num_accepted)
    // INFO_VAR(min_calc)
    INFO_VAR(min_calc_acc.transpose())
    INFO_VAR(min_calc_times.transpose())

    // INFO_VAR(min_calc_body_a.transpose())

    // sleep(1);
    // double max_val_stored = min_calc + 2.0;


    // std::ofstream myfile;
    // myfile.open("acc_distribution_times.csv");
    // if (myfile.is_open()) {
    //   for (size_t i = 0; i < data_acc_time.size(); i++) {
    //     if (i % ((int)(data_acc_time.size() / 10.0)) == 0) {
    //       INFO("saved " << i / ((double)data_acc_time.size()))
    //     }
    //     auto [acc, time] = data_acc_time[i];
    //     // if (time > max_val_stored) {
    //     //   time = max_val_stored;
    //     // }
    //     myfile << std::setprecision(3) << std::fixed << acc(0) << "," <<
    //     acc(1)
    //            << "," << std::setprecision(6) << acc(2) << "," << time
    //            << std::endl;
    //   }
    //   myfile.close();
    // }


    if (fabs(tst_time - min_calc) > 0.05) {
      INFO_RED("diff between descent and sampled")
      INFO("from_state p:" << from_state.p.transpose()
                           << " v:" << from_state.v.transpose());
      INFO("to_state p:" << to_state.p.transpose()
                         << " v:" << to_state.v.transpose());
      INFO_VAR(min_calc)
      INFO_VAR(min_calc_acc.transpose())
      INFO_VAR(tst_time)
      INFO_VAR(min_calc_times)
      Vector<3> tst_times(tst.get_axis(0).time(), tst.get_axis(1).time(),
                          tst.get_axis(2).time());
      INFO_VAR(tst_times)
      const double tst_times_mint = tst_times.minCoeff();
      const double tst_times_maxt = tst_times.maxCoeff();
      if (fabs(tst_times_maxt - tst_times_mint) <
          ALLOWED_DIFF_TIMES_RATIO * tst_times_maxt) {
        INFO("tst time is within constraints!!!")
      } else {
        INFO_RED("tst not within constrains but presented as it is")
        exit(1);
      }
    }


    // INFO("test exit")
    // exit(1);
  }
}


void save_times_per_acc(const DroneState &from, const DroneState &to) {
  /*saving time per acc and its plotting begin*/
  INFO("saving time per acc")
  const double prec_a = 0.001;

  for (size_t i = 0; i < 3; i++) {
    INFO("axis " << i)
    const double ps = from.p(i);
    const double vs = from.v(i);
    const double pe = to.p(i);
    const double ve = to.v(i);


    std::ofstream myfile_valat_neg;
    std::stringstream ss_neg;
    ss_neg << "val_a_t_negative" << i << ".csv";
    myfile_valat_neg.open(ss_neg.str().c_str());
    if (myfile_valat_neg.is_open()) {
      for (double a = -prec_a; a >= -35.0; a -= prec_a) {
        Tr1D tst;  // = one_dim_double_integrator(ps, vs, pe, ve, a, 0, true);

        if (i < 2) {
          tst =
            one_dim_double_integrator_two_acc(ps, vs, pe, ve, a, -a, i, true);
        } else {
          const double a_w_g_pos = a - G;
          const double a_w_g_neg = -a - G;
          tst = one_dim_double_integrator_two_acc(ps, vs, pe, ve, a_w_g_pos,
                                                  a_w_g_neg, i, true);
        }
        if (tst.exists) {
          // INFO(a << ":" << tst.time())
          myfile_valat_neg << std::setprecision(16) << std::fixed << a << ","
                           << tst.time() << "," << tst.dt_da << "," << tst.a1
                           << "," << tst.a2 << std::endl;
          if (i != 2 and tst.a1 != a) {
            INFO_RED("not good " << i)
            INFO("tst.a1 " << tst.a1)
            INFO("a " << a)
            exit(1);
          }
        }
      }
      myfile_valat_neg.close();
    }

    std::ofstream myfile_valat_pos;
    std::stringstream ss_pos;
    ss_pos << "val_a_t_positive" << i << ".csv";
    myfile_valat_pos.open(ss_pos.str().c_str());
    if (myfile_valat_pos.is_open()) {
      for (double a = prec_a; a <= 35.0; a += prec_a) {
        Tr1D
          tst;  // tr = one_dim_double_integrator(ps, vs, pe, ve, a, 0, true);
        if (i < 2) {
          tst =
            one_dim_double_integrator_two_acc(ps, vs, pe, ve, a, -a, i, true);
        } else {
          const double a_w_g_pos = a - G;
          const double a_w_g_neg = -a - G;
          tst = one_dim_double_integrator_two_acc(ps, vs, pe, ve, a_w_g_pos,
                                                  a_w_g_neg, i, true);
        }
        if (tst.exists) {
          // INFO(a << ":" << tst.time())
          myfile_valat_pos << std::setprecision(16) << std::fixed << a << ","
                           << tst.time() << "," << tst.dt_da << "," << tst.a1
                           << "," << tst.a2 << std::endl;
          if (i != 2 and tst.a1 != a) {
            INFO_RED("not good " << i)
            INFO("tst.a1 " << tst.a1)
            INFO("a " << a)
            exit(1);
          }
        }
      }
      myfile_valat_pos.close();
    }
  }
}


std::tuple<double, Vector<3>, Vector<3>,
           std::vector<std::pair<Vector<3>, double>>, int>
test_samling_thrust(const DroneState &from_state, const DroneState &to_state,
                    const double a_max) {
  std::vector<std::pair<Vector<3>, double>> data_acc_time;
  Vector<3> min_calc_acc = Vector<3>(0, 0, 0);
  Vector<3> min_calc_times = Vector<3>(0, 0, 0);
  Vector<3> min_calc_body_a = min_calc_acc;
  double min_calc = DBL_MAX;

  double num_sphere_patches = 2000;
  int num_accepted = 0;
  for (int ipitch = 0; ipitch <= num_sphere_patches; ipitch++) {
    for (int iyaw = 0; iyaw <= num_sphere_patches; iyaw++) {
      // INFO_VAR(ix)

      const double yaw = -M_PI + iyaw * 2 * M_PI / ((double)num_sphere_patches);
      const double pitch =
        M_PI_2 - ipitch * M_PI / ((double)num_sphere_patches);
      Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());
      Eigen::AngleAxisd pitch_rot(pitch, Eigen::Vector3d::UnitY());
      Vector<3> thrust_dir_acc =
        yaw_rot * pitch_rot * Eigen::Vector3d::UnitX() * a_max;

      // const Vector<3> thrust_dir(ix / loop_size, iy / loop_size,
      //                            iz / loop_size);
      // const Vector<3> thrust_dir_acc = thrust_dir.normalized() * a_max;
      // const Vector<3> thrust_dir_acc(-8.41168, -29.8228, 11.1789);
      const Vector<3> body_a_plus = thrust_dir_acc + GVEC;
      const Vector<3> body_a_minus = -thrust_dir_acc + GVEC;


      Vector<3> times(DBL_MAX, DBL_MAX, DBL_MAX);
      Vector<3> close_to_boundar(DBL_MAX, DBL_MAX, DBL_MAX);
      bool exists = true;
      for (int i = 0; i < 3; ++i) {
        const double max_acc = body_a_plus(i);
        const double min_acc = body_a_minus(i);
        const double vs = from_state.v(i);
        const double ve = to_state.v(i);
        const double ps = from_state.p(i);
        const double pe = to_state.p(i);
        const Tr1D tr = one_dim_double_integrator_two_acc(ps, vs, pe, ve,
                                                          max_acc, min_acc, i);
        if (!tr.exists) {
          exists = false;
          break;
        } else {
          times(i) = tr.time();
        }
      }
      if (exists) {
        // INFO("times " << times.transpose());
        double max_coef = 0;        // = times.maxCoeff();
        double min_coef = DBL_MAX;  //= times.minCoeff();
        for (size_t ti = 0; ti < 3; ti++) {
          if (times(ti) != 0) {
            if (times(ti) < min_coef) {
              min_coef = times(ti);
            }
            if (times(ti) > max_coef) {
              max_coef = times(ti);
            }
          }
        }


        if (fabs(max_coef - min_coef) < ALLOWED_DIFF_TIMES_RATIO * max_coef) {
          data_acc_time.push_back({thrust_dir_acc, max_coef});
          num_accepted++;
          if (max_coef < min_calc) {
            min_calc = max_coef;
            min_calc_acc = thrust_dir_acc;
            min_calc_body_a = body_a_plus;
            min_calc_times = times;
          }
        } else {
          data_acc_time.push_back({thrust_dir_acc, NAN});
        }
      } else {
        data_acc_time.push_back({thrust_dir_acc, NAN});
      }
    }
  }
  return {min_calc, min_calc_acc, min_calc_times, data_acc_time, num_accepted};
}