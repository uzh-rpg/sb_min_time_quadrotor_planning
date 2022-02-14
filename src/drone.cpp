#include "drone.hpp"

#include "common.hpp"

Drone::Drone(const YAML::Node& drone_config) {
  max_t_motor_ = loadParam<Scalar>(drone_config, "thrust_max");
  min_t_motor_ = loadParam<Scalar>(drone_config, "thrust_min");
  m_ = loadParam<Scalar>(drone_config, "mass");
  max_acc_ = max_t_motor_ * 4.0 / m_;
  l_ = loadParam<Scalar>(drone_config, "arm_length");
  kappa_ = loadParam<Scalar>(drone_config, "torque_coeff");
  std::vector<std::vector<double>> J_array;
  parseArrayParam(drone_config, "inertia", J_array);
  // J_ = Vector<3>(0.001, 0.001, 0.0014).asDiagonal();
  // J_;
  for (int row = 0; row < J_array.size(); ++row) {
    for (int col = 0; col < J_array[row].size(); ++col) {
      J_(row, col) = J_array[row][col];
    }
  }
  INFO("J_ is");
  std::cout << J_ << std::endl;
  //= Vector<3>(0.001, 0.001, 0.0014).asDiagonal();

  J_inv_ = J_.inverse();
  // total thrust (1line),
  motor_allocation_ << Vector<4>::Ones().transpose(),
    l_ * Vector<4>(-1.0, 1.0, -1.0, 1.0).transpose(),
    l_ * Vector<4>(-1.0, 1.0, 1.0, -1.0).transpose(),
    kappa_ * Vector<4>(-1.0, -1.0, 1.0, 1.0).transpose();
  motor_allocation_inv_ = motor_allocation_.inverse();
  omega_max_xy = loadParam<Scalar>(drone_config, "omega_max_xy");
  omega_max_z = loadParam<Scalar>(drone_config, "omega_max_z");
  // exit(1);
}

/*
inline Vector<3> Drone::get_tau(const Ref<const Vector<4>> T) {
  Vector<3> tau;
  tau << l_ * (T(0) - T(1) - T(2) + T(3)), l_ * (-T(0) - T(1) + T(2) + T(3)),
    kappa_ * (T(0) - T(1) + T(2) - T(3));
  return tau;
}
*/

void Drone::get_derivative(const Ref<const Vector<DroneState::SIZE>> state,
                           const Ref<const Vector<4>> T,
                           Ref<Vector<DroneState::SIZE>> derivative) {
  //                ^
  //     *3   *0    |
  //       \ /      |
  //        .       |
  //       / \      |x
  //     *1   *2    |
  //                |
  //  <_____________|
  //         y

  // Scalar weight_norm_t_thrust = (T(0) + T(1) + T(2) + T(3)) / this->m_;

  // Vector<3> tau = this->get_tau(T);

  const Vector<3> omega(state(DroneState::OMEX), state(DroneState::OMEY),
                        state(DroneState::OMEZ));
  const Quaternion q_omega(0, state(DroneState::OMEX), state(DroneState::OMEY),
                           state(DroneState::OMEZ));

  // p_dot = v
  derivative.segment<DroneState::NPOS>(DroneState::POS) =
    state.segment<DroneState::NVEL>(DroneState::VEL);

  // q_dot = 0.5 * q_right(q_omega) * [0, w]
  derivative.segment<DroneState::NATT>(DroneState::ATT) =
    0.5 * Q_right(q_omega) * state.segment<DroneState::NATT>(DroneState::ATT);


  //[total_thrust;tau_x;tau_y;tau_z]
  // std::cout << "T" << T << std::endl;
  const Vector<4> force_torques = motor_allocation_ * T;
  // std::cout << "motor_allocation_" << motor_allocation_ << std::endl;

  // w_dot = Jinv * (tau -  omega x (quad_.J_ * omega))
  derivative.segment<DroneState::NOME>(DroneState::OME) =
    J_inv_ * (force_torques.segment<3>(1) - omega.cross(J_ * omega));

  // std::cout << derivative.segment<DroneState::NOME>(DroneState::OME)
  //          << std::endl;

  const Matrix<3, 3> R =
    Quaternion(state(DroneState::ATTW), state(DroneState::ATTX),
               state(DroneState::ATTY), state(DroneState::ATTZ))
      .toRotationMatrix();
  // v_dot = rotate_quat(q, vertcat(0, 0, (T[0]+T[1]+T[2]+T[3])/self.m)) + g
  //(- v * self.cd)

  const Vector<3> force(0.0, 0.0, force_torques[0]);
  derivative.segment<DroneState::NVEL>(DroneState::VEL) = R * force / m_ + GVEC;
}

bool Drone::check_limits(const Ref<const Vector<>> state) const {
  bool ok = check_omegas(state.segment<3>(DroneState::IDX::OME));
  return ok;
}

bool Drone::check_omegas(const Vector<3> omegas) const {
  if (omegas(0) - omega_max_xy > EQUALITY_ERROR ||
      omegas(0) + omega_max_xy < -EQUALITY_ERROR ||
      omegas(1) - omega_max_xy > EQUALITY_ERROR ||
      omegas(1) + omega_max_xy < -EQUALITY_ERROR ||
      omegas(2) - omega_max_z > EQUALITY_ERROR ||
      omegas(2) + omega_max_z < -EQUALITY_ERROR) {
    INFO_RED("out of bounds w " << omegas(0) << " " << omegas(1) << " "
                                << omegas(2));

    /*INFO_RED("out of bounds w " << state(DroneState::IDX::POSX) << " "
                                << state(DroneState::IDX::POSY) << " "
                                << state(DroneState::IDX::POSZ));*/
    // INFO_RED((state(DroneState::IDX::OMEX) < -omega_max_xy));
    // INFO_RED("bounds are " << omega_max_xy << " " << omega_max_z);

    return false;
  } else {
    return true;
  }
}


std::tuple<Scalar, Scalar, Vector<4>, Scalar>
Drone::get_max_motors_for_rotation(const Vector<3> rotation_axis) const {
  // set tot_trust + torque to 0 and ortation vector
  const Vector<4> tau(0, rotation_axis[0], rotation_axis[1], rotation_axis[2]);
  // get the motor values for such required torques
  const Vector<4> motor_test = motor_allocation_inv_ * tau;
  // scale and shift the motor values to be within min-max renge
  const Scalar mint = motor_test.minCoeff();
  const Scalar maxt = motor_test.maxCoeff();
  const Scalar range_thrust = max_t_motor_ - min_t_motor_;
  const Scalar scale = range_thrust / (maxt - mint);
  const Vector<4> motor_full =
    scale * (motor_test - Vector<4>::Constant(mint)) +
    Vector<4>::Constant(min_t_motor_);

  // now get the total thrust and torque for scaled-shifter motor values
  const Vector<4> tau_full = motor_allocation_ * motor_full;

  // get the andular acceleration for such tau, ignore the giroscopic part of
  // the Euler's rotation equations
  Vector<3> ang_acc = J_inv_ * tau_full.segment<3>(1);

  const Vector<3> ang_acc_u = ang_acc.normalized();
  Scalar scale_z = std::numeric_limits<Scalar>::max();
  if (fabs(ang_acc_u[2]) > PRECISION) {
    scale_z = omega_max_z /
              fabs(ang_acc_u[2]);  // calc normalized time to reach max omega z
  }
  const Scalar scale_xy =
    omega_max_xy /
    std::max(fabs(ang_acc_u[0]),
             fabs(ang_acc_u[1]));  // calc normalized time to reach max omega xy
  const Vector<3> max_angular_speeds = ang_acc_u * std::min(scale_xy, scale_z);
  check_omegas(max_angular_speeds);

  // calc times to reach from 0 to max_angular_speeds using ang_acc
  const Vector<3> times_ang_acc =
    max_angular_speeds.cwiseProduct(ang_acc.cwiseInverse());
  const double max_time_acc = times_ang_acc.maxCoeff();

  // get the ang speed and acc in the direction of the rotation axis == dot
  // product of those two
  Scalar ang_speed_rot_vec = rotation_axis.transpose() * max_angular_speeds;
  Scalar ang_acc_rot_vec = rotation_axis.transpose() * ang_acc;

  return {ang_acc_rot_vec, ang_speed_rot_vec, motor_full, max_time_acc};
}


Matrix<3, 3> Drone::get_Rq_integral(const Scalar& omg0, const Scalar& t,
                                    const Scalar& w,
                                    const Vector<3>& rot_axis) {
  const Scalar i = rot_axis(0);
  const Scalar j = rot_axis(1);
  const Scalar k = rot_axis(2);
  const Scalar sin_omg0_wt = sin(omg0 + w * t);
  const Scalar cos_omg0_wt = cos(omg0 + w * t);
  const Scalar w_2 = (2.0 * w);
  const Scalar min_sin_omg0__ = (-sin_omg0_wt + omg0 + w * t);
  const Scalar a_p2 = (sin_omg0_wt + omg0 + w * t) / w_2;  // a^2
  const Scalar b_p2 = i * i * (min_sin_omg0__ / w_2);      // b^2
  const Scalar c_p2 = j * j * (min_sin_omg0__ / w_2);      // c^2
  const Scalar d_p2 = k * k * (min_sin_omg0__ / w_2);      // d^2
  const Scalar b_c = i * j * (min_sin_omg0__ / w_2);       // b*c
  const Scalar b_d = i * k * (min_sin_omg0__ / w_2);       // b*d
  const Scalar c_d = j * k * (min_sin_omg0__ / w_2);       // c*d
  const Scalar a_b = i * (-cos_omg0_wt / w_2);             // a*b
  const Scalar a_c = j * (-cos_omg0_wt / w_2);             // a*c
  const Scalar a_d = k * (-cos_omg0_wt / w_2);             // a*d
  Matrix<3, 3> rq_int;
  rq_int << a_p2 + b_p2 - c_p2 - d_p2, 2 * b_c - 2 * a_d, 2 * b_d + 2 * a_c,
    2 * b_c + 2 * a_d, a_p2 - b_p2 + c_p2 - d_p2, 2 * c_d - 2 * a_b,
    2 * b_d - 2 * a_c, 2 * c_d + 2 * a_b, a_p2 - b_p2 - c_p2 + d_p2;
  return rq_int;
}

Vector<4> Drone::get_const_rotation_not_fall_thrust(const Quaternion& q_from,
                                                    const Scalar& theta_from,
                                                    const Scalar& theta_to,
                                                    const Vector<3>& rot_axis,
                                                    const Scalar& time) {
  // INFO("get_const_rotation_not_fall_thrust begin");

  const Scalar omg0 = theta_from;
  const Scalar w = (theta_to - theta_from) / time;
  const Matrix<3, 3> qrot_to = get_Rq_integral(omg0, time, w, rot_axis);
  const Matrix<3, 3> qrot_from = get_Rq_integral(omg0, 0, w, rot_axis);
  const Matrix<3, 3> qrot_diff_old = qrot_to - qrot_from;

  // const Quaternion q_from(AngleAxis(theta_from, rot_axis));
  // INFO_VAR(qrot_diff_old);

  const Matrix<3, 3> qrot_diff = q_from.toRotationMatrix() * qrot_diff_old;
  // nv_qrot_diff = np.linalg.inv(qrot_diff)
  // INFO_VAR(qrot_diff);


  Vector<4> thrust;
  if (qrot_diff(2, 2) == 0) {
    thrust = Vector<4>::Ones() * min_t_motor_;
  } else {
    const Scalar tot = G * time * m_ / qrot_diff(2, 2);
    thrust = Vector<4>::Ones() * tot / 4.0;
  }
  thrust = thrust.cwiseMin(max_t_motor_).cwiseMax(min_t_motor_);
  return thrust;
}

Vector<3> Drone::thrust_acc_from_body_acc(const Vector<3> body_acc) {
  return body_acc - GVEC;
}

Scalar Drone::distance_pos(const DroneState& a, const DroneState& b) {
  Scalar dist = 0;
  for (int var = DroneState::IDX::POS;
       var < DroneState::IDX::POS + DroneState::IDX::NPOS; ++var) {
    dist += (a.x(var) - b.x(var)) * (a.x(var) - b.x(var));
  }
  // INFO("dist " << a.p.transpose() << " vs " << b.p.transpose())
  // DEBUG
  if (fabs(sqrt(dist) - (a.p - b.p).norm()) > 1e-10) {
    INFO("bad distance calc pos")
    INFO("state a pos " << a.p)
    INFO("state b pos " << b.p)
    INFO_VAR(sqrt(dist))
    INFO_VAR((a.p - b.p).norm())
    exit(1);
  }
  return sqrt(dist);
}

Scalar Drone::distance_vel(const DroneState& a, const DroneState& b) {
  Scalar dist = 0;
  for (int var = DroneState::IDX::VEL;
       var < DroneState::IDX::VEL + DroneState::IDX::NVEL; ++var) {
    dist += DroneDistance<float>::vel_scale_square * (a.x(var) - b.x(var)) *
            (a.x(var) - b.x(var));
  }
  if (fabs(sqrt(dist) - (a.v - b.v).norm()) > 1e-10) {
    INFO("bad distance calc vel")
    INFO_VAR(sqrt(dist))
    INFO_VAR((a.v - b.v).norm())
    exit(1);
  }
  return sqrt(dist);
}

Scalar Drone::distance(const DroneState& a, const DroneState& b) {
  // static Scalar max_att_dist = 0;
  Scalar dist = 0;
  for (int var = 0; var < a.x.rows(); ++var) {
    if (var < DroneState::IDX::POS + DroneState::IDX::NPOS) {
      // pos
      dist += DroneDistance<float>::pos_scale_square * (a.x(var) - b.x(var)) *
              (a.x(var) - b.x(var));
    } else if (var < DroneState::IDX::ATT + DroneState::IDX::NATT) {
      // attitude
      dist += DroneDistance<float>::att_scale_square * (a.x(var) - b.x(var)) *
              (a.x(var) - b.x(var));
    } else if (var < DroneState::IDX::VEL + DroneState::IDX::NVEL) {
      // velocity
      dist += DroneDistance<float>::vel_scale_square * (a.x(var) - b.x(var)) *
              (a.x(var) - b.x(var));
    } else if (var < DroneState::IDX::OME + DroneState::IDX::NOME) {
      // omega
      dist += DroneDistance<float>::omega_scale_square * (a.x(var) - b.x(var)) *
              (a.x(var) - b.x(var));
    }
  }
  // if ((a.qx - b.qx).norm() > max_att_dist) {
  //   max_att_dist = (a.qx - b.qx).norm();
  //   INFO((a.qx - b.qx).norm())
  // }
  return sqrt(dist);
}

Scalar Drone::distance_without_pos(const DroneState& a, const DroneState& b) {
  Scalar dist = 0;
  for (int var = 0; var < a.x.rows(); ++var) {
    if (var < DroneState::IDX::ATT + DroneState::IDX::NATT) {
      // attitude
      dist += DroneDistance<float>::att_scale_square * (a.x(var) - b.x(var)) *
              (a.x(var) - b.x(var));
    } else if (var < DroneState::IDX::VEL + DroneState::IDX::NVEL) {
      // velocity
      dist += DroneDistance<float>::vel_scale_square * (a.x(var) - b.x(var)) *
              (a.x(var) - b.x(var));
    } else if (var < DroneState::IDX::OME + DroneState::IDX::NOME) {
      // omega
      dist += DroneDistance<float>::omega_scale_square * (a.x(var) - b.x(var)) *
              (a.x(var) - b.x(var));
    }
  }
  return sqrt(dist);
}

void Drone::rk4(const Ref<const Vector<>> initial_state, const Scalar dt,
                const Ref<const Vector<4>> T, Ref<Vector<>> final_state) {
  static const Vector<4> rk4_sum_vec{1.0 / 6.0, 2.0 / 6.0, 2.0 / 6.0,
                                     1.0 / 6.0};
  Matrix<> k = Matrix<>::Zero(initial_state.rows(), 4);

  final_state = initial_state;

  // k_1
  this->get_derivative(final_state, T, k.col(0));

  // k_2
  final_state = initial_state + 0.5 * dt * k.col(0);
  this->get_derivative(final_state, T, k.col(1));

  // k_3
  final_state = initial_state + 0.5 * dt * k.col(1);
  this->get_derivative(final_state, T, k.col(2));

  // k_4
  final_state = initial_state + dt * k.col(2);
  this->get_derivative(final_state, T, k.col(3));


  final_state = initial_state + dt * k * rk4_sum_vec;
}

void Drone::set_min_max_states(const DroneState min_state,
                               const DroneState max_state) {
  min_state_ = min_state;
  max_state_ = max_state;
  min_max_range_state_.x = max_state.x - min_state.x;
}

void Drone::get_random_state(DroneState& rs, DroneState* end_state_bias) {
  if (end_state_bias == NULL) {
    rs.p = min_max_range_state_.p.cwiseProduct(
             (Vector<3>::Ones(3) + Vector<3>::Random(3)) / 2.0) +
           min_state_.p;
    rs.v = min_max_range_state_.v.cwiseProduct(
             (Vector<3>::Ones(3) + Vector<3>::Random(3)) / 2.0) +
           min_state_.v;

    Quaternion rq = Quaternion::UnitRandom();
    rs.qx(0) = rq.w();
    rs.qx(1) = rq.x();
    rs.qx(2) = rq.y();
    rs.qx(3) = rq.z();
    rs.w = min_max_range_state_.w.cwiseProduct(
             (Vector<3>::Ones(3) + Vector<3>::Random(3)) / 2.0) +
           min_state_.w;
  } else {
    rs.p = min_max_range_state_.p.cwiseProduct(
             (Vector<3>::Ones(3) + Vector<3>::Random(3)) / 2.0) +
           min_state_.p;
    rs.v = min_max_range_state_.v.cwiseProduct(
             (Vector<3>::Ones(3) + Vector<3>::Random(3)) / 2.0) +
           min_state_.v;

    Quaternion rq = Quaternion::UnitRandom();
    rs.qx(0) = rq.w();
    rs.qx(1) = rq.x();
    rs.qx(2) = rq.y();
    rs.qx(3) = rq.z();
    rs.w = min_max_range_state_.w.cwiseProduct(
             (Vector<3>::Ones(3) + Vector<3>::Random(3)) / 2.0) +
           min_state_.w;
  }
  // return rs;
}

Matrix<3, 3> skew(const Vector<3>& v) {
  return (Matrix<3, 3>() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(),
          0)
    .finished();
}

Matrix<4, 4> Q_left(const Quaternion& q) {
  return (Matrix<4, 4>() << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(),
          q.y(), q.y(), q.z(), q.w(), -q.x(), q.z(), -q.y(), q.x(), q.w())
    .finished();
}

Matrix<4, 4> Q_right(const Quaternion& q) {
  return (Matrix<4, 4>() << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), q.z(),
          -q.y(), q.y(), -q.z(), q.w(), q.x(), q.z(), q.y(), -q.x(), q.w())
    .finished();
}

std::ostream& operator<<(std::ostream& o, const DroneState& s) {
  o << std::setprecision(4) << "[t:" << s.t << ";p:" << s.p(0) << "," << s.p(1)
    << "," << s.p(2) << ";q:" << s.qx(0) << "," << s.qx(1) << "," << s.qx(2)
    << "," << s.qx(3) << ";" << std::endl
    << "v:" << s.v(0) << "," << s.v(1) << "," << s.v(2) << ";w:" << s.w(0)
    << "," << s.w(1) << "," << s.w(2) << ";a:" << s.a(0) << "," << s.a(1) << ","
    << s.a(2) << "]";
  o << s.command;
  return o;
}

std::ostream& operator<<(std::ostream& o, const Command& f) {
  o << "comm id " << f.id << " for " << f.time << " type " << f.type
    << " motors " << f.command.transpose() << " rot_ax "
    << f.x_y_rotation_vector.transpose();
  return o;
}

std::string to_string_raw(const Command& c) {
  std::stringstream o;
  for (int var = 0; var < 4; ++var) {
    o << c.command(var) << ",";
  }
  o << c.time << "," << c.id << "," << c.type;

  return o.str();
}

std::string to_string_raw_header(const Command& c) {
  std::stringstream o;
  for (int var = 1; var <= 4; ++var) {
    o << "u_" << var << ",";
  }
  o << "comm_t,comm_id,comm_type";

  return o.str();
}

std::string to_string_raw_header(const DroneState& s) {
  std::stringstream o;
  o << std::setprecision(4)
    << "t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,a_lin_x,a_lin_y,"
       "a_lin_z";
  o << "," << to_string_raw_header(s.command);
  return o.str();
}

std::string to_string_raw(const DroneState& s) {
  std::stringstream o;
  o << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << s.t
    << "," << s.p(0) << "," << s.p(1) << "," << s.p(2) << "," << s.qx(0) << ","
    << s.qx(1) << "," << s.qx(2) << "," << s.qx(3) << "," << s.v(0) << ","
    << s.v(1) << "," << s.v(2) << "," << s.w(0) << "," << s.w(1) << ","
    << s.w(2) << "," << s.a(0) << "," << s.a(1) << "," << s.a(2);
  // t, p_x, p_y, p_z, q_w, q_x, q_y, q_z, v_x, v_y, v_z, w_x, w_y, w_z,
  // a_lin_x, a_lin_y, a_lin_z, a_rot_x, a_rot_y, a_rot_z, u_0, u_1, u_2, u_3
  o << "," << to_string_raw(s.command);
  return o.str();
}

std::tuple<Quaternion, Quaternion> decompose_xy_z(const Quaternion& from_q,
                                                  const Quaternion& to_q) {
  const Quaternion q_e = from_q.inverse() * to_q;
  Quaternion q_z(q_e.w(), 0.0, 0.0, q_e.z());
  q_z.normalize();
  const Quaternion q_xy = q_e * q_z.inverse();

  const Quaternion new_q_to = from_q * q_xy;

  return {new_q_to, q_xy};
}


void DroneState::saveSamplesToFile(
  std::string filename, std::vector<std::vector<DroneState>> samples) {
  std::ofstream myfile;
  myfile.open(filename.c_str());
  if (myfile.is_open()) {
    if (samples.size() > 0 && samples[0].size() > 0) {
      myfile << to_string_raw_header(samples[0][0]) << std::endl;
    }

    const int s1 = samples.size();
    for (int var1 = 0; var1 < s1; ++var1) {
      const int s2 = samples[var1].size();

      for (int var2 = 0; var2 < s2; ++var2) {
        const DroneState& data = samples[var1][var2];
        myfile << to_string_raw(data);
        myfile << std::endl;
      }
    }
    myfile.close();
  }
}

double distance(Vector<3> from, Vector<3> to) { return (from - to).norm(); }
