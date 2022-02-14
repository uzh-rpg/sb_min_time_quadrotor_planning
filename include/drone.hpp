#pragma once

#include <eigen3/Eigen/Eigen>
#include <flann/flann.hpp>
#include <limits>
#include <tuple>

#include "common.hpp"

#define EQUALITY_ERROR (0.0001)
#define PRECISION (1E-4)

// using namespace Eigen;

static constexpr Scalar G = 9.8066;
const Vector<3> GVEC{0, 0, -G};

Matrix<3, 3> skew(const Vector<3>& v);
Matrix<4, 4> Q_left(const Quaternion& q);
Matrix<4, 4> Q_right(const Quaternion& q);

enum CommandType {
  NONE,
  MAX_OMEGA_ROTATION,
  RISE,
  MAX_TAU_POSITIVE,
  MAX_TAU_NEGATIVE,
};

struct Command {
  Command() {}
  Command(double time_, Vector<4> command_, CommandType type_, int id_,
          Vector<3> x_y_rotation_vector_) {
    this->id = id_;
    this->type = type_;
    this->command = command_;
    this->time = time_;
    this->x_y_rotation_vector = x_y_rotation_vector_;
  }
  Command(const Command& command)
    : command(command.command),
      time(command.time),
      id(command.id),
      type(command.type),
      x_y_rotation_vector(command.x_y_rotation_vector) {}
  Vector<4> command = Vector<4>::Constant(NAN);
  Vector<3> x_y_rotation_vector = Vector<3>::Constant(0);
  CommandType type = NONE;
  int id = -1;
  Scalar time = NAN;
};

struct DroneState {
  enum IDX : int {
    POS = 0,
    POSX = 0,
    POSY = 1,
    POSZ = 2,
    NPOS = 3,
    ATT = 3,
    ATTW = 3,
    ATTX = 4,
    ATTY = 5,
    ATTZ = 6,
    NATT = 4,
    VEL = 7,
    VELX = 7,
    VELY = 8,
    VELZ = 9,
    NVEL = 3,
    OME = 10,
    OMEX = 10,
    OMEY = 11,
    OMEZ = 12,
    NOME = 3,
    ACC = 13,
    ACCX = 13,
    ACCY = 14,
    ACCZ = 15,
    NACC = 3,
    //    T = 13,
    //    T1 = 10,
    //    T2 = 11,
    //    T3 = 12,
    //    T4 = 13,
    //    NT = 4,
    SIZE = 16,
    SIZE_NN = 13,
  };

  DroneState(){};
  DroneState(const Vector<IDX::SIZE>& x, const Command& command,
             const Scalar t = NAN)
    : x(x), t(t), command(command) {}
  DroneState(const DroneState& state)
    : x(state.x), t(state.t), command(state.command) {}
  Quaternion getAttitude() const {
    return Quaternion(qx(0), qx(1), qx(2), qx(3));
  }
  void setZero() {
    x.setZero();
    t = 0;
    x(ATTW) = 1.0;
  }
  static void saveSamplesToFile(std::string filename,
                                std::vector<std::vector<DroneState>> samples);

  Vector<IDX::SIZE> x = Vector<IDX::SIZE>::Constant(NAN);

  Scalar t{NAN};
  Command command;

  Ref<Vector<3>> p{x.segment<IDX::NPOS>(IDX::POS)};
  Ref<Vector<4>> qx{x.segment<IDX::NATT>(IDX::ATT)};
  Ref<Vector<3>> v{x.segment<IDX::NVEL>(IDX::VEL)};
  Ref<Vector<3>> w{x.segment<IDX::NOME>(IDX::OME)};
  Ref<Vector<3>> a{x.segment<IDX::NACC>(IDX::ACC)};
  // Ref<Vector<4>> u{x.segment<IDX::NT>(IDX::T)};
};


std::string to_string_raw_header(const DroneState& s);
std::string to_string_raw(const DroneState& s);
std::string to_string_raw_header(const Command& c);
std::string to_string_raw(const Command& c);
std::tuple<Quaternion, Quaternion> decompose_xy_z(const Quaternion& from_q,
                                                  const Quaternion& to_q);

std::ostream& operator<<(std::ostream& o, const DroneState& s);
std::ostream& operator<<(std::ostream& o, const Command& f);

template<class T, class R>
T AttDifference(T a1, R a2) {
  T diff{(T)a2 - a1};
  return diff;
}

// FLANN FUNCTOR
template<class T>
struct DroneDistance {
  typedef bool is_kdtree_distance;

  static T pos_scale_square;
  static T att_scale_square;
  static T vel_scale_square;
  static T omega_scale_square;

  typedef T ElementType;
  typedef typename flann::Accumulator<T>::Type ResultType;

  template<typename Iterator1, typename Iterator2>
  ResultType operator()(Iterator1 a, Iterator2 b, size_t size,
                        ResultType worst_dist = -1) const {
    ResultType result = ResultType();
    ResultType diff;

    // position
    for (int i = DroneState::IDX::POS;
         i < DroneState::IDX::POS + DroneState::IDX::NPOS; ++i) {
      diff = (ResultType)(*a++ - *b++);
      result += pos_scale_square * diff * diff;
    }

    // attitude
    for (size_t i = DroneState::IDX::ATT;
         i < DroneState::IDX::ATT + DroneState::IDX::NATT; ++i) {
      // diff = (ResultType)AttDifference(*a++, *b++);
      diff = (ResultType)(*a++ - *b++);
      result += att_scale_square * diff * diff;
    }

    // velocity
    for (size_t i = DroneState::IDX::VEL;
         i < DroneState::IDX::VEL + DroneState::IDX::NVEL; ++i) {
      diff = (ResultType)(*a++ - *b++);
      result += vel_scale_square * diff * diff;
    }

    // omega
    for (size_t i = DroneState::IDX::OME;
         i < DroneState::IDX::OME + DroneState::IDX::NOME; ++i) {
      diff = (ResultType)(*a++ - *b++);
      result += omega_scale_square * diff * diff;
    }
    return result;
  }

  template<typename U, typename V>
  inline ResultType accum_dist(const U& a, const V& b, int part) const {
    if (part < DroneState::IDX::POS + DroneState::IDX::NPOS) {
      // pos
      return pos_scale_square * (a - b) * (a - b);
    } else if (part < DroneState::IDX::ATT + DroneState::IDX::NATT) {
      // attitude
      // ResultType ang_diff = (ResultType)AttDifference(a, b);
      return att_scale_square * (a - b) * (a - b);
    } else if (part < DroneState::IDX::VEL + DroneState::IDX::NVEL) {
      // velocity
      return vel_scale_square * (a - b) * (a - b);
    } else if (part < DroneState::IDX::OME + DroneState::IDX::NOME) {
      // omega
      return omega_scale_square * (a - b) * (a - b);
    }
    ERROR("this should not happen!!!!!");
    return 0;
  }
};
template<class T>
T DroneDistance<T>::pos_scale_square = 0;
template<class T>
T DroneDistance<T>::att_scale_square = 0;
template<class T>
T DroneDistance<T>::vel_scale_square = 0;
template<class T>
T DroneDistance<T>::omega_scale_square = 0;

double distance(Vector<3> from, Vector<3> to);

class Drone {
 public:
  Drone(const YAML::Node& drone_config);

  Scalar max_t_motor_;
  Scalar min_t_motor_;
  Scalar omega_max_xy;
  Scalar omega_max_z;
  Scalar m_;
  Scalar l_;
  Scalar kappa_;
  Scalar max_acc_;
  Matrix<3, 3> J_;
  Matrix<3, 3> J_inv_;
  Matrix<4, 4> motor_allocation_;
  Matrix<4, 4> motor_allocation_inv_;
  void rk4(const Ref<const Vector<>> initial_state, const Scalar dt,
           const Ref<const Vector<4>> T, Ref<Vector<>> final_state);
  void get_derivative(const Ref<const Vector<DroneState::SIZE>> state,
                      const Ref<const Vector<4>> T,
                      Ref<Vector<DroneState::SIZE>> derivative);
  bool check_limits(const Ref<const Vector<>> state) const;
  bool check_omegas(const Vector<3> omegas) const;
  Scalar distance(const DroneState& a, const DroneState& b);
  Scalar distance_without_pos(const DroneState& a, const DroneState& b);
  Scalar distance_pos(const DroneState& a, const DroneState& b);
  Scalar distance_vel(const DroneState& a, const DroneState& b);
  std::tuple<Scalar, Scalar, Vector<4>, Scalar> get_max_motors_for_rotation(
    const Vector<3> rotation_axis) const;
  Vector<4> get_const_rotation_not_fall_thrust(const Quaternion& q_from,
                                               const Scalar& theta_from,
                                               const Scalar& theta_to,
                                               const Vector<3>& rot_axis,
                                               const Scalar& time);
  Matrix<3, 3> get_Rq_integral(const Scalar& omg0, const Scalar& t,
                               const Scalar& w, const Vector<3>& rot_axis);
  static Vector<3> thrust_acc_from_body_acc(const Vector<3> body_acc);

  DroneState min_state_;
  DroneState max_state_;
  DroneState min_max_range_state_;
  void get_random_state(DroneState& state, DroneState* end_state_bias);
  void set_min_max_states(const DroneState min_state,
                          const DroneState max_state);
};
