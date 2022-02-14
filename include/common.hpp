/*
 * common.h
 *
 *  Created on: Oct 12, 2020
 *      Author: Robert Penicka
 */

#pragma once

#include <math.h>
#include <tclap/CmdLine.h>
#include <yaml-cpp/yaml.h>

#include <cstring>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include "log4cxx/basicconfigurator.h"
#include "log4cxx/helpers/exception.h"
#include "log4cxx/logger.h"
#include "log4cxx/logmanager.h"
#include "log4cxx/propertyconfigurator.h"

/**********************************************************/
/*     logging begin           						      */

extern log4cxx::LoggerPtr logger;

void startLogger(const std::string &name);
void startLogger(const char *loggerName, const char *configName);
log4cxx::LoggerPtr getLogger(const char *loggerName);
void quitLogger(void);

#define INFO(s) LOG4CXX_INFO(logger, s)
#define ERROR(s) LOG4CXX_ERROR(logger, s)
#define OUTPUT_DEFAULT "\033[0m"
#define OUTPUT_BLACK "\033[30m"
#define OUTPUT_RED "\033[31m"
#define OUTPUT_GREEN "\033[32m"
#define OUTPUT_YELLOW "\033[33m"
#define OUTPUT_BLUE "\033[34m"
#define OUTPUT_MAGENTA "\033[35m"
#define OUTPUT_CYAN "\033[36m"
#define OUTPUT_WHITE "\033[37m"

#define ERROR_RED(x) ERROR(OUTPUT_RED << x << OUTPUT_DEFAULT)
#define INFO_RED(x) INFO(OUTPUT_RED << x << OUTPUT_DEFAULT)
#define INFO_YELLOW(x) INFO(OUTPUT_YELLOW << x << OUTPUT_DEFAULT)
#define INFO_MAGENTA(x) INFO(OUTPUT_MAGENTA << x << OUTPUT_DEFAULT)
#define INFO_CYAN(x) INFO(OUTPUT_CYAN << x << OUTPUT_DEFAULT)
#define INFO_GREEN(x) INFO(OUTPUT_GREEN << x << OUTPUT_DEFAULT)
#define INFO_WHITE(x) INFO(OUTPUT_WHITE << x << OUTPUT_DEFAULT)
#define INFO_BLUE(x) INFO(OUTPUT_BLUE << x << OUTPUT_DEFAULT)
#define INFO_BLACK(x) INFO(OUTPUT_BLACK << x << OUTPUT_DEFAULT)
#define INFO_COND(cond, x) \
  if (cond) {              \
    INFO(x);               \
  }

#define INFO_COND_COLOR(cond, color, x) \
  if (cond) {                           \
    INFO(color << x << OUTPUT_DEFAULT); \
  }

#define VARIABLE_STR(s) #s
#define STR(s) VARIABLE_STR(s)
#define INFO_VAR(x) INFO(STR(x) << " = " << x)

#define VALUE_PRINT_DELIMITER (", ")

template<typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
  os << "[";
  for (int i = 0; i < v.size(); ++i) {
    os << v[i];
    if (i != v.size() - 1) os << ", ";
  }
  os << "]";
  return os;
}


template<typename T>
std::ostream &operator<<(std::ostream &os, const std::unordered_set<T> &v) {
  os << "{";

  for (auto it = v.begin(); it != v.end(); ++it) {
    if (it != v.begin()) os << ", ";
    os << *it;
  }
  os << "}";
  return os;
}

/*     logging end           						      */
/**********************************************************/

/**********************************************************/
/*     some math stuff begin						      */
extern std::default_random_engine random_generator;
#define M_2PI (2 * M_PI)
#define POW(x) ((x) * (x))
#define MIN(x, y) ((x > y) ? y : x)
#define MAX(x, y) ((x > y) ? (x) : (y))
#define ABS(x) ((x < 0) ? (-(x)) : (x))
#define ANGLE_MIN (0)
#define ANGLE_MAX (M_2PI)
void seed();
double randDoubleMinMax(double min, double max);
int randIntMinMax(int min, int max);
double normalizeAngle(double angle, double min, double max);
double rand_real_uniform(double min, double max);
int sgn(int val);
float sgn(float val);
double sgn(double val);
std::vector<double> range(double min, double max, double step);

template<typename T>
T clip(const T &n, const T &lower, const T &upper) {
  return std::max(lower, std::min(n, upper));
}
double angleBetween(Eigen::VectorXd a, Eigen::VectorXd b);

// generate random number within an ellipse given by A, B and two_major_length
Eigen::VectorXd random_ellipsoid_point(Eigen::VectorXd A, Eigen::VectorXd B,
                                       double two_major_length);
Eigen::VectorXd random_ball_point(Eigen::VectorXd C, double radius);
double ball_volume(double radius, double dim_double);
double ellipse_volume(double two_major_length, double two_focal_length,
                      double dim_double);

/*     some math stuff end						          */
/**********************************************************/

/*********************************************************/
/*				 for 3d objects start					 */

typedef struct Face {
  unsigned int id1;
  unsigned int id2;
  unsigned int id3;
} Face;

typedef struct Point2D {
  double x;
  double y;
  static double dim;
  Point2D() {
    this->x = NAN;
    this->y = NAN;
  }

  Point2D(double x_, double y_) {
    this->x = x_;
    this->y = y_;
  }

  std::vector<double> toVector() {
    std::vector<double> vector;
    vector.push_back(x);
    vector.push_back(y);
    return vector;
  }

  double distance(Point2D other) {
    double diffx = this->x - other.x;
    double diffy = this->y - other.y;
    return sqrt(diffx * diffx + diffy * diffy);
  }

  Point2D getStateInDistance(Point2D to, double in_distance) {
    double mutual_distance = this->distance(to);
    double x_ = this->x + (to.x - this->x) * (in_distance / mutual_distance);
    double y_ = this->y + (to.y - this->y) * (in_distance / mutual_distance);
    return Point2D(x_, y_);
  }
  std::string toString() {
    std::stringstream ss;
    ss << this->x << VALUE_PRINT_DELIMITER << this->y;
    return ss.str();
  }
} Point2D;

typedef struct Point3D {
  double x;
  double y;
  double z;
  static double dim;
  Point3D() : x(NAN), y(NAN), z(NAN) {}
  Point3D(const double &_x, const double &_y, const double &_z)
    : x(_x), y(_y), z(_z) {}

  Point3D operator+(const Point3D &p) const {
    return Point3D(x + p.x, y + p.y, z + p.z);
  }
  Point3D operator-(const Point3D &p) const {
    return Point3D(x - p.x, y - p.y, z - p.z);
  }
  Point3D operator*(double c) const { return Point3D(c * x, c * y, c * z); }
  Point3D operator/(double c) const { return Point3D(x / c, y / c, z / c); }
  bool operator==(const Point3D &p) const {
    return ((p.x == this->x) && (p.y == this->y) && (p.z == this->z));
  }
  std::vector<double> toVector() {
    std::vector<double> vec(3);
    vec[0] = this->x;
    vec[1] = this->y;
    vec[2] = this->z;
    return vec;
  }

  double distanceTo(const Point3D &p) {
    const double dfx = p.x - this->x;
    const double dfy = p.y - this->y;
    const double dfz = p.z - this->z;
    return sqrt(dfx * dfx + dfy * dfy + dfz * dfz);
  }

  Point3D getStateInDistance(Point3D to, double in_distance) {
    double mutual_distance = this->distanceTo(to);
    double x_ = this->x + (to.x - this->x) * (in_distance / mutual_distance);
    double y_ = this->y + (to.y - this->y) * (in_distance / mutual_distance);
    double z_ = this->z + (to.z - this->z) * (in_distance / mutual_distance);
    return Point3D(x_, y_, z_);
  }

  Point3D &operator=(const std::string &str) {
    std::istringstream iss(str);
    if (!(iss >> x >> y >> z))
      throw TCLAP::ArgParseException(str + " is not a 3D vector");

    return *this;
  }

  void print() {
    std::cout << "[" << x << "," << y << "," << z << ","
              << "]" << std::endl;
  }

} Point3D;

template<typename T>
struct LoadVector {
  std::vector<T> vector;
  LoadVector &operator=(const std::string &str) {
    std::istringstream iss(str);
    std::cout << "[" << str << "]" << std::endl;
    T val;
    while (iss >> val) {
      std::cout << "val = " << val << std::endl;
      if (iss.fail()) {
        throw TCLAP::ArgParseException(str + " is not a vector");
      }
      vector.push_back(val);
    }

    return *this;
  }
};

template<typename T, int dim>
struct LoadVector2D {
  std::vector<std::vector<T>> vector;
  LoadVector2D &operator=(const std::string &str) {
    std::istringstream iss(str);
    std::cout << "[" << str << "]" << std::endl;
    T val;
    int val_count = dim;
    while (iss >> val) {
      std::cout << "val = " << val << std::endl;
      if (iss.fail()) {
        throw TCLAP::ArgParseException(str + " is not a vector");
      }
      if (val_count == dim) {
        vector.push_back(std::vector<T>());
        val_count = 0;
      }
      vector.back().push_back(val);
      val_count++;
    }

    return *this;
  }
};

typedef struct Point3D Vector3D;

typedef struct RotationMatrix {
  double M[3][3];
} RotationMatrix;

typedef struct TransformationMatrix {
  double M[4][4];
} TransformationMatrix;

typedef struct Position3D {
  double x;
  double y;
  double z;
  double yaw;
  double pitch;
  double roll;
  double rotationMatrix[4][4];

  Position3D() : x(0.0), y(0.0), z(0.0), yaw(0.0), pitch(0.0), roll(0.0) {}

  Position3D(double x, double y, double z, double yaw, double pitch,
             double roll) {
    // yaw okolo z
    // pitch okolo y
    // roll okolo x
    this->x = x;
    this->y = y;
    this->z = z;
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
  }

  void setTranslation(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
  }

  void setRotation(double yaw, double pitch, double roll) {
    this->yaw = yaw;
    this->pitch = pitch;
    this->roll = roll;
  }

  void updateRotationMatrix() {
    // XYZ rotation from http://www.songho.ca/opengl/gl_anglestoaxes.html
    // yaw rotation around z
    // pitch rotation around y
    // roll rotation around x
    rotationMatrix[0][3] = this->x;
    rotationMatrix[1][3] = this->y;
    rotationMatrix[2][3] = this->z;
    rotationMatrix[3][3] = 1;

    rotationMatrix[0][0] = cos(this->pitch) * cos(this->yaw);
    rotationMatrix[0][1] = -cos(this->pitch) * sin(this->yaw);
    rotationMatrix[0][2] = sin(this->pitch);

    rotationMatrix[1][0] = sin(this->roll) * sin(this->pitch) * cos(this->yaw) +
                           cos(this->roll) * sin(this->yaw);
    rotationMatrix[1][1] =
      -sin(this->roll) * sin(this->pitch) * sin(this->yaw) +
      cos(this->roll) * cos(this->yaw);
    rotationMatrix[1][2] = -sin(this->roll) * cos(this->pitch);

    rotationMatrix[2][0] =
      -cos(this->roll) * sin(this->pitch) * cos(this->yaw) +
      sin(this->roll) * sin(this->yaw);
    rotationMatrix[2][1] = cos(this->roll) * sin(this->pitch) * sin(this->yaw) +
                           sin(this->roll) * cos(this->yaw);
    rotationMatrix[2][2] = cos(this->roll) * cos(this->pitch);

    rotationMatrix[3][0] = 0;
    rotationMatrix[3][1] = 0;
    rotationMatrix[3][2] = 0;
  }

  RotationMatrix getRotationMatrix() {
    updateRotationMatrix();
    RotationMatrix t;
    memcpy(&t.M[0][0], &(this->rotationMatrix[0][0]), 3 * sizeof(double));
    memcpy(&t.M[1][0], &(this->rotationMatrix[1][0]), 3 * sizeof(double));
    memcpy(&t.M[2][0], &(this->rotationMatrix[2][0]), 3 * sizeof(double));
    return t;
  }

  TransformationMatrix getTransformationMatrix() {
    updateRotationMatrix();
    TransformationMatrix t;
    memcpy(&t.M, this->rotationMatrix, sizeof(TransformationMatrix));
    return t;
  }

  Vector3D getTranslationVector() {
    Vector3D v(this->x, this->y, this->z);
    return v;
  }

  Position3D random(double posMIN, double posMAX, double rotMIN,
                    double rotMAX) {
    Position3D pos;
    pos.randomFill(posMIN, posMAX, rotMIN, rotMAX);
    return pos;
  }

  void randomFill(double posMIN, double posMAX, double rotMIN, double rotMAX) {
    this->x = randDoubleMinMax(posMIN, posMAX);
    this->y = randDoubleMinMax(posMIN, posMAX);
    this->z = randDoubleMinMax(posMIN, posMAX);
    this->yaw = randDoubleMinMax(rotMIN, rotMAX);
    this->pitch = randDoubleMinMax(rotMIN, rotMAX);
    this->roll = randDoubleMinMax(rotMIN, rotMAX);
  }

  double distanceXYZ(Position3D otherPosition) {
    double diffx = otherPosition.getX() - this->x;
    double diffy = otherPosition.getY() - this->y;
    double diffz = otherPosition.getZ() - this->z;
    return sqrt(diffx * diffx + diffy * diffy + diffz * diffz);
  }

  std::vector<double> toVector() {
    std::vector<double> vector(6);
    vector[0] = this->x;
    vector[1] = this->y;
    vector[2] = this->z;
    vector[3] = this->yaw;
    vector[4] = this->pitch;
    vector[5] = this->roll;
    return vector;
  }

  inline bool operator==(const Position3D other) {
    if ((getX() != other.x) || (y != other.y) || (z != other.z) ||
        (yaw != other.yaw) || (pitch != other.pitch) || (roll != other.roll)) {
      return false;
    }
    return true;
  }

  std::ostream &operator<<(std::ostream &o) {
    o << std::fixed << std::setprecision(6) << "[" << x << "," << y << "," << z
      << "," << yaw << "," << pitch << "," << roll << "]";
    return o;
  }

  Position3D operator-(const Position3D &other) {
    return Position3D(x - other.x, y - other.y, z - other.z, yaw - other.yaw,
                      pitch - other.pitch, roll - other.roll);
  }

  void print() {
    std::cout << std::fixed << std::setprecision(6) << "[" << this->getX()
              << "," << this->getY() << "," << this->getZ() << ","
              << this->getYaw() << "," << this->getPitch() << ","
              << this->getRoll() << "]" << std::endl;
  }

  double getX() { return this->x; }
  double getY() { return this->y; }
  double getZ() { return this->z; }
  double getYaw() { return this->yaw; }
  double getPitch() { return this->pitch; }
  double getRoll() { return this->roll; }
  void setX(double x) { this->x = x; }
  void setY(double y) { this->y = y; }
  void setZ(double z) { this->z = z; }
  void setYaw(double yaw) { this->yaw = yaw; }
  void setPitch(double pitch) { this->pitch = pitch; }
  void setRoll(double roll) { this->roll = roll; }

} Position3D;

typedef struct RGBColor {
  float r;
  float g;
  float b;
  RGBColor() : r(0.0), g(0.0), b(0.0) {}
  RGBColor(const float &_r, const float &_g, const float &_b)
    : r(_r), g(_g), b(_b) {}

  RGBColor operator+(const RGBColor &p) const {
    return RGBColor(r + p.r, g + p.g, b + p.b);
  }
  RGBColor operator-(const RGBColor &p) const {
    return RGBColor(r - p.r, g - p.g, b - p.b);
  }
  RGBColor operator*(float c) const { return RGBColor(c * r, c * g, c * b); }
  RGBColor operator/(float c) const { return RGBColor(r / c, g / c, b / c); }
  void print() {
    std::cout << "[" << r << "," << g << "," << b << ","
              << "]" << std::endl;
  }

} RGBColor;

std::ostream &operator<<(std::ostream &o, const Point2D &p);
std::ostream &operator<<(std::ostream &o, const Point3D &p);
std::ostream &operator<<(std::ostream &o, const Position3D &p);
std::ostream &operator<<(std::ostream &o, const RGBColor &c);
std::ostream &operator<<(std::ostream &o, const RotationMatrix &p);
std::ostream &operator<<(std::ostream &o, const TransformationMatrix &p);
/*				 for 3d objects end					 */
/*********************************************************/

/*********************************************************/
/*				 for filesystem hangling start   	     */
#define PATH_SEPARATOR std::string("/")
std::string getFilename(std::string &fullfilename);
std::string getPath(std::string &fullfilename);
/*				 for filesystem hangling end     	     */
/*********************************************************/

/**********************************************************/
/*     config loading begin           				      */

template<typename T>
T loadParam(YAML::Node config, std::string param_name) {
  T val;
  if (config[param_name]) {
    val = config[param_name].as<T>();
    INFO("loaded " << param_name << ": " << val);
  } else {
    ERROR("can not load " << param_name);
    exit(1);
  }
  return val;
}
bool parseArrayParam(const YAML::Node &config, std::string param,
                     std::vector<Point2D> &vector_to_fill);
bool parseArrayParam(const YAML::Node &config, std::string param,
                     std::vector<std::vector<double>> &array2d);
bool parseArrayParam(const YAML::Node &config, std::string param,
                     std::vector<double> &vector_to_fill);
/*     config loading begin    						      */
/**********************************************************/

/**********************************************************/
/*                      types begin    						        */
// used from agilib
using Scalar = double;
static constexpr Scalar INF = std::numeric_limits<Scalar>::infinity();

static constexpr int Dynamic = Eigen::Dynamic;

template<int rows = Dynamic, int cols = rows>
using Matrix = Eigen::Matrix<Scalar, rows, cols>;

template<int rows = Dynamic, int cols = rows>
using Array = Eigen::Array<Scalar, rows, cols>;

template<int rows = Dynamic>
using Vector = Matrix<rows, 1>;

template<int rows = Dynamic>
using ArrayVec = Array<rows, 1>;

// Using `Quaternion` with type.
using Quaternion = Eigen::Quaternion<Scalar>;
using AngleAxis = Eigen::AngleAxis<Scalar>;

template<class Derived>
using Ref = Eigen::Ref<Derived>;

template<class Derived>
using ConstRef = const Eigen::Ref<const Derived>;
/*                       types end     						        */
/**********************************************************/


/**********************************************************/
/*                       TCLAP begin                      */

namespace TCLAP {
template<>
struct ArgTraits<Point3D> {
  typedef StringLike ValueCategory;
};

template<typename T>
struct ArgTraits<LoadVector<T>> {
  typedef StringLike ValueCategory;
};

template<typename T, int dim>
struct ArgTraits<LoadVector2D<T, dim>> {
  typedef StringLike ValueCategory;
};
}  // namespace TCLAP


/*                       TCLAP end                        */
/**********************************************************/