/*
 * common.cpp
 *
 *  Created on: Oct 12, 2020
 *      Author: robert
 */

#include "common.hpp"

double Point2D::dim = 2;
double Point3D::dim = 3;

/**********************************************************/
/* 						param loader begin 				  */

bool parseArrayParam(const YAML::Node &config, std::string param,
                     std::vector<Point2D> &vector_to_fill) {
  if (config[param.c_str()]) {
    YAML::Node vector = config[param.c_str()];
    std::stringstream ss;
    ss << "loaded " << param << ": [";
    Point2D filling_point;
    int i = 0;
    double x = 0;
    double y = 0;
    for (YAML::const_iterator ti = vector.begin(); ti != vector.end(); ++ti) {
      const YAML::Node &value_in_list = *ti;
      i++;
      if (i == 1) {
        x = value_in_list.as<double>();
      } else if (i >= 2) {
        y = value_in_list.as<double>();
        vector_to_fill.push_back(Point2D(x, y));
        ss << "[" << x << "," << y << "]";
        i = 0;
      }
    }
    ss << "]";
    INFO(ss.str());
  } else {
    ERROR("can not load param " << param);
    return false;
  }
  return true;
}

bool parseArrayParam(const YAML::Node &config, std::string param,
                     std::vector<std::vector<double>> &array2d) {
  if (config[param.c_str()]) {
    YAML::Node vector = config[param.c_str()];
    std::cout << "what " << std::endl;
    std::stringstream ss;
    ss << "loaded " << param << ": [";
    for (YAML::const_iterator ti = vector.begin(); ti != vector.end(); ++ti) {
      const YAML::Node &vector2 = *ti;
      array2d.push_back(std::vector<double>());
      int s = array2d.size() - 1;
      for (YAML::const_iterator ti2 = vector2.begin(); ti2 != vector2.end();
           ++ti2) {
        const YAML::Node &value_in_list = *ti2;
        array2d[s].push_back(value_in_list.as<double>());
        ss << value_in_list.as<double>() << " ";
      }
      ss << std::endl;
      // INFO("array " << param.c_str() << " has " <<
      // value_in_list.as<double>());
    }
    ss << "]";
    std::cout << "what 2 " << std::endl;
    INFO(ss.str());
  } else {
    ERROR("can not load param " << param);
    return false;
  }
  return true;
}

bool parseArrayParam(const YAML::Node &config, std::string param,
                     std::vector<double> &vector_to_fill) {
  if (config[param.c_str()]) {
    YAML::Node vector = config[param.c_str()];
    std::stringstream ss;
    ss << "loaded " << param << ": [";
    for (YAML::const_iterator ti = vector.begin(); ti != vector.end(); ++ti) {
      const YAML::Node &value_in_list = *ti;
      vector_to_fill.push_back(value_in_list.as<double>());
      ss << value_in_list.as<double>() << " ";
      // INFO("array " << param.c_str() << " has " <<
      // value_in_list.as<double>());
    }
    ss << "]";
    INFO(ss.str());
  } else {
    ERROR("can not load param " << param);
    return false;
  }
  return true;
}

/* 						param loader end 				  */
/**********************************************************/

log4cxx::LoggerPtr logger = 0;

void startLogger(const std::string &name) {
  if (logger) {
    std::cerr << "main logger already configured" << std::endl;
  } else {
    logger = getLogger(name.c_str());
    try {
      log4cxx::BasicConfigurator::configure();
    } catch (log4cxx::helpers::Exception &e) {
      std::cerr << "Error : log4cxx::BasicConfigurator" << std::endl;
    }
    atexit(quitLogger);
  }
}

void startLogger(const char *loggerName, const char *configName) {
  // same as above but use config file
  logger = getLogger(loggerName);
  log4cxx::PropertyConfigurator::configure(configName);
}

log4cxx::LoggerPtr getLogger(const char *loggerName) {
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(loggerName);
  return logger;
}

void quitLogger(void) { log4cxx::LogManager::shutdown(); }

/**********************************************************/
/*     some math stuff begin						      */
bool randSeeded = false;
std::uniform_real_distribution<> uniform_dist{0, 1};
std::normal_distribution<> normal_dist{0, 1};
std::default_random_engine random_generator;
// std::mt19937_64 generator;


void seed() {
  // std::random_device rd;
  // std::mt19937 random_generator = std::mt19937_64(rd());
  time_t seedval = time(NULL);
  random_generator = std::default_random_engine(seedval);
  srand(seedval);
  // srand(0);

  uniform_dist(random_generator);
  // srand(0);
  randSeeded = true;
  std::cout << "srand seed" << std::endl;
}

double randDoubleMinMax(double min, double max) {
  if (!randSeeded) {
    // srand(time(NULL));
    seed();
  }
  // std::uniform_real_distribution<> dis(min, std::nextafter(max,
  // std::numeric_limits<double>::max()));
  double random = ((double)rand() / (double)RAND_MAX);
  random = min + random * (max - min);
  // double random = dis(generator);
  return random;
}

int randIntMinMax(int min, int max) {
  if (!randSeeded) {
    seed();
  }
  // std::uniform_int_distribution<int> dis(min, max);
  int random =
    min + (int)(((double)rand() / ((double)RAND_MAX + 1)) * (max - min + 1));
  // int random = dis(generator);
  return random;
}

double rand_real(double min, double max) {
  return (max - min) * normal_dist(random_generator) + min;
}

double rand_uniform(double min, double max) {
  return (max - min) * uniform_dist(random_generator) + min;
}

double normalizeAngle(double angle, double min, double max) {
  double normalized = angle;
  while (normalized < min) {
    normalized += M_2PI;
  }
  while (normalized > max) {
    normalized -= M_2PI;
  }
  return normalized;
}

int sgn(int val) {
  if (val > 0) {
    return 1;
  } else if (val < 0) {
    return -1;
  } else {
    return 0;
  }
}

double sgn(double val) {
  if (val > 0) {
    return 1;
  } else if (val < 0) {
    return -1;
  } else {
    return 0;
  }
}

float sgn(float val) {
  if (val > 0) {
    return 1;
  } else if (val < 0) {
    return -1;
  } else {
    return 0;
  }
}

std::vector<double> range(double min, double max, double step) {
  std::vector<double> range_values;
  if (min <= max) {
    for (double val = 0; val < max; val = val + step) {
      range_values.push_back(val);
    }
    range_values.push_back(max);
  }
  return range_values;
}

double ball_volume(double radius, double dim_double) {
  const double dim_half = dim_double / 2.0;
  const double volume =
    (pow(M_PI, dim_half) / tgamma(dim_half + 1)) * pow(radius, dim_double);
  return volume;
}
double ellipse_volume(double two_major_length, double two_focal_length,
                      double dim_double) {
  const double dim_half = dim_double / 2.0;
  const double axis_lengths = sqrt(two_major_length * two_major_length -
                                   two_focal_length * two_focal_length);
  const double volume = (pow(M_PI, dim_half) / tgamma(dim_half + 1)) *
                        (two_major_length / 2.0) *
                        pow(axis_lengths / 2.0, dim_double - 1.0);
  return volume;
}

Eigen::VectorXd random_ellipsoid_point(Eigen::VectorXd A, Eigen::VectorXd B,
                                       double two_major_length) {
  // 3D ellipsoind axes are -  a = two_major_length/2.0, and others are b=c=
  // axis_lengths / 2.0

  const int dim = A.size();

  const double dim_double = (double)dim;
  Eigen::VectorXd center = (A + B) / 2.0;
  const double two_focal_length = (B - A).norm();

  Eigen::VectorXd a1 = (B - A) / two_focal_length;
  Eigen::VectorXd one1 = Eigen::VectorXd::Zero(dim);
  one1(0) = 1;

  Eigen::MatrixXd M = a1 * one1.transpose();
  // INFO("M "<<M)
  Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::NoQRPreconditioner> svd_result(
    M, Eigen::ComputeFullV | Eigen::ComputeFullU);

  Eigen::VectorXd rot_matrix_mid = Eigen::VectorXd::Ones(dim);
  rot_matrix_mid(dim - 1) =
    svd_result.matrixU().determinant() * svd_result.matrixV().determinant();
  // INFO("rot_matrix_mid "<<rot_matrix_mid)

  Eigen::MatrixXd C = svd_result.matrixU() * rot_matrix_mid.asDiagonal() *
                      svd_result.matrixV().transpose();
  // INFO("C "<<C)

  double axis_lengths = sqrt(two_major_length * two_major_length -
                             two_focal_length * two_focal_length);

  const double volume =
    (2.0 / dim_double) *
    (pow(M_PI, dim_double / 2.0) / tgamma(dim_double / 2.0)) *
    (two_major_length / 2.0) * pow(axis_lengths / 2.0, dim_double - 1.0);

  Eigen::VectorXd L(dim);
  L.fill(axis_lengths / 2.0);
  L(0) = two_major_length / 2.0;
  // INFO("L "<<L);
  // INFO(" C * L.asDiagonal() "<< C * L.asDiagonal())

  const double r = 1.0;  // unit ball
  const double random_radii = r * pow(rand_uniform(0.0, 1.0), 1.0 / dim_double);

  Eigen::VectorXd random_vec(dim);
  for (int var = 0; var < dim; ++var) {
    random_vec(var) = rand_real(0, 1);
  }
  double norm = random_vec.norm();
  Eigen::VectorXd ball_point = random_vec * random_radii / norm;

  Eigen::VectorXd return_point = C * L.asDiagonal() * ball_point + center;

  return return_point;
}

Eigen::VectorXd random_ball_point(Eigen::VectorXd C, double radius) {
  const int dim = C.size();

  const double dim_double = (double)dim;
  const double random_radii =
    radius * pow(rand_uniform(0.0, 1.0), 1.0 / dim_double);
  Eigen::VectorXd random_vec(dim);
  for (int var = 0; var < dim; ++var) {
    random_vec(var) = rand_real(0, 1);
  }
  double norm = random_vec.norm();
  Eigen::VectorXd ball_point = random_vec * random_radii / norm;
  return ball_point;
}

double angleBetween(Eigen::VectorXd a, Eigen::VectorXd b) {
  const double cosval = a.dot(b) / (a.norm() * b.norm());
  return acos(clip(cosval, -1.0, 1.0));
}

/*     some math stuff end   							  */
/**********************************************************/

/*********************************************************/
/*				 for 3d objects end					     */
std::ostream &operator<<(std::ostream &o, const Face &f) {
  o << "f " << f.id1 << " " << f.id2 << " " << f.id3;
  return o;
}

std::ostream &operator<<(std::ostream &o, const Point2D &p) {
  std::cout.precision(6);
  o << std::fixed << " " << std::setprecision(6) << p.x << " "
    << std::setprecision(6) << p.y;
  return o;
}

std::ostream &operator<<(std::ostream &o, const Point3D &p) {
  std::cout.precision(6);
  o << std::fixed << " " << std::setprecision(6) << p.x << " "
    << std::setprecision(6) << p.y << " " << std::setprecision(6) << p.z;
  return o;
}

std::ostream &operator<<(std::ostream &o, const Position3D &p) {
  std::cout.precision(6);
  o << std::fixed << std::setprecision(6) << p.x << " " << std::setprecision(6)
    << p.y << " " << std::setprecision(6) << p.yaw;
  return o;
}

std::ostream &operator<<(std::ostream &o, const RGBColor &c) {
  std::cout.precision(6);
  o << std::fixed << " " << std::setprecision(6) << c.r << " "
    << std::setprecision(6) << c.g << " " << std::setprecision(6) << c.b;
  return o;
}

std::ostream &operator<<(std::ostream &o, const RotationMatrix &p) {
  o << "[" << p.M[0][0] << "," << p.M[0][1] << "," << p.M[0][2] << ","
    << std::endl;
  o << "[" << p.M[1][0] << "," << p.M[1][1] << "," << p.M[1][2] << ","
    << std::endl;
  o << "[" << p.M[2][0] << "," << p.M[2][1] << "," << p.M[2][2] << "]"
    << std::endl;
  return o;
}

std::ostream &operator<<(std::ostream &o, const TransformationMatrix &p) {
  o << "[" << p.M[0][0] << "," << p.M[0][1] << "," << p.M[0][2] << ","
    << p.M[0][3] << "," << std::endl;
  o << "[" << p.M[1][0] << "," << p.M[1][1] << "," << p.M[1][2] << ","
    << p.M[1][3] << "," << std::endl;
  o << "[" << p.M[2][0] << "," << p.M[2][1] << "," << p.M[2][2] << ","
    << p.M[2][3] << "," << std::endl;
  o << "[" << p.M[3][0] << "," << p.M[3][1] << "," << p.M[3][2] << ","
    << p.M[3][3] << "]" << std::endl;
  return o;
}

/*				 for 3d objects end					     */
/*********************************************************/

/*********************************************************/
/*				 for filesystem hangling start   	     */
std::string getFilename(std::string &fullfilename) {
  unsigned found = fullfilename.find_last_of("/\\");
  std::string filename = fullfilename.substr(found + 1);
  return filename;
}

std::string getPath(std::string &fullfilename) {
  unsigned found = fullfilename.find_last_of("/\\");
  std::string path = fullfilename.substr(0, found);
  return path;
}
/*				 for filesystem hangling end     	     */
/*********************************************************/
