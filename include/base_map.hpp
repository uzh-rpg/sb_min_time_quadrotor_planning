#pragma once

#include "cnpy.h"
#include "common.hpp"
#include "drone.hpp"

class BaseMap {
 public:
  BaseMap(){};
  virtual void load(std::string filename){};
  virtual double getClearence(const Vector<3> pos) = 0;
  virtual Vector<3> getMinPos() { return Vector<3>::Zero(); };
  virtual Vector<3> getMaxPos() { return Vector<3>::Zero(); };
  //   Vector<3> getMinPos() { return centroid - extents / 2; }
  //   Vector<3> getMaxPos() { return centroid + extents / 2; }

  virtual std::pair<Vector<3>, Vector<3>> gradientInVoxelCenter(
    const Vector<3> pos) {
    return {Vector<3>::Zero(), Vector<3>::Zero()};
  };

  static cnpy::NpyArray getArray(FILE* fp) {
    std::vector<size_t> shape;
    size_t word_size;
    bool fortran_order;
    cnpy::parse_npy_header(fp, word_size, shape, fortran_order);

    cnpy::NpyArray arr(shape, word_size, fortran_order);
    size_t nread = fread(arr.data<char>(), 1, arr.num_bytes(), fp);
    if (nread != arr.num_bytes()) {
      INFO("badly read array");
      exit(1);
    }
    INFO("get_array with " << arr.num_bytes() << "bytes end with shape")
    for (size_t i = 0; i < arr.shape.size(); i++) {
      INFO(i << " shape " << arr.shape[i])
    }
    return arr;
  }
};
