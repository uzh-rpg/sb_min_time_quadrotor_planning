#pragma once

#include <string>
#include <vector>

#include "base_map.hpp"
#include "cnpy.h"
#include "common.hpp"
#include "drone.hpp"

class ESDFMap : public BaseMap {
 public:
  ESDFMap();
  void load(std::string filename);
  double getClearence(const Vector<3> pos);
  // cnpy::NpyArray getArray(FILE* fp);
  Vector<3> getMinPos();
  Vector<3> getMaxPos();

  std::pair<Vector<3>, Vector<3>> gradientInVoxelCenter(const Vector<3> pos);

  Vector<3> getRealPosFromIndex(Vector<3> pos_ijk);

 private:
  std::vector<std::vector<std::vector<float>>> map_data;
  Vector<3> centroid;
  Vector<3> extents;
  double num_vexels_per_axis;
  double max_extents_;

  template<typename T>
  void fill_data(T* data, cnpy::NpyArray& voxels_arr);
};

template<typename T>
void ESDFMap::fill_data(T* data, cnpy::NpyArray& voxels_arr) {
  for (size_t xi = 0; xi < voxels_arr.shape[0]; xi++) {
    for (size_t yi = 0; yi < voxels_arr.shape[1]; yi++) {
      for (size_t zi = 0; zi < voxels_arr.shape[2]; zi++) {
        map_data[xi][yi][zi] =
          data[xi * voxels_arr.shape[0] * voxels_arr.shape[1] +
               yi * voxels_arr.shape[1] + zi];
      }
    }
  }
};