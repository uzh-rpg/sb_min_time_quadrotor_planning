
#include "esdf_map.hpp"


ESDFMap::ESDFMap() {}
void ESDFMap::load(std::string filename) {
  INFO("loading from file " << filename);

  FILE* fp = fopen(filename.c_str(), "rb");

  if (!fp)
    throw std::runtime_error("npy_load: Unable to open file " + filename);


  cnpy::NpyArray voxels_arr = getArray(fp);
  cnpy::NpyArray centroid_arr = getArray(fp);
  std::vector<double> centroid_vec = centroid_arr.as_vec<double>();
  cnpy::NpyArray extents_arr = getArray(fp);
  std::vector<double> extents_vec = extents_arr.as_vec<double>();
  for (size_t i = 0; i < 3; i++) {
    centroid(i) = centroid_vec[i];
    extents(i) = extents_vec[i];
  }
  num_vexels_per_axis = voxels_arr.shape[0];
  cnpy::NpyArray resolution_arr = getArray(fp);
  fclose(fp);

  max_extents_ = extents.maxCoeff();

  INFO("centroid " << centroid.transpose())
  INFO("extents " << extents.transpose())
  INFO("resolution " << num_vexels_per_axis)

  int num_voxels_in_rarray = 1;
  for (size_t i = 0; i < voxels_arr.shape.size(); i++) {
    INFO("resolution_arr " << i << " shape " << voxels_arr.shape[i])
    num_voxels_in_rarray *= voxels_arr.shape[i];
  }

  int num_bytes_per_voxel = (voxels_arr.num_bytes() / num_voxels_in_rarray);
  INFO("there should be " << num_voxels_in_rarray << " voxels")

  // std::vector<double> data = voxels_arr.as_vec<double>();
  // INFO("data size " << data.size())

  INFO("num bytes per voxel is " << num_bytes_per_voxel)


  map_data.resize(voxels_arr.shape[0]);
  for (size_t xi = 0; xi < voxels_arr.shape[0]; xi++) {
    // INFO("xi " << xi)
    map_data[xi].resize(voxels_arr.shape[1]);
    for (size_t yi = 0; yi < voxels_arr.shape[1]; yi++) {
      // INFO("yi " << yi)
      map_data[xi][yi].resize(voxels_arr.shape[2]);
    }
  }

  INFO("esdf vector resized")
  if (num_bytes_per_voxel == 4) {
    float* data = voxels_arr.data<float>();
    fill_data<float>(data, voxels_arr);
  } else if (num_bytes_per_voxel == 8) {
    double* data = voxels_arr.data<double>();
    fill_data<double>(data, voxels_arr);
  } else {
    INFO("bad number of bytes per voxel " << num_bytes_per_voxel)
    exit(1);
  }


  INFO("filled voxel map")
}

Vector<3> ESDFMap::getMinPos() { return centroid - extents / 2; }
Vector<3> ESDFMap::getMaxPos() { return centroid + extents / 2; }

Vector<3> ESDFMap::getRealPosFromIndex(Vector<3> pos_ijk) {
  double dp = max_extents_ / (num_vexels_per_axis - 1.0);
  Vector<3> min_ext = centroid - Vector<3>::Ones() * max_extents_ / 2.0;
  Vector<3> pos = min_ext + pos_ijk * dp;
  return pos;
}

double ESDFMap::getClearence(const Vector<3> pos) {
  if ((pos.array() < getMinPos().array()).any() ||
      (pos.array() > getMaxPos().array()).any()) {
    return NAN;
  }
  int xi, yi, zi;
  Vector<3> pos_in_box = pos - centroid;
  pos_in_box *= 2.0 / max_extents_;         // now it is -1 to 1
  pos_in_box += Vector<3>::Ones();          // now it is 0 to 2
  pos_in_box *= num_vexels_per_axis / 2.0;  // now 0 to resolution

  // INFO("getClearence pos " << pos << " centroid " << centroid << " pos_in_box
  // "
  //                          << pos_in_box << " num_vexels_per_axis "
  //                          << num_vexels_per_axis << " max_extents_ "
  //                          << max_extents_)

  xi = round(pos_in_box[0]);
  yi = round(pos_in_box[1]);
  zi = round(pos_in_box[2]);
  // INFO("xi " << xi << " yi " << yi << " zi " << zi)
  if ((xi < 0 || xi >= num_vexels_per_axis) ||
      (yi < 0 || yi >= num_vexels_per_axis) ||
      (zi < 0 || zi >= num_vexels_per_axis)) {
    // INFO("xi " << xi << " yi " << yi << " zi " << zi)
    return NAN;
  }

  // INFO("getClearence " << map_data[xi][yi][zi])
  // INFO(xi << " " << yi << " " << zi)
  // INFO("map_data.size " << map_data.size())
  return map_data[xi][yi][zi];
}

std::pair<Vector<3>, Vector<3>> ESDFMap::gradientInVoxelCenter(
  const Vector<3> pos) {
  int xi, yi, zi;
  Vector<3> pos_in_box = pos - centroid;
  pos_in_box *= 2 / extents.maxCoeff();     // now it is -1 to 1
  pos_in_box += Vector<3>::Ones();          // now it is 0 to 2
  pos_in_box *= num_vexels_per_axis / 2.0;  // now 0 to resolution


  xi = round(pos_in_box[0]);
  yi = round(pos_in_box[1]);
  zi = round(pos_in_box[2]);

  const Vector<3> voxel_center = getRealPosFromIndex(Vector<3>(xi, yi, zi));
  // INFO_VAR(voxel_center.transpose())
  // INFO_VAR(pos.transpose())

  double dvalx = 0;
  double dx = 0;
  if (xi + 1 >= 0 && xi + 1 < num_vexels_per_axis) {
    dvalx += map_data[xi + 1][yi][zi];
    dx += num_vexels_per_axis;
  }
  if (xi - 1 >= 0 && xi - 1 < num_vexels_per_axis) {
    dvalx -= map_data[xi - 1][yi][zi];
    dx += num_vexels_per_axis;
  }

  double dvaly = 0;
  double dy = 0;
  if (yi + 1 >= 0 && yi + 1 < num_vexels_per_axis) {
    dvaly += map_data[xi][yi + 1][zi];
    dy += num_vexels_per_axis;
  }
  if (yi - 1 >= 0 && yi - 1 < num_vexels_per_axis) {
    dvaly -= map_data[xi][yi - 1][zi];
    dy += num_vexels_per_axis;
  }

  double dvalz = 0;
  double dz = 0;
  if (zi + 1 >= 0 && zi + 1 < num_vexels_per_axis) {
    dvalz += map_data[xi][yi][zi + 1];
    dz += num_vexels_per_axis;
  }
  if (zi - 1 >= 0 && zi - 1 < num_vexels_per_axis) {
    dvalz -= map_data[xi][yi][zi - 1];
    dz += num_vexels_per_axis;
  }

  Vector<3> central_gradient(dvalx / dx, dvaly / dy, dvalz / dz);
  return {central_gradient.normalized(), voxel_center};
}
