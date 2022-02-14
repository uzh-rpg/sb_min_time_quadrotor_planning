
#include "topological_prm.hpp"


template<>
bool TopologicalPRM<Vector<3>>::isInCollision(Vector<3> object_position,
                                              const double clearance) {
  const double clearance_val = map_->getClearence(object_position);
  // INFO("clearance " << clearance)
  return !std::isfinite(clearance_val) || clearance_val < clearance;
  // return MeshObject::collide(&obstacles, object, object_position);
}

template<>
bool TopologicalPRM<Vector<3>>::isInCollision(Vector<3> object_position) {
  return isInCollision(object_position, min_clearance_);
}

template<>
void TopologicalPRM<Vector<3>>::fillRandomState(
  HeapNode<Vector<3>>* positionToFill) {
  // positionToFill->city_node = false;
  positionToFill->data = (0.5 * (Vector<3>::Ones() + Vector<3>::Random()))
                           .cwiseProduct(position_range_) +
                         min_position_;
}

template<>
void TopologicalPRM<Vector<3>>::fillRandomStateInEllipse(
  HeapNode<Vector<3>>* new_node, HeapNode<Vector<3>>* start,
  HeapNode<Vector<3>>* goal) {
  // INFO("fillRandomStateInEllipse")
  // INFO("start " << start)
  // INFO("goal " << goal)
  const double dist = (goal->data - start->data).norm();
  // INFO("dist " << dist);
  // INFO("start->data " << start->data.transpose());
  // INFO("goal->data " << goal->data.transpose());
  // INFO("ellipse_ratio_major_axis_focal_length_ "
  //  << ellipse_ratio_major_axis_focal_length_);
  Vector<3> rand_ellipse_point = random_ellipsoid_point(
    start->data, goal->data, dist * ellipse_ratio_major_axis_focal_length_);
  // exit(0);
  new_node->data = rand_ellipse_point;
}

/*
check if the path between two vectors is collision free
*/
template<>
std::pair<bool, Vector<3>>
TopologicalPRM<Vector<3>>::isSimplePathFreeBetweenNodes(
  Vector<3> actual, Vector<3> neigbour, const double clearance) {
  // INFO("isSimplePathFreeBetweenNodes " << actual.transpose() << " and "
  //                                      << neigbour.transpose())

  const Vector<3> vec_between = neigbour - actual;
  const double vec_between_norm = vec_between.norm();
  // const Vector<3> vec_between_normalized = vec_between.normalized();
  const double num_points_path =
    ceil((vec_between_norm / collision_distance_check_) + 1.0);
  // INFO_VAR(vec_between_norm)
  // INFO_VAR(num_points_path)
  // do not have to chesk initial and final
  for (int index = 1; index < num_points_path; index += 1) {
    const double length_between_01 = ((double)index / (double)num_points_path);
    // INFO_VAR(length_between_01)
    const Vector<3> object_position = actual + vec_between * length_between_01;
    // INFO("check " << object_position.transpose())
    if (isInCollision(object_position, clearance)) {
      // INFO("collision")
      return {false, object_position};
    }
  }
  // INFO("free")
  return {true, Vector<3>::Constant(NAN)};
}

template<>
std::pair<bool, Vector<3>>
TopologicalPRM<Vector<3>>::isSimplePathFreeBetweenNodes(Vector<3> actual,
                                                        Vector<3> neigbour) {
  return isSimplePathFreeBetweenNodes(actual, neigbour, min_clearance_);
}

template<>
std::vector<Vector<3>> TopologicalPRM<Vector<3>>::samplePath(
  std::vector<HeapNode<Vector<3>>*> path, const double length_tot,
  const double num_samples) {
  // INFO("sample path begin")
  // INFO("path num nodes " << path.size())
  // INFO("pnum_samples " << num_samples)

  std::vector<Vector<3>> samples;
  samples.resize(num_samples);

  int path_current_id = 0;
  double current_path_length =
    distance(path[path_current_id]->data, path[path_current_id + 1]->data);
  double path_current_path_start = 0;

  for (double i = 0; i < num_samples; ++i) {
    double at_length = length_tot * (i / (num_samples - 1));
    if (at_length > path_current_path_start + current_path_length) {
      path_current_id += 1;
      if (path_current_id + 1 >= path.size()) {
        if (at_length - length_tot > PRECISION) {
          ERROR("baaaaaad path_current_id")
          INFO_VAR(at_length)
          INFO_VAR(length_tot)
          exit(1);
        } else {
          // precision metter
          path_current_id -= 1;
          at_length = path_current_path_start + current_path_length;
        }
      } else {
        path_current_path_start += current_path_length;
        current_path_length = distance(path[path_current_id]->data,
                                       path[path_current_id + 1]->data);
      }
    }
    const Vector<3> pos =
      path[path_current_id]->data +
      (path[path_current_id + 1]->data - path[path_current_id]->data) *
        ((at_length - path_current_path_start) / current_path_length);
    samples[i] = pos;
  }

  // INFO("sample path end")
  return samples;
}

template<>
bool TopologicalPRM<Vector<3>>::isDeformablePath(
  std::vector<HeapNode<Vector<3>>*> path1, const double length_tot1,
  std::vector<HeapNode<Vector<3>>*> path2, const double length_tot2,
  const double num_collision_check, const double clearance) {
  // INFO("isDeformablePath clearance " << clearance)
  std::vector<Vector<3>> samples1 =
    samplePath(path1, length_tot1, num_collision_check);
  std::vector<Vector<3>> samples2 =
    samplePath(path2, length_tot2, num_collision_check);

  // for (size_t i = 1; i < samples1.size(); ++i) {
  //   const double dist = (samples1[i] - samples1[i - 1]).norm();
  //   if (dist > collision_distance_check_) {
  //     ERROR_RED("too distant samples")
  //     INFO(i << "/" << samples1.size())
  //     INFO_VAR(dist);
  //     exit(1);
  //   }
  // }
  // for (size_t i = 1; i < samples2.size(); ++i) {
  //   const double dist = (samples2[i] - samples2[i - 1]).norm();
  //   if (dist > collision_distance_check_) {
  //     ERROR_RED("too distant samples")
  //     INFO(i << "/" << samples2.size())
  //     INFO_VAR(dist)
  //     exit(1);
  //   }
  // }
  // INFO_VAR(samples1.size());
  // INFO_VAR(samples2.size());
  // INFO_VAR(num_collision_check)
  for (double i = 0; i < num_collision_check; ++i) {
    if (!isSimplePathFreeBetweenNodes(samples1[i], samples2[i], clearance)
           .first) {
      // INFO("not deformable between " << samples1[i].transpose() << " "
      //                             << samples2[i].transpose())
      const Vector<3> pos =
        isSimplePathFreeBetweenNodes(samples1[i], samples2[i], clearance)
          .second;
      // INFO_VAR(map_->getClearence(pos))
      // std::cout << "samples1 " << samples1 << std::endl;
      // std::cout << "samples2 " << samples2 << std::endl;
      return false;
    }
  }
  return true;
  //     return false;
  //   }

  // int path1_current_id = 0;
  // int path2_current_id = 0;
  // double current_path_length1 =
  //   distance(path1[path1_current_id]->data, path1[path1_current_id +
  //   1]->data);
  // double current_path_length2 =
  //   distance(path2[path2_current_id]->data, path2[path2_current_id +
  //   1]->data);
  // double path1_current_path_start = 0;
  // double path2_current_path_start = 0;
  // for (double i = 0; i <= num_collision_check; ++i) {
  //   const double at_length1 = length_tot1 * (i / num_collision_check);
  //   const double at_length2 = length_tot2 * (i / num_collision_check);
  //   if (at_length1 > path1_current_path_start + current_path_length1) {
  //     if (path1_current_id + 2 >= path1.size()) {
  //       if (at_length1 - (path1_current_path_start + current_path_length1) >
  //           PRECISION) {
  //         ERROR("baaaaaad path1_current_id")
  //         INFO("at_length1 " << at_length1)
  //         INFO("length_tot1 " << length_tot1)
  //         exit(1);
  //       }
  //     } else {
  //       path1_current_id += 1;
  //       path1_current_path_start += current_path_length1;
  //       current_path_length1 = distance(path1[path1_current_id]->data,
  //                                       path1[path1_current_id + 1]->data);
  //     }
  //   }
  //   if (at_length2 > path2_current_path_start + current_path_length2) {
  //     if (path2_current_id + 2 >= path2.size()) {
  //       if (at_length2 - (path2_current_path_start + current_path_length2) >
  //           PRECISION) {
  //         ERROR("baaaaaad path2_current_id")
  //         INFO("at_length2 " << at_length2)
  //         INFO("length_tot2 " << length_tot2)
  //         exit(1);
  //       }
  //     } else {
  //       path2_current_id += 1;
  //       path2_current_path_start += current_path_length2;
  //       current_path_length2 = distance(path2[path2_current_id]->data,
  //                                       path2[path2_current_id + 1]->data);
  //     }
  //   }

  //   const Vector<3> first =
  //     path1[path1_current_id]->data +
  //     (path1[path1_current_id + 1]->data - path1[path1_current_id]->data) *
  //       ((at_length1 - path1_current_path_start) / current_path_length1);
  //   const Vector<3> second =
  //     path2[path2_current_id]->data +
  //     (path2[path2_current_id + 1]->data - path2[path2_current_id]->data) *
  //       ((at_length2 - path2_current_path_start) / current_path_length2);

  //   if (!isSimplePathFreeBetweenNodes(first, second, clearance).first) {
  //     return false;
  //   }
  // }
  return true;
}

template<>
std::string TopologicalPRM<Vector<3>>::to_string_raw(Vector<3> data) {
  std::stringstream ss;
  ss << data(0) << "," << data(1) << "," << data(2);
  return ss.str();
}
