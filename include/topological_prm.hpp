#pragma once

#include <algorithm>
#include <flann/flann.hpp>
#include <limits>
#include <memory>
#include <vector>

#include "base_map.hpp"
#include "common.hpp"
#include "dijkstra.hpp"
#include "distinct_path_dfs.hpp"
#include "esdf_map.hpp"
#include "tree_node.hpp"

struct path_node_split {
  int node_id;
  int path_id;
  int node_index_inside_path;
};

template<class T>
class TopologicalPRM {
 public:
  TopologicalPRM(const YAML::Node& planner_config, std::shared_ptr<BaseMap> map,
                 std::string output_folder, T start, T end,
                 double start_yaw_deg = NAN, double end_yaw_deg = NAN);

  void samplePoint();
  void sampleMultiple(const int num_samples);
  void setBorders(Vector<3> min_position, Vector<3> max_position);
  void saveRoadmap(std::string filename);
  static void savePath(std::string filename, std::vector<HeapNode<T>*> path);
  static void savePathSamples(std::string filename, std::vector<T> path);
  static path_with_length<T> shorten_path(path_with_length<T> path,
                                          bool forward = true);

  std::vector<path_with_length<T>> removeWrongGateDirectionPaths(
    std::vector<path_with_length<T>> paths);
  std::vector<path_with_length<T>> removeTooLongPaths(
    std::vector<path_with_length<T>> paths);
  static std::vector<path_with_length<T>> removeEquivalentPaths(
    std::vector<path_with_length<T>> paths);

  std::vector<path_with_length<T>> findDistinctPaths();
  std::vector<path_with_length<T>> findDistinctPathsBlockingSpheres();
  std::vector<path_with_length<T>> findShortestBetween(
    std::vector<int> start_nodes_vec, std::vector<int> end_nodes_vec,
    int depth);
  std::vector<path_with_length<T>> findShortestPath();

  static std::vector<Vector<3>> samplePath(
    std::vector<HeapNode<Vector<3>>*> path, const double length_tot,
    const double num_samples);

  static std::vector<std::vector<path_with_length<Vector<3>>>>
  find_geometrical_paths(const YAML::Node& planner_config,
                         std::shared_ptr<BaseMap> map,
                         std::vector<Vector<3>>& gates_with_start_end_poses,
                         std::vector<double> gates_orientations,
                         std::string output_folder);

  double getEllipseRatioMajorAxis() {
    return ellipse_ratio_major_axis_focal_length_;
  };
  void setEllipseRatioMajorAxis(
    const double ellipse_ratio_major_axis_focal_length) {
    ellipse_ratio_major_axis_focal_length_ =
      ellipse_ratio_major_axis_focal_length;
  };

 private:
  void fillRandomState(HeapNode<T>* positionToFill);
  void fillRandomStateInEllipse(HeapNode<T>* positionToFill, HeapNode<T>* start,
                                HeapNode<T>* goal);
  static bool isInCollision(T object_position);
  static bool isInCollision(T object_position, const double clearance);
  static std::pair<bool, T> isSimplePathFreeBetweenNodes(T actual, T neigbour);
  static std::pair<bool, T> isSimplePathFreeBetweenNodes(
    T actual, T neigbour, const double clearance);
  static std::pair<bool, T> isPathCollisionFree(path_with_length<T> path);
  bool isSimplePathFreeBetweenNodes(HeapNode<T>* actual, HeapNode<T>* neigbour);
  std::vector<HeapNode<T>*> getVisibleGuards(HeapNode<T>* node);
  std::map<HeapNode<T>*, double> nodesBetweenNodes(HeapNode<T>* node1,
                                                   HeapNode<T>* node2);
  static bool isDeformablePathBetween(HeapNode<T>* start, HeapNode<T>* end,
                                      HeapNode<T>* between1,
                                      HeapNode<T>* between2);
  static bool isDeformablePath(std::vector<HeapNode<T>*> path1,
                               const double length_tot1,
                               std::vector<HeapNode<T>*> path2,
                               const double length_tot2,
                               const double num_collision_check);
  static bool isDeformablePath(std::vector<HeapNode<T>*> path1,
                               const double length_tot1,
                               std::vector<HeapNode<T>*> path2,
                               const double length_tot2,
                               const double num_collision_check,
                               const double clearance);

  static std::string to_string_raw(T data);
  // static double distance(HeapNode<T>* from, HeapNode<T>* to);


  // std::vector<HeapNode<T>*> cities_nodes_;
  std::vector<HeapNode<T>*> guard_nodes_;
  std::vector<HeapNode<T>*> connection_nodes_;
  static std::shared_ptr<BaseMap> map_;
  static std::string output_folder_;
  static double collision_distance_check_;
  static double min_clearance_;
  static double angle_limit_start_end_;
  static double cutoof_distance_ratio_to_shortest_;
  static double ellipse_sampling_density_;

  double ellipse_ratio_major_axis_focal_length_;

  HeapNode<T>* start_;
  HeapNode<T>* end_;

  Quaternion startq_;
  Quaternion endq_;
  bool constraint_start_;
  bool constraint_end_;
  Vector<3> min_position_, max_position_, position_range_;

  Dijkstra<T> dijkstra;
};

template<class T>
std::shared_ptr<BaseMap> TopologicalPRM<T>::map_;
template<class T>
double TopologicalPRM<T>::collision_distance_check_;
template<class T>
double TopologicalPRM<T>::min_clearance_;
template<class T>
double TopologicalPRM<T>::angle_limit_start_end_;
template<class T>
double TopologicalPRM<T>::cutoof_distance_ratio_to_shortest_;

template<class T>
double TopologicalPRM<T>::ellipse_sampling_density_;

template<class T>
std::string TopologicalPRM<T>::output_folder_("./");


template<class T>
TopologicalPRM<T>::TopologicalPRM(const YAML::Node& planner_config,
                                  std::shared_ptr<BaseMap> map,
                                  std::string output_folder, T start, T end,
                                  double start_yaw_deg, double end_yaw_deg) {
  map_ = map;
  output_folder_ = output_folder;
  start_ = new HeapNode<T>(start);
  start_->city_node = true;
  start_->id = 0;
  end_ = new HeapNode<T>(end);
  end_->city_node = true;
  end_->id = 1;

  angle_limit_start_end_ = M_PI_2;
  INFO_VAR(start_yaw_deg)
  INFO_VAR(end_yaw_deg)
  if (!isnan(start_yaw_deg)) {
    startq_ = Quaternion(cos((M_PI / 180.0) * start_yaw_deg / 2.0), 0, 0,
                         sin((M_PI / 180.0) * start_yaw_deg / 2.0));
    constraint_start_ = true;
  } else {
    constraint_start_ = false;
  }
  if (!isnan(end_yaw_deg)) {
    endq_ = Quaternion(cos((M_PI / 180.0) * end_yaw_deg / 2.0), 0, 0,
                       sin((M_PI / 180.0) * end_yaw_deg / 2.0));
    constraint_end_ = true;
  } else {
    constraint_end_ = false;
  }

  // INFO_VAR(constraint_start_)
  // INFO_VAR(constraint_end_)
  // INFO(startq_.w() << " " << startq_.vec().transpose())
  // INFO(endq_.w() << " " << endq_.vec().transpose())
  cutoof_distance_ratio_to_shortest_ =
    loadParam<double>(planner_config, "cutoof_distance_ratio_to_shortest");
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");
  collision_distance_check_ =
    loadParam<double>(planner_config, "collision_distance_check");
  ellipse_ratio_major_axis_focal_length_ =
    loadParam<double>(planner_config, "ellipse_ratio_major_axis_focal_length");
  if (collision_distance_check_ == 0) {
    ERROR(
      "you need to specify collision_distance_check for sampling-based motion "
      "planning");
    exit(1);
  }

  guard_nodes_.push_back(start_);
  guard_nodes_.push_back(end_);
}

/*
similar to
Integrated online trajectory planning and optimization in distinctive topologies
it is not much working
*/
template<class T>
void TopologicalPRM<T>::sampleMultiple(const int num_samples) {
  INFO("sampleMultiple begin")
  std::vector<HeapNode<T>*> samples;
  const int num_nn = 14;

  INFO_VAR(constraint_start_)
  INFO_VAR(constraint_end_)

  flann::Matrix<float> flann_matrix(new float[num_samples * 3], num_samples, 3);
  samples.push_back(start_);
  flann_matrix[0][0] = start_->data(0);
  flann_matrix[0][1] = start_->data(1);
  flann_matrix[0][2] = start_->data(2);
  samples.push_back(end_);
  flann_matrix[1][0] = end_->data(0);
  flann_matrix[1][1] = end_->data(1);
  flann_matrix[1][2] = end_->data(2);

  for (size_t i = 2; i < num_samples; i++) {
    HeapNode<T>* new_node = new HeapNode<T>();
    new_node->id = i;
    do {
      fillRandomStateInEllipse(new_node, start_, end_);
    } while (isInCollision(new_node->data));
    samples.push_back(new_node);
    // INFO_VAR(new_node->data)
    // INFO_VAR(map_->getClearence(new_node->data));
    flann_matrix[i][0] = new_node->data(0);
    flann_matrix[i][1] = new_node->data(1);
    flann_matrix[i][2] = new_node->data(2);
  }
  INFO("points sampled")
  flann::IndexParams params = flann::KDTreeIndexParams(4);
  flann::Index<flann::L2<float>> flann_indexes(flann_matrix, params);
  std::vector<std::vector<int>> indices;
  std::vector<std::vector<float>> dists;
  flann_indexes.buildIndex();
  INFO("index built")

  const int num_found = flann_indexes.knnSearch(
    flann_matrix, indices, dists, num_nn, flann::SearchParams(128));

  INFO("searched with num_found " << num_found)
  Vector<3> vec_start_goal_normalized =
    (end_->data - start_->data).normalized();

  static constexpr auto comparator_distance_point =
    [](const std::pair<double, HeapNode<T>*>& a,
       const std::pair<double, HeapNode<T>*>& b) -> bool {
    return a.first < b.first;
  };
  // max angle between start-end vector and the newly added edges
  const double max_angle = 2.0 * M_PI / 3.0;
  // INFO(indices.size())
  for (size_t fromi = 0; fromi < num_samples; ++fromi) {
    HeapNode<T>* from_node = samples[fromi];

    if (from_node == end_) continue;  // nothing goes from end


    std::vector<std::pair<double, HeapNode<T>*>> distance_neighbors;
    for (size_t nni = 1; nni < num_nn;
         ++nni) {  // skip 0 as it is the same point
      const int nnindex = indices[fromi][nni];
      // INFO("nnindex " << nnindex)
      // INFO("dist " << dists[fromi][nni])


      HeapNode<T>* to_node = samples[nnindex];
      if (to_node == start_) continue;  // nothing goes to start

      Vector<3> vect_between = to_node->data - from_node->data;

      const double vect_between_norm = vect_between.norm();

      // check the path angle with start-goal angle is same
      // constraint all edges to not go back
      // const double angle_between =
      //   angleBetween(vect_between, vec_start_goal_normalized);
      // if (angle_between > max_angle) continue;

      if (from_node == start_ && constraint_start_) {
        const Vector<3> fromgate = startq_ * Vector<3>::UnitX();
        const Vector<3> new_vec = to_node->data - start_->data;
        // INFO("constraint_start_ dot " << fromgate.dot(new_vec))
        if (fromgate.dot(new_vec) < 0) {
          continue;
        }
      }

      if (to_node == end_ && constraint_end_) {
        const Vector<3> fromgate = endq_ * Vector<3>::UnitX();
        const Vector<3> new_vec = end_->data - from_node->data;
        // INFO("constraint_end_ dot " << fromgate.dot(new_vec))
        if (fromgate.dot(new_vec) < 0) {
          continue;
        }
      }

      // check collisions
      if (isSimplePathFreeBetweenNodes(from_node, to_node)) {
        from_node->visibility_node_ids[to_node] = vect_between_norm;
        // distance_neighbors.push_back({vect_between_norm, to_node});
      }
    }

    // std::sort(distance_neighbors.begin(), distance_neighbors.end(),
    //           comparator_distance_point);
    // for (size_t i = 0; i < std::min(distance_neighbors.size(), 4ul); i++) {
    //   from_node->visibility_node_ids[distance_neighbors[i].second] =
    //     distance_neighbors[i].first;
    // }
  }

  connection_nodes_ = samples;
  INFO("sampleMultiple end")
}


/*
sample individual points and connect them imediately based number of guard
created
similar to raptor
it has problem that it does not find the shortest path......
*/
template<class T>
void TopologicalPRM<T>::samplePoint() {
  // random point
  // INFO("sample point")
  // INFO("collision_distance_check_" << collision_distance_check_)
  // INFO("min_clearance_" << min_clearance_)

  HeapNode<T>* new_node = new HeapNode<T>();
  Vector<3> vec_start_goal_normalized =
    (end_->data - start_->data).normalized();

  do {
    fillRandomStateInEllipse(new_node, start_, end_);
  } while (isInCollision(new_node->data));

  std::vector<HeapNode<T>*> visible_cities = getVisibleGuards(new_node);
  // INFO("visible_cities.size() " << visible_cities.size())
  if (visible_cities.size() == 0) {
    // INFO("new guard")
    guard_nodes_.push_back(new_node);
  } else if (visible_cities.size() == 2) {
    bool distinct = true;

    const double max_angle = 0;  // cos(2.0 * M_PI / 3.0);
    // sc[0] -> new_node -> sc[1]
    Vector<3> vec_vis0_new = new_node->data - visible_cities[0]->data;
    Vector<3> vec_new_vis1 = visible_cities[1]->data - new_node->data;
    bool colinear1 =
      vec_start_goal_normalized.dot(vec_vis0_new) / vec_vis0_new.norm() >
      max_angle;
    colinear1 &=
      vec_start_goal_normalized.dot(vec_new_vis1) / vec_new_vis1.norm() >
      max_angle;

    // sc[1] -> new_node -> sc[0] == -vect_between1_aft and -vect_between1_bef
    bool colinear2 =
      vec_start_goal_normalized.dot(-vec_new_vis1) / vec_new_vis1.norm() >
      max_angle;
    colinear2 &=
      vec_start_goal_normalized.dot(-vec_vis0_new) / vec_vis0_new.norm() >
      max_angle;


    // check the path angle with start-goal angle is same
    if (!colinear1 and !colinear2) {
      return;
    }
    // now there is path between visible_cities[0] new_node and
    // visible_cities[1]
    std::map<HeapNode<T>*, double> between_nodes =
      nodesBetweenNodes(visible_cities[0], visible_cities[1]);

    for (const auto& nb : between_nodes) {
      // check if one path visible_cities[0] new_node visible_cities[1] is
      // deformable to other visible_cities[0] nb.first visible_cities[1]
      bool is_deformable = isDeformablePathBetween(
        visible_cities[0], visible_cities[1], new_node, nb.first);
      if (is_deformable) {
        distinct = false;
        const double dist1 = distance(visible_cities[0]->data, new_node->data);
        const double dist2 = distance(new_node->data, visible_cities[1]->data);
        // const double distance_new = distance(visible_cities[0], new_node) +
        //                             distance(new_node, visible_cities[1]);
        if (dist1 + dist2 < nb.second) {
          nb.first->data = new_node->data;
          nb.first->visibility_node_ids[visible_cities[0]] = dist1;
          visible_cities[0]->visibility_node_ids[nb.first] = dist1;
          nb.first->visibility_node_ids[visible_cities[1]] = dist2;
          visible_cities[1]->visibility_node_ids[nb.first] = dist2;
          // INFO("improved pos " << new_node->data)
        }
        break;
      }
    }

    if (distinct) {
      if (colinear1) {
        // sc[0] -> new_node -> sc[1]
        visible_cities[0]->visibility_node_ids[new_node] = vec_vis0_new.norm();
        new_node->visibility_node_ids[visible_cities[1]] = vec_new_vis1.norm();
        connection_nodes_.push_back(new_node);
      }
      if (colinear2) {
        // sc[1] -> new_node -> sc[0]
        visible_cities[1]->visibility_node_ids[new_node] = vec_new_vis1.norm();
        new_node->visibility_node_ids[visible_cities[0]] = vec_vis0_new.norm();
        connection_nodes_.push_back(new_node);
      }
    }
  }
}

template<class T>
std::vector<HeapNode<T>*> TopologicalPRM<T>::getVisibleGuards(
  HeapNode<T>* node) {
  std::vector<HeapNode<T>*> visible_cities;
  // INFO("get visible guards")
  // for (size_t i = 0; i < cities_nodes_.size(); i++) {
  //   if (isSimplePathFreeBetweenNodes(cities_nodes_[i], node)) {
  //     visible_cities.push_back(cities_nodes_[i]);
  //   }
  // }
  for (size_t i = 0; i < guard_nodes_.size(); i++) {
    if (isSimplePathFreeBetweenNodes(guard_nodes_[i], node)) {
      // require start to be reached from direction of going through gate
      if (guard_nodes_[i] == start_ && constraint_start_) {
        const Vector<3> fromgate = startq_ * Vector<3>::UnitX();
        const Vector<3> new_vec = node->data - guard_nodes_[i]->data;
        // INFO("constraint_start_ dot " << fromgate.dot(new_vec))
        if (fromgate.dot(new_vec) < 0) {
          continue;
        }
      }
      // require end to be reached from direction of going through gate
      if (guard_nodes_[i] == end_ && constraint_end_) {
        const Vector<3> fromgate = endq_ * Vector<3>::UnitX();
        // INFO_VAR(fromgate)
        const Vector<3> new_vec = guard_nodes_[i]->data - node->data;
        // INFO("constraint_end_ dot " << fromgate.dot(new_vec))
        if (fromgate.dot(new_vec) < 0) {
          continue;
        }
      }
      visible_cities.push_back(guard_nodes_[i]);
    }
  }
  return visible_cities;
}

template<class T>
std::map<HeapNode<T>*, double> TopologicalPRM<T>::nodesBetweenNodes(
  HeapNode<T>* node1, HeapNode<T>* node2) {
  std::map<HeapNode<T>*, double> between_nodes;
  typename std::map<HeapNode<T>*, double>::iterator it;
  // std::vector<HeapNode<T>*> between_nodes;
  for (const auto& vn1 : node1->visibility_node_ids) {
    it = vn1.first->visibility_node_ids.find(node2);
    if (it != vn1.first->visibility_node_ids.end()) {
      between_nodes.insert(
        std::pair<HeapNode<T>*, double>(vn1.first, vn1.second + it->second));
    }
  }
  return between_nodes;
}


template<class T>
bool TopologicalPRM<T>::isDeformablePathBetween(HeapNode<T>* start,
                                                HeapNode<T>* end,
                                                HeapNode<T>* between1,
                                                HeapNode<T>* between2) {
  const double length_b1_1 = distance(start->data, between1->data);
  const double length_b1_2 = distance(between1->data, end->data);
  const double length_b1_tot = length_b1_1 + length_b1_2;
  const double length_b2_1 = distance(start->data, between2->data);
  const double length_b2_2 = distance(between2->data, end->data);
  const double length_b2_tot = length_b2_1 + length_b2_2;
  //
  const double larger_length = std::max(length_b1_tot, length_b2_tot);
  const double num_check_collision =
    ceil(larger_length / collision_distance_check_) + 1;


  bool deformable = isDeformablePath({start, between1, end}, length_b1_tot,
                                     {start, between2, end}, length_b2_tot,
                                     num_check_collision);
  // std::vector<Vector<3>> second = interpolatePath(, num_check_collision);


  return deformable;
}

template<class T>
bool TopologicalPRM<T>::isSimplePathFreeBetweenNodes(HeapNode<T>* actual,
                                                     HeapNode<T>* neigbour) {
  return isSimplePathFreeBetweenNodes(actual->data, neigbour->data).first;
}

template<class T>
std::pair<bool, T> TopologicalPRM<T>::isPathCollisionFree(
  path_with_length<T> path) {
  std::pair<bool, T> cp;
  for (size_t i = 1; i < path.plan.size(); i++) {
    cp =
      isSimplePathFreeBetweenNodes(path.plan[i - 1]->data, path.plan[i]->data);
    if (!cp.first) {
      return cp;
    }
  }
  return cp;
}

template<class T>
void TopologicalPRM<T>::setBorders(Vector<3> min_position,
                                   Vector<3> max_position) {
  min_position_ = min_position;
  max_position_ = max_position;
  position_range_ = max_position - min_position;
}

template<class T>
void TopologicalPRM<T>::saveRoadmap(std::string filename) {
  // INFO("save TopologicalPRM map to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    // for (size_t i = 0; i < guard_nodes_.size(); i++) {
    //   std::string city_node_str = to_string_raw(guard_nodes_[i]->data);
    //   myfile << city_node_str << std::endl;
    //   for (const auto& vn1 : guard_nodes_[i]->visibility_node_ids) {
    //     std::string neighbor_str = to_string_raw(vn1.first->data);
    //     ss_connections << city_node_str << "," << neighbor_str << std::endl;
    //   }
    // }
    for (size_t i = 0; i < connection_nodes_.size(); i++) {
      std::string city_node_str = to_string_raw(connection_nodes_[i]->data);
      myfile << city_node_str << std::endl;
      for (const auto& vn1 : connection_nodes_[i]->visibility_node_ids) {
        std::string neighbor_str = to_string_raw(vn1.first->data);
        ss_connections << city_node_str << "," << neighbor_str << std::endl;
      }
    }
    myfile << ss_connections.str();
    myfile.close();
  }
}

template<class T>
void TopologicalPRM<T>::savePath(std::string filename,
                                 std::vector<HeapNode<T>*> path) {
  // INFO("save TopologicalPRM map to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    for (size_t ni = 1; ni < path.size(); ni++) {
      std::string from_str = to_string_raw(path[ni - 1]->data);
      std::string to_str = to_string_raw(path[ni]->data);
      myfile << from_str << "," << to_str << std::endl;
    }
    myfile.close();
  }
}

template<class T>
void TopologicalPRM<T>::savePathSamples(std::string filename,
                                        std::vector<T> path) {
  // INFO("savePathSamples to file " << filename);
  std::ofstream myfile;
  myfile.open(filename.c_str());
  std::stringstream ss_connections;

  if (myfile.is_open()) {
    for (size_t ni = 0; ni < path.size(); ni++) {
      std::string from_str = to_string_raw(path[ni]);
      myfile << from_str << std::endl;
    }
    myfile.close();
  }
}

template<class T>
std::vector<path_with_length<T>> TopologicalPRM<T>::findDistinctPaths() {
  INFO("findDistinctPaths begin")
  std::vector<HeapNode<T>*> all_nodes = connection_nodes_;
  all_nodes.insert(all_nodes.end(), guard_nodes_.begin() + 2,
                   guard_nodes_.end());  // do not add start and end
  DistinctPathDFS<T> dpDFS(all_nodes, start_, end_);
  std::vector<path_with_length<T>> distinct_paths = dpDFS.findPaths();
  INFO("findDistinctPaths end")
  return distinct_paths;
}

template<class T>
std::vector<path_with_length<T>>
TopologicalPRM<T>::findDistinctPathsBlockingSpheres() {
  INFO("findDistinctPathsBlockingSpheres begin");


  for (size_t i = 0; i < connection_nodes_.size(); i++) {
    if (i != connection_nodes_[i]->id) {
      INFO("not corresponding id.....");
      exit(1);
    }
  }

  // start is 0, end is 1
  std::vector<path_with_length<T>> found_paths =
    findShortestBetween({0}, {1}, 0);

  INFO("findDistinctPathsBlockingSpheres end");
  return found_paths;
}

template<class T>
std::vector<path_with_length<T>> TopologicalPRM<T>::findShortestBetween(
  std::vector<int> start_nodes_vec, std::vector<int> end_nodes_vec, int depth) {
  INFO("findShortestBetween begin with depth " << depth)

  // std::vector<int> end_nodes_vec(end_nodes.begin(), end_nodes.end());
  // std::vector<int> start_nodes_vec(start_nodes.begin(), start_nodes.end());

  std::unordered_set<int> start_nodes_set(start_nodes_vec.begin(),
                                          start_nodes_vec.end());
  std::unordered_set<int> end_nodes_set(end_nodes_vec.begin(),
                                        end_nodes_vec.end());

  std::cout << " start_nodes " << start_nodes_vec << std::endl;
  std::cout << " end_nodes " << end_nodes_vec << std::endl;

  std::unordered_map<int, path_node_split> deleted_nodes;
  std::vector<path_with_length<T>> found_paths;


  const int num_test = 30;

  for (size_t i = 0; i < num_test; i++) {
    INFO_CYAN("test " << i)

    // std::vector<path_with_length<T>> shortest = findShortestPath();
    std::vector<path_with_length<T>> shortest;
    for (size_t starti = 0; starti < start_nodes_vec.size(); starti++) {
      std::vector<path_with_length<T>> shortest_per_start = dijkstra.findPath(
        start_nodes_vec[starti], end_nodes_vec, connection_nodes_);
      shortest.insert(shortest.end(), shortest_per_start.begin(),
                      shortest_per_start.end());
    }


    // INFO("size shortest " << shortest.size())

    bool no_shortest_found = true;

    // loop found shortest paths
    for (size_t shortest_i = 0; shortest_i < shortest.size(); shortest_i++) {
      // consider only the one that has some nodes == plan found
      if (shortest[shortest_i].plan.size() > 0) {
        // INFO("path " << shortest_i << " shortest_length "
        //              << shortest[shortest_i].length << " is:")

        found_paths.push_back(shortest[shortest_i]);
        // found_paths.back().print();

        double smallest_distance_from_obstacle = DBL_MAX;
        HeapNode<T>* node_smallest = NULL;
        int pos_smallest_in_path = 0;

        // find smalles clearance path
        // do not consider start and end nodes
        for (size_t si = 0; si < shortest[shortest_i].plan.size(); ++si) {
          // INFO_VAR(si)
          HeapNode<T>* node = shortest[shortest_i].plan[si];

          const double clearance = map_->getClearence(node->data);

          if (isfinite(clearance) &&
              clearance < smallest_distance_from_obstacle &&
              end_nodes_set.count(node->id) == 0 &&
              start_nodes_set.count(node->id) == 0) {
            smallest_distance_from_obstacle = clearance;
            node_smallest = node;
            pos_smallest_in_path = si;
            no_shortest_found = false;  // shortest found with deletable node
          }
        }

        // if some node between
        if (node_smallest != NULL) {
          // INFO("smallest_distance " << smallest_distance_from_obstacle << "
          // id "
          //                           << node_smallest->id)
          path_node_split to_delete;
          to_delete.node_id = node_smallest->id;
          to_delete.path_id = found_paths.size() - 1;
          to_delete.node_index_inside_path = pos_smallest_in_path;
          deleted_nodes[to_delete.node_id] = to_delete;


          node_smallest->visibility_node_ids.clear();
          // INFO("remove " << node_smallest->id << "  from roadmap")
          const double collision_free_radius =
            smallest_distance_from_obstacle - min_clearance_;
          // INFO_VAR(collision_free_radius);
          // exit(1);
          // do not consider start_nodes and end_nodes
          for (size_t cni = 0; cni < connection_nodes_.size(); cni++) {
            if ((connection_nodes_[cni]->data - node_smallest->data).norm() <
                  collision_free_radius &&
                connection_nodes_[cni]->id != node_smallest->id &&
                end_nodes_set.count(cni) == 0 &&
                start_nodes_set.count(cni) == 0) {
              // other_close_nodes.push_back(connection_nodes_[cni]->id);


              path_node_split to_delete_other;
              to_delete_other.node_id = connection_nodes_[cni]->id;
              to_delete_other.path_id = found_paths.size() - 1;
              to_delete_other.node_index_inside_path = pos_smallest_in_path;
              if (deleted_nodes.count(to_delete_other.node_id) > 0) {
                INFO_RED("deleted already contains this node id "
                         << to_delete_other.node_id)
                // exit(1);
              }
              deleted_nodes[to_delete_other.node_id] = to_delete_other;

              connection_nodes_[cni]->visibility_node_ids.clear();
              // INFO("remove " << connection_nodes_[cni]->id << "  from
              // roadmap")
            }
          }
        }
      }
    }

    // debug test begin
    /*
    for (auto dn : deleted_nodes) {
      bool contains = false;
      for (size_t fpi = 0; fpi < found_paths[dn.second.path_id].plan.size();
           fpi++) {
        if (found_paths[dn.second.path_id].plan[fpi]->id == dn.second.node_id) {
          contains = true;
          break;
        }
      }
      if (!contains) {
        INFO_VAR(dn.second.path_id)
        INFO_VAR(dn.second.node_id)
        INFO_VAR(dn.second.node_index_inside_path)
        INFO_VAR(found_paths.size())
        INFO("it does not contain it")
        exit(1);
      }
    }
    */
    // debug test end

    // if no shortest found then do the recursion
    if (no_shortest_found) {
      INFO("no other shortest found")
      if (deleted_nodes.size() > 0) {
        // for (auto dn : deleted_nodes) {
        //   INFO("deleted " << connection_nodes_[dn.first]->data.transpose())
        //   // dn.second.path_id
        // }

        if (depth < 1) {
          std::vector<int> deleted_ids;
          for (auto dn : deleted_nodes) {
            deleted_ids.push_back(dn.first);
          }
          std::cout << "deleted_ids " << deleted_ids << std::endl;


          /*find different paths before the one deleted begin*/
          std::vector<path_with_length<T>> part_before =
            findShortestBetween(start_nodes_vec, deleted_ids, depth + 1);
          // std::cout << "deleted_ids was " << deleted_ids << std::endl;
          for (size_t pbi = 0; pbi < part_before.size(); pbi++) {
            // INFO("part_before[" << pbi << "] size "
            //                     << part_before[pbi].plan.size())
            // INFO_VAR(part_before[pbi].to_id)
            // INFO_VAR(part_before[pbi].from_id)
            if (part_before[pbi].plan.size() > 0) {
              if (deleted_nodes.count(part_before[pbi].to_id) == 0) {
                INFO("path deleted_nodes should contain path end")
                INFO_VAR(part_before[pbi].to_id)
                exit(1);
              }
              path_node_split dn = deleted_nodes[part_before[pbi].to_id];

              // now create other paths using the part_before and the original
              // path
              path_with_length<T>& original = found_paths[dn.path_id];
              path_with_length<T> new_path;
              new_path.plan = part_before[pbi].plan;
              // INFO("original")
              // original.print();
              // INFO("new before part")
              // part_before[pbi].print();
              // INFO_VAR(part_before[pbi].to_id)
              // INFO_VAR(part_before[pbi].from_id)
              // INFO_VAR(dn.node_id)

              new_path.plan.insert(
                new_path.plan.end(),
                original.plan.begin() + dn.node_index_inside_path + 1,
                original.plan.end());

              // INFO("new created:")
              // new_path.print();

              new_path.length = new_path.calc_path_length();
              new_path.from_id = new_path.plan.front()->id;
              new_path.to_id = new_path.plan.back()->id;
              if (!isPathCollisionFree(new_path).first) {
                INFO("connected path is not collision free");
                auto tst = isPathCollisionFree(new_path);
                INFO(tst.second.transpose())
                exit(1);
              }
              found_paths.push_back(new_path);
              // end creating new composed path
            }
          }
          // INFO("part before " << part_before.size())


          /*find different paths before the one deleted end*/

          /*find different paths after the one deleted begin*/
          std::unordered_map<int, path_node_split> new_starts;
          for (auto dn : deleted_nodes) {
            HeapNode<T>* new_start =
              found_paths[dn.second.path_id]
                .plan[dn.second.node_index_inside_path + 1];
            if (end_nodes_set.count(new_start->id) == 0) {
              path_node_split dn_start = dn.second;
              dn_start.node_id = new_start->id;
              dn_start.node_index_inside_path += 1;
              new_starts[new_start->id] = dn_start;
            }
          }
          std::vector<int> new_starts_vec;
          for (auto dn : new_starts) {
            new_starts_vec.push_back(dn.first);
          }
          std::cout << "new_starts_vec " << new_starts_vec << std::endl;
          std::vector<path_with_length<T>> part_after =
            findShortestBetween(new_starts_vec, end_nodes_vec, depth + 1);
          INFO_VAR(part_after.size());

          for (size_t pai = 0; pai < part_after.size(); ++pai) {
            if (part_after[pai].plan.size() > 0) {
              if (new_starts.count(part_after[pai].from_id) == 0) {
                INFO("path new_starts should contain path start")
                INFO_VAR(part_after[pai].from_id)
                exit(1);
              }

              path_node_split pns = new_starts[part_after[pai].from_id];
              path_with_length<T>& original = found_paths[pns.path_id];
              // INFO("original")
              // original.print();
              // INFO("new after part")
              // part_after[pai].print();
              path_with_length<T> new_path;
              new_path.plan.insert(
                new_path.plan.end(), original.plan.begin(),
                original.plan.begin() + (pns.node_index_inside_path));
              new_path.plan.insert(new_path.plan.end(),
                                   part_after[pai].plan.begin(),
                                   part_after[pai].plan.end());
              new_path.length = new_path.calc_path_length();
              new_path.from_id = new_path.plan.front()->id;
              new_path.to_id = new_path.plan.back()->id;
              // INFO("new created:")
              // new_path.print();
              if (!isPathCollisionFree(new_path).first) {
                INFO("connected path is not collision free");
                auto tst = isPathCollisionFree(new_path);
                INFO(tst.second.transpose())
                exit(1);
              }
              found_paths.push_back(new_path);
            }
          }


          // exit(1);
          /*find different paths after the one deleted end*/
        }
      }

      return found_paths;
      // exit(1);
    }
  }

  INFO("findShortestBetween end with depth " << depth)
  return found_paths;
}


template<class T>
std::vector<path_with_length<T>> TopologicalPRM<T>::findShortestPath() {
  // Dijkstra<T> dijkstra;
  std::vector<path_with_length<T>> shortest_plan =
    dijkstra.findPath(0, {1}, connection_nodes_);
  // INFO("shortest_plan size " << shortest_plan.size())
  // INFO_VAR(shortest_plan[0].length)
  // INFO_VAR(shortest_plan[0].plan.size())
  return shortest_plan;
}

template<class T>
bool TopologicalPRM<T>::isDeformablePath(std::vector<HeapNode<T>*> path1,
                                         const double length_tot1,
                                         std::vector<HeapNode<T>*> path2,
                                         const double length_tot2,
                                         const double num_collision_check) {
  return isDeformablePath(path1, length_tot1, path2, length_tot2,
                          num_collision_check, min_clearance_);
}

template<class T>
std::vector<path_with_length<T>> TopologicalPRM<T>::removeTooLongPaths(
  std::vector<path_with_length<T>> paths) {
  std::vector<path_with_length<T>> proned = paths;
  INFO("removeTooLongPaths begin")

  // find shortest length
  double min_length = DBL_MAX;
  for (auto p : proned) {
    if (p.length < min_length) {
      min_length = p.length;
    }
  }

  // remove the one that are longer than cutoof_distance_ratio_to_shortest_ *
  // min_length
  for (int i = proned.size() - 1; i >= 0; i--) {
    if (proned[i].length > cutoof_distance_ratio_to_shortest_ * min_length) {
      proned.erase(proned.begin() + i);
    }
  }
  INFO("remove " << (paths.size() - proned.size())
                 << " paths due to being too long")
  INFO("removeTooLongPaths end")
  return proned;
}

template<class T>
std::vector<path_with_length<T>>
TopologicalPRM<T>::removeWrongGateDirectionPaths(
  std::vector<path_with_length<T>> paths) {
  std::vector<path_with_length<T>> proned = paths;
  INFO("removeWrongGateDirectionPaths begin")
  for (int i = proned.size() - 1; i >= 0; i--) {
    // INFO("i " << i)
    const int plan_size = proned[i].plan.size();
    if (plan_size > 2) {
      double angle_to_end = 0.0;
      if (constraint_end_) {
        Vector<3> vec_to_end = proned[i].plan[plan_size - 1]->data -
                               proned[i].plan[plan_size - 2]->data;
        const Vector<3> end_gate_x = endq_ * Vector<3>::UnitX();
        angle_to_end = angleBetween(vec_to_end, end_gate_x);
        // INFO_VAR(end_gate_x.transpose())
      }

      double angle_from_start = 0.0;
      if (constraint_start_) {
        Vector<3> vec_from_start =
          proned[i].plan[1]->data - proned[i].plan[0]->data;
        const Vector<3> start_gate_x = startq_ * Vector<3>::UnitX();
        angle_from_start = angleBetween(vec_from_start, start_gate_x);
        // INFO_VAR(start_gate_x.transpose())
      }


      // INFO_VAR(angle_to_end)
      // INFO_VAR(angle_from_start)
      // INFO_VAR(angle_limit_start_end_)
      if (angle_to_end > angle_limit_start_end_ ||
          angle_from_start > angle_limit_start_end_) {
        // INFO("remove path " << i)
        proned.erase(proned.begin() + i);
        // INFO("removed " << i)
      }
    }
  }
  INFO("removeWrongGateDirectionPaths end")
  return proned;
}

template<class T>
std::vector<path_with_length<T>> TopologicalPRM<T>::removeEquivalentPaths(
  std::vector<path_with_length<T>> paths) {
  INFO_GREEN("removeEquivalentPaths begin with " << paths.size() << " paths")
  INFO_VAR(min_clearance_)
  INFO_VAR(collision_distance_check_)
  // INFO_VAR(map_->getResolution())
  std::vector<path_with_length<T>> paths_copy = paths;
  // std::vector<path_with_length<T>> proned;
  if (paths_copy.size() > 1) {
    // bool something_removed = true;
    // while (something_removed) {
    // something_removed = false;
    for (size_t i = 0; i < paths_copy.size(); i++) {
      int shortest_path_i = i;
      double shortest_length = paths_copy[i].length;
      std::vector<int> to_remove_indexes;
      for (size_t j = i + 1; j < paths_copy.size(); j++) {
        const double larger_length =
          std::max(paths_copy[i].length, paths_copy[j].length);
        const double num_check_collision =
          ceil(larger_length / collision_distance_check_) + 1;
        bool deformable = isDeformablePath(
          paths_copy[i].plan, paths_copy[i].length, paths_copy[j].plan,
          paths_copy[j].length, num_check_collision, collision_distance_check_);
        if (deformable) {
          // INFO("path " << i << " is deformable to " << j)
          // ith_unique = false;
          to_remove_indexes.push_back(j);
          if (paths_copy[j].length < shortest_length) {
            shortest_path_i = j;
            shortest_length = paths_copy[j].length;
          }
        } else {
          // INFO("path " << i << " not deformable to " << j)
        }
      }
      if (shortest_path_i != i) {
        paths_copy[i] = paths_copy[shortest_path_i];
      }
      for (int tri = to_remove_indexes.size() - 1; tri >= 0; tri--) {
        // INFO("removing " << to_remove_indexes[tri])
        paths_copy.erase(paths_copy.begin() + to_remove_indexes[tri]);
        // INFO("size " << to_remove_indexes[tri])
      }
      // INFO("purged")
      //}
    }
  }
  // INFO_GREEN("removeEquivalentPaths end")
  return paths_copy;
  /*
  if (paths_copy.size() >= 1) {
    // proned.push_back(paths_copy[0]);
    // paths_copy.erase(paths_copy.begin());
    INFO_VAR(paths_copy.size())
    // remove the paths with same visible homotopy
    for (size_t i = 0; i < proned.size(); i++) {
      INFO("beg forcycle")
      INFO_VAR(proned.size())
      INFO_VAR(paths_copy.size())
      // bool ith_unique = true;
      int shortest_idx_same = -1;
      double shortest_length = proned[i].length;
      std::vector<int> to_remove_indexes;

      for (size_t j = 0; j < paths_copy.size(); j++) {
        const double larger_length =
          std::max(shortest_length, paths_copy[j].length);
        const double num_check_collision =
          ceil(2 * larger_length / collision_distance_check_) + 1;

        // INFO_VAR(shortest_length / num_check_collision)
        // INFO_VAR(paths_copy[j].length / num_check_collision)

        bool deformable = isDeformablePath(
          proned[i].plan, proned[i].length, paths_copy[j].plan,
          paths_copy[j].length, num_check_collision, min_clearance_ * (0.6));
        if (deformable) {
          INFO("path " << j << " is deformable to proned " << i)
          // ith_unique = false;
          to_remove_indexes.push_back(j);
          if (paths_copy[j].length < shortest_length) {
            shortest_length = paths_copy[j].length;
            shortest_idx_same = j;
          }
        } else {
          INFO("path " << j << " not deformable to proned " << i)
        }
      }

      if (shortest_idx_same >= 0) {
        proned[i] = paths_copy[shortest_idx_same];
      }

      for (int tri = to_remove_indexes.size(); tri >= 0; tri--) {
        paths_copy.erase(paths_copy.begin() + to_remove_indexes[tri]);
      }

      if (paths_copy.size() > 0) {
        proned.push_back(paths_copy[0]);
        paths_copy.erase(paths_copy.begin());
      } else {
        break;
      }
    }
  }

  return proned;
  */
}

/*
shorten the path by maximizing the collision-free distance of straight lines
starting from the start and adding points that are obtained from the collision
place and pushed away from obstacles
*/
template<class T>
path_with_length<T> TopologicalPRM<T>::shorten_path(path_with_length<T> path,
                                                    bool forward) {
  // INFO_GREEN("shorten_path begin with forward " << forward)
  // INFO("path from " << path.plan.front()->data.transpose() << " and "
  //                   << path.plan.back()->data.transpose())
  path_with_length<T> shortened;
  shortened.length = 0;

  const double num_samples = ceil(path.length / collision_distance_check_) + 1;
  // INFO("path.path size " << path.path.size())
  // INFO("num_samples " << num_samples);
  // INFO("collision_distance_check_ " << collision_distance_check_);
  std::vector<T> sampled = samplePath(path.plan, path.length, num_samples);
  // INFO("num samples " << sampled.size());

  // int dir = 1;
  int start_iter = 1;
  if (forward) {
    shortened.plan.push_back(path.plan.front());
  } else {
    shortened.plan.push_back(path.plan.back());
    std::reverse(sampled.begin(), sampled.end());
    // dir = -1;
    // start_iter = sampled.size() - 2;
  }


  // for (size_t i = sampled.size()-2; i < sampled.size(); i += dir) {
  for (size_t i = 1; i < sampled.size(); i++) {
    HeapNode<T>* start_node = shortened.plan.back();
    // INFO("from " << start_node->data.transpose() << " to "
    //              << sampled[i].transpose())


    std::pair<bool, Vector<3>> is_free =
      isSimplePathFreeBetweenNodes(start_node->data, sampled[i]);

    if (!is_free.first) {
      // INFO("not free")
      const Vector<3> collision_place = is_free.second;
      const auto [gradient_in_place, voxel_center] =
        map_->gradientInVoxelCenter(collision_place);
      // INFO("collision in pos " << collision_place.transpose())
      // INFO("gradient_in_place " << gradient_in_place.transpose())
      // go perpendicular to line shortened.path.back()->data, sampled[i]
      // and towards shortest distance to line shortened.path.back()->data,
      // sampled[i-1]
      // const Vector<3> old_point_vector = sampled[i - 1] - start_node->data;


      const Vector<3> new_point_vector = sampled[i] - start_node->data;
      // INFO_VAR(new_point_vector.transpose())
      const Vector<3> normal_gradient_new_point =
        gradient_in_place.cross(new_point_vector);
      // INFO_VAR(normal_gradient_new_point.transpose())

      // const Vector<3> normal_new_old_start =
      //   old_point_vector.cross(new_point_vector);
      Vector<3> vec_from_collision =
        new_point_vector.cross(normal_gradient_new_point);
      vec_from_collision.normalize();
      // INFO_VAR(vec_from_collision.transpose())

      // const double clearance_collision =
      // map_->getClearence(collision_place); INFO("clearance_collision " <<
      // clearance_collision)
      const double ds = collision_distance_check_;

      // double count_check =
      // std::min(collision_distance_check_ / clearance_collision, 1.0);
      // INFO("count_check " << count_check)
      bool added_after_collision = false;
      for (double tci = min_clearance_; tci <= min_clearance_ * 4; tci += ds) {
        const Vector<3> new_point = voxel_center + vec_from_collision * tci;
        // INFO("test collision in new place " << new_point.transpose())
        if (!isInCollision(new_point)) {
          HeapNode<T>* new_node = new HeapNode<T>(new_point);
          // HeapNode<T>* back_old = shortened.path.back();
          const double dist_new_node = distance(start_node->data, new_point);
          start_node->visibility_node_ids[new_node] = dist_new_node;
          new_node->visibility_node_ids[start_node] = dist_new_node;
          shortened.length += dist_new_node;
          shortened.plan.push_back(new_node);
          added_after_collision = true;
          break;
        } else {
          INFO("in collision wiht calue" << map_->getClearence(new_point))
        }
      }

      if (!added_after_collision) {
        ERROR_RED("no point added to shortened path after collision");
        exit(1);
      }
    }
  }

  // INFO("shortened from " << path.length << " to " << shortened.length)
  // INFO("shortened.path.size() " << shortened.path.size())
  // INFO("shorten_path end")

  HeapNode<T>* last_node_original;
  if (forward) {
    last_node_original = path.plan.back();
  } else {
    last_node_original = path.plan.front();
  }

  HeapNode<T>* back_old = shortened.plan.back();
  const double dist_new_node =
    distance(back_old->data, last_node_original->data);
  back_old->visibility_node_ids[last_node_original] = dist_new_node;
  last_node_original->visibility_node_ids[back_old] = dist_new_node;
  shortened.length += dist_new_node;
  shortened.plan.push_back(last_node_original);

  if (!forward) {
    reverse(shortened.plan.begin(), shortened.plan.end());
  }


  // debuging begin
  // check distance
  double calc_distance = 0;
  for (size_t i = 1; i < shortened.plan.size(); i++) {
    calc_distance +=
      distance(shortened.plan[i - 1]->data, shortened.plan[i]->data);
  }
  if (fabs(calc_distance - shortened.length) > PRECISION) {
    INFO_RED("shortened length does not equal")
    INFO_VAR(shortened.length)
    INFO_VAR(calc_distance)
    exit(1);
  }
  // INFO("calc_distance " << calc_distance)
  // debuging end


  return shortened;
}


template<class T>
std::vector<std::vector<path_with_length<Vector<3>>>>
TopologicalPRM<T>::find_geometrical_paths(
  const YAML::Node& planner_config, std::shared_ptr<BaseMap> map,
  std::vector<Vector<3>>& gates_with_start_end_poses,
  std::vector<double> gates_orientations, std::string output_folder) {
  // gates_with_start_end_poses.resize(2);
  // gates_with_start_end_poses =
  //   std::vector<Vector<3>>(gates_with_start_end_poses.begin() + 1,
  //                          gates_with_start_end_poses.begin() + 3);
  std::vector<std::shared_ptr<TopologicalPRM<Vector<3>>>> topological_prms;

  std::vector<std::vector<path_with_length<Vector<3>>>> paths_between_gates;
  INFO_VAR(gates_with_start_end_poses.size())

  int num_samples_between_gate =
    loadParam<double>(planner_config, "num_samples_between_gate");

  paths_between_gates.resize(gates_with_start_end_poses.size() - 1);
  topological_prms.resize(gates_with_start_end_poses.size() - 1);

  for (size_t i = 0; i < topological_prms.size(); i++) {
    // for (size_t i = 0; i < 3; i++) {
    // create the prms first
    INFO("gates_with_start_end_poses[i] " << gates_with_start_end_poses[i])
    INFO("gates_with_start_end_poses[i+1] "
         << gates_with_start_end_poses[i + 1]);


    double first_gate_yaw_deg = gates_orientations[i];
    double second_gate_yaw_deg = gates_orientations[i + 1];
    if (i == 0) {
      first_gate_yaw_deg = NAN;
    }
    if (i == gates_with_start_end_poses.size() - 1) {
      second_gate_yaw_deg = NAN;
    }

    INFO_VAR(first_gate_yaw_deg)
    INFO_VAR(second_gate_yaw_deg)

    topological_prms[i] = std::make_shared<TopologicalPRM<Vector<3>>>(
      planner_config, map, output_folder, gates_with_start_end_poses[i],
      gates_with_start_end_poses[i + 1], first_gate_yaw_deg,
      second_gate_yaw_deg);
    INFO("map min pos " << map->getMinPos())
    INFO("map max pos " << map->getMaxPos())
    topological_prms[i]->setBorders(map->getMinPos(), map->getMaxPos());

    // exit(1);
    // for (size_t si = 0; si < 300; si++) {
    //   topological_prms[i]->samplePoint();
    // }
    topological_prms[i]->sampleMultiple(num_samples_between_gate);

    // std::stringstream ss;
    // ss << "roadmap_" << num_samples_between_gate << ".csv";
    // topological_prms[i]->saveRoadmap(ss.str());
    // exit(1);

    std::vector<path_with_length<Vector<3>>> shortest_test =
      topological_prms[i]->findShortestPath();
    while (shortest_test.size() == 0 || shortest_test[0].plan.size() == 0) {
      INFO("no path found between gates " << i << " and " << (i + 1))
      // exit(1);
      if (topological_prms[i]->getEllipseRatioMajorAxis() < 3.0) {
        topological_prms[i]->setEllipseRatioMajorAxis(
          topological_prms[i]->getEllipseRatioMajorAxis() * 1.5);
      }

      num_samples_between_gate = num_samples_between_gate * 1.5;
      topological_prms[i]->sampleMultiple(num_samples_between_gate);
      shortest_test = topological_prms[i]->findShortestPath();
    }

    // std::stringstream ss;
    // ss << output_folder_ << "roadmap_all" << i << ".csv";
    // topological_prms[i]->saveRoadmap(ss.str());


    // std::vector<path_with_length<Vector<3>>> diff_paths =
    //   topological_prms[i]->findDistinctPaths();1731 [0x7fc96404e780] INFO
    //   main null - connected path is not collision free

    std::vector<path_with_length<Vector<3>>> diff_paths =
      topological_prms[i]->findDistinctPathsBlockingSpheres();

    // std::vector<path_with_length<Vector<3>>> diff_paths =
    //   topological_prms[i]->findShortestPath();

    INFO_GREEN("diff_paths size " << diff_paths.size())
    // exit(1);
    // INFO("soring")
    // std::sort(diff_paths.begin(), diff_paths.end(),
    //           comparator_path_with_length<Vector<3>>);

    // INFO("keeping only 5 shortest paths")
    diff_paths.resize(std::min(diff_paths.size(), 800ul));


    INFO("shortening " << diff_paths.size() << " paths")
    std::vector<path_with_length<Vector<3>>> shortened_paths = diff_paths;
    // shortened_paths.resize(diff_paths.size());
    for (size_t pi = 0; pi < diff_paths.size(); pi++) {
      // INFO("shortening path bef " << pi)
      // INFO("path length " << diff_paths[pi].length << " with num nodes "
      //                     << diff_paths[pi].path.size())
      shortened_paths[pi] =
        TopologicalPRM<Vector<3>>::shorten_path(shortened_paths[pi]);
      shortened_paths[pi] =
        TopologicalPRM<Vector<3>>::shorten_path(shortened_paths[pi], false);
      // shortened_paths[pi] =
      //   TopologicalPRM<Vector<3>>::shorten_path(diff_paths[pi]);
      // INFO("shortening path aft " << pi)
    }
    INFO("shortened")

    // for (size_t pi = 0; pi < shortened_paths.size(); pi++) {
    //   std::stringstream path_ss;
    //   // INFO(shortened_paths[pi].length)
    //   path_ss << output_folder_ << "roadmap_shortened_path" << i << "_" << pi
    //           << ".csv";
    //   TopologicalPRM<Vector<3>>::savePath(path_ss.str(),
    //                                       shortened_paths[pi].plan);
    // }

    INFO("before removing wrong direction start end size:"
         << shortened_paths.size())
    shortened_paths =
      topological_prms[i]->removeWrongGateDirectionPaths(shortened_paths);
    INFO("after removing wrong direction start end size:"
         << shortened_paths.size())

    // for (size_t pi = 0; pi < shortened_paths.size(); pi++) {
    //   std::stringstream path_ss;
    //   // INFO(shortened_paths[pi].length)
    //   path_ss << output_folder_ << "roadmap_shortened_correct_dir_path" << i
    //           << "_" << pi << ".csv";
    //   TopologicalPRM<Vector<3>>::savePath(path_ss.str(),
    //                                       shortened_paths[pi].plan);
    // }


    shortened_paths =
      TopologicalPRM<Vector<3>>::removeEquivalentPaths(shortened_paths);


    INFO("after remove equivalent 1 size is " << shortened_paths.size())
    shortened_paths =
      TopologicalPRM<Vector<3>>::removeEquivalentPaths(shortened_paths);
    INFO("after remove equivalent 2 size is " << shortened_paths.size())

    shortened_paths = topological_prms[i]->removeTooLongPaths(shortened_paths);

    std::sort(shortened_paths.begin(), shortened_paths.end(),
              comparator_path_with_length<Vector<3>>);
    shortened_paths.resize(std::min(shortened_paths.size(), 5ul));

    paths_between_gates[i] = shortened_paths;

    // for (size_t pi = 0; pi < diff_paths.size(); pi++) {
    // std::stringstream path_ss;
    // INFO(diff_paths[pi].length)
    // path_ss << output_folder_ << "roadmap_path" << i << "_" << pi <<
    // ".csv"; TopologicalPRM<Vector<3>>::savePath(path_ss.str(),
    // diff_paths[pi].plan);

    // std::stringstream path_ss_samples;
    // path_ss_samples << output_folder_ << "roadmap_samples_path" << i << "_"
    //                 << pi << ".csv";
    // const double num_samples =
    //   ceil(diff_paths[pi].length / collision_distance_check_) + 1;
    // std::vector<Vector<3>> samples = TopologicalPRM<Vector<3>>::samplePath(
    //   diff_paths[pi].plan, diff_paths[pi].length, num_samples);
    // TopologicalPRM<Vector<3>>::savePathSamples(path_ss_samples.str(),
    //                                            samples);
    // }

    for (size_t pi = 0; pi < shortened_paths.size(); pi++) {
      std::stringstream path_ss;
      INFO("shortened length " << shortened_paths[pi].length)
      path_ss << output_folder_ << "roadmap_shortened_unique_path" << i << "_"
              << pi << ".csv";
      INFO("juchuuuuu")
      TopologicalPRM<Vector<3>>::savePath(path_ss.str(),
                                          shortened_paths[pi].plan);
      INFO("saved");
    }
    INFO("ended loop")
  }
  INFO("samples created");
  // exit(1);
  INFO("return")
  return paths_between_gates;
}