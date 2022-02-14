/*
 * PRM.h
 *
 *  Created on: 1. 2. 2015
 *      Author: Robert Pěnička
 */

#pragma once

#include <algorithm>
#include <flann/flann.hpp>
#include <limits>
#include <memory>
#include <vector>

#include "common.hpp"
#include "dijkstra.hpp"
#include "esdf_map.hpp"
#include "tree_node.hpp"

#define OUTPUT_AFTER_ADDED_NUM_PRM 100

template<class T>
class PRM {
 public:
  PRM(const YAML::Node& planner_config, std::shared_ptr<ESDFMap> map);
  void createInitialGraph(std::vector<HeapNode<T>*>& cities_nodes_,
                          int initial_size);

  void addUniformPoints(int num_points_to_add);

  // bool add_point(Point3DOriented point);
  void calculateAddedPoints(bool add_points = true);
  std::vector<HeapNode<T>*>& get_points();

  path_with_length<T> plan(int start_index, int goal_index);
  std::vector<path_with_length<T>> plan(int start_index,
                                        std::vector<int> goal_indexes);
  void setBorders(Vector<3> min_position, Vector<3> max_position);

 private:
  // FILL RANDUM STATE
  void fillRandomState(HeapNode<T>* positionToFill);
  void fillPoint(HeapNode<T>* newNode, T point);

  std::shared_ptr<ESDFMap> map;

  // TEST POINTS BETWEEN STATES
  bool isPathFreeBetweenNodes(HeapNode<T>* actual, HeapNode<T>* neigbour);
  // MPNN::ANNpoint createANNpoint(MapPoint<PLANNER_STATE>* node);
  // MPNN::ANNpoint fillANNpoint(MPNN::ANNpoint aNNpoint,
  // MapPoint<PLANNER_STATE>* treeNode); FILL OR CREATE ANN POINT MPNN::ANNpoint
  // fillANNpoint(HeapNode<T> * heepNode, MPNN::ANNpoint aNNpoint = NULL);

  // test collision
  bool isInCollision(T object_position);
  bool isInCollision(HeapNode<T>* node);

  flann::Index<flann::L2<float>>* flann_index;

  Dijkstra<T> dijskra_;

  std::vector<HeapNode<T>*> cities_nodes_;

  std::vector<HeapNode<T>*> generated_free_positions_;
  std::shared_ptr<ESDFMap> map_;
  double collision_distance_check_;
  double min_clearance_;

  bool continuous_point_adding_;
  int continuous_point_adding_start_id_;

  Vector<3> min_position_, max_position_, position_range_;
};

template<class T>
PRM<T>::PRM(const YAML::Node& planner_config, std::shared_ptr<ESDFMap> map) {
  // TODO Auto-generated constructor stub
  map_ = map;
  continuous_point_adding_ = false;
  min_clearance_ = loadParam<double>(planner_config, "min_clearance");
  this->collision_distance_check_ =
    loadParam<double>(planner_config, "collision_distance_check");
  continuous_point_adding_start_id_ = 0;

  if (collision_distance_check_ == 0) {
    ERROR(
      "you need to specify collision_distance_check for sampling-based motion "
      "planning");
    exit(1);
  }
}

template<class T>
void PRM<T>::createInitialGraph(std::vector<HeapNode<T>*>& cities_nodes_,
                                int initial_size) {
  INFO("generate graph PRM");

  // generated_number_of_neighbors = NUMBER_OF_NEAREST_NEIGBOURS + 1;
  this->cities_nodes_ = cities_nodes_;

  int generateIndex = 0;
  int addedCounter = 0;
  int collisionCounter = 0;
  int collisionCounterTot = 0;
  bool collisionDetected = true;

  // add also cities
  generated_free_positions_.reserve(initial_size + cities_nodes_.size());
  INFO("reserved");

  // try to connect goal
  int citiesNumConnectionsAdded = 0;
  INFO("we have " << cities_nodes_.size() << "city nodes")
  for (int cityIndex = 0; cityIndex < cities_nodes_.size(); ++cityIndex) {
    // MPNN::ANNpoint newPoint = fillANNpoint(cities_nodes[cityIndex]);
    INFO("city: " << *cities_nodes_[cityIndex]);
    if (!isInCollision(cities_nodes_[cityIndex])) {
      // kdTree->AddPoint(newPoint, cities_nodes[cityIndex]);
      generated_free_positions_.push_back(cities_nodes_[cityIndex]);
    } else {
      ERROR("collision detected in city with index " << cityIndex
                                                     << "!!!! exiting....");
      exit(1);
    }
  }
  INFO("added " << generated_free_positions_.size() << " cities positions");


  double next_initial_sampling_info = 0.0;
  // generate NUM_GENERATE_POSITIONS_AT_ONCE points
  for (generateIndex = 0; generateIndex < initial_size; ++generateIndex) {
    collisionDetected = true;
    HeapNode<T>* newNode = new HeapNode<T>();
    // INFO("generated "<<generateIndex);
    newNode->node_id =
      generated_free_positions_.size();      // start and goal are 0 and 1
    newNode->cluster_id = newNode->node_id;  // start and goal are 0 and 1
    newNode->city_node = false;
    while (collisionDetected) {
      fillRandomState(newNode);
      if (!isInCollision(newNode)) {
        generated_free_positions_.push_back(newNode);
        collisionDetected = false;
        addedCounter++;
        // output every OUTPUT_AFTER_ADDED_NUM_PRM generated free positions
        if (((double)addedCounter) / ((double)initial_size) >=
            next_initial_sampling_info) {
          next_initial_sampling_info += 0.1;
          // INFO(	"collision states
          // "<<(100.0*((double)collisionCounter)/((double)
          // (collisionCounter+addedCounter)))<<"%");
          INFO("added " << addedCounter << " configurations out of "
                        << initial_size);
        }
      } else {
        // INFO("collision detected");
        collisionDetected = true;
        collisionCounter++;
        collisionCounterTot++;
      }
    }
  }

  INFO("testing flann");

  flann::Matrix<float> new_points_matrix(
    new float[generated_free_positions_.size() * 3],
    generated_free_positions_.size(), 3);
  for (int var = 0; var < generated_free_positions_.size(); ++var) {
    new_points_matrix[var][0] = generated_free_positions_[var]->data.x;
    new_points_matrix[var][1] = generated_free_positions_[var]->data.y;
    new_points_matrix[var][2] = generated_free_positions_[var]->data.z;
  }
  flann_index = new flann::Index<flann::L2<float>>(new_points_matrix,
                                                   flann::KDTreeIndexParams(4));
  flann_index->buildIndex();


  // int nn = generated_number_of_neighbors;

  // float sd = 2; //configuration space dimension
  // float num_points = generatedFreePositions.size();


  INFO("generated " << initial_size << " random positions");
  INFO("collisionCounterTot " << collisionCounterTot << " positions");
  continuous_point_adding_start_id_ = 0;
  this->continuous_point_adding_ = true;
  calculateAddedPoints(false);
  INFO("end create_initial_graph");
}

template<class T>
void PRM<T>::addUniformPoints(int num_points_to_add) {
  if (!this->continuous_point_adding_) {
    this->continuous_point_adding_ = true;
    this->continuous_point_adding_start_id_ = generated_free_positions_.size();
  }
  int generateIndex = 0;
  int addedCounter = 0;
  int collisionCounter = 0;
  int collisionCounterTot = 0;
  bool collisionDetected = true;
  for (generateIndex = 0; generateIndex < num_points_to_add; ++generateIndex) {
    collisionDetected = true;
    HeapNode<T>* newNode = new HeapNode<T>();
    // INFO("generated "<<generateIndex);
    newNode->node_id =
      generated_free_positions_.size();      // start and goal are 0 and 1
    newNode->cluster_id = newNode->node_id;  // start and goal are 0 and 1
    newNode->city_node = false;
    while (collisionDetected) {
      fillRandomState(newNode);
      if (!isInCollision(newNode)) {
        generated_free_positions_.push_back(newNode);
        collisionDetected = false;
        addedCounter++;

      } else {
        // INFO("collision detected");
        collisionDetected = true;
        collisionCounter++;
        collisionCounterTot++;
      }
    }
  }
}

/*
template<class T>
bool PRM<T>::add_point(Point3DOriented point) {
  if (!this->continuous_point_adding) {
    this->continuous_point_adding = true;
    this->continuous_point_adding_start_id = generatedFreePositions.size();
  }
  bool point_added = false;
  HeapNode<T>* newNode = new HeapNode<T>();
  newNode->node_id =
    generatedFreePositions.size();  // start and goal are 0 and 1
  newNode->cluster_id = generatedFreePositions.size();
  newNode->city_node = false;
  fillPoint(newNode, point);

  // INFO("test collision");
  if (!testCollision(this->obstacles, this->robot, newNode)) {
    // MPNN::ANNpoint newPoint = fillANNpoint(newNode);

    // kdTree->AddPoint(newPoint, newNode);
    generatedFreePositions.push_back(newNode);
    // INFO("new point created, points size "<<generatedFreePositions.size()<<"
    // flann_index.size() " <<flann_index->size()<<"
    // continuous_point_adding_start_id "<<continuous_point_adding_start_id);
    point_added = true;
  } else {
    delete newNode;
  }
  return point_added;
}
*/

template<class T>
void PRM<T>::setBorders(Vector<3> min_position, Vector<3> max_position) {
  min_position_ = min_position;
  max_position_ = max_position;
  position_range_ = max_position - min_position;
}

template<class T>
void PRM<T>::calculateAddedPoints(bool add_points) {
  if (continuous_point_adding_) {
    // INFO("calculate_added_points begin");

    // INFO("new_points bef");
    std::vector<HeapNode<T>*> new_points(
      generated_free_positions_.begin() + continuous_point_adding_start_id_,
      generated_free_positions_.begin() + generated_free_positions_.size());
    // INFO("new_points af");

    // connect new points to the map
    flann::Matrix<float> new_points_matrix;
    // INFO("want to add " <<new_points.size() <<" points to flann index");

    new_points_matrix = flann::Matrix<float>(new float[new_points.size() * 3],
                                             new_points.size(), 3);
    for (int var = 0; var < new_points.size(); ++var) {
      new_points_matrix[var][0] = new_points[var]->data.x;
      new_points_matrix[var][1] = new_points[var]->data.y;
      new_points_matrix[var][2] = new_points[var]->data.z;
      // INFO(*new_points[var]);
    }
    if (add_points) {
      flann_index->addPoints(new_points_matrix,
                             2.0);  // do not add points when initializing
    }

    // INFO("added points");

    float sd = 3;  // configuration space dimension
    float num_points = generated_free_positions_.size();

    int k = M_E * (1 + 1 / sd) * log10(num_points);
    // k += num_headings;
    int k_search = k;

    // INFO_VAR(k);
    // INFO_VAR(k_search);
    std::vector<std::vector<int>> indices;
    std::vector<std::vector<int>> indices_rew;
    std::vector<std::vector<float>> dists;
    // flann::Matrix<int> indices(new int[new_points_matrix.rows * k_search],
    // new_points_matrix.rows, k_search); flann::Matrix<float> dists(new
    // float[new_points_matrix.rows * k_search], new_points_matrix.rows,
    // k_search);

    flann_index->knnSearch(new_points_matrix, indices, dists, k_search,
                           flann::SearchParams(128));

    indices_rew = indices;
    int numConnectionsAdded = 0;
    int numConnectionsAlreadyAdded = 0;
    int addedCounter = 0;
    int max_nn_used = 0;
    // INFO("add connections begin with k="<<k);
    // connect all points to generated_number_of_neighbors positions
    for (int generateIndex = continuous_point_adding_start_id_;
         generateIndex < generated_free_positions_.size(); ++generateIndex) {
      HeapNode<T>* actual = generated_free_positions_[generateIndex];

      int connection_per_target = 0;

      for (int neigbourIndex = 0; neigbourIndex < k; ++neigbourIndex) {
        int nnindex = indices[generateIndex - continuous_point_adding_start_id_]
                             [neigbourIndex];
        if (generated_free_positions_[nnindex]->cluster_id ==
            actual->cluster_id) {
          // INFO("skip same id gi "<<generateIndex<<" ni "<<neigbourIndex<<"
          // for cluster id "<< actual->cluster_id)
          continue;
        }

        HeapNode<T>* neigbour = generated_free_positions_[nnindex];

        bool freePath_act_neigh = isPathFreeBetweenNodes(actual, neigbour);
        if (freePath_act_neigh) {
          double distance_act_neigh = actual->data.distance(neigbour->data);
          // INFO("distance_act_neigh "<<distance_act_neigh<<"
          // dists[generateIndex -
          // continuous_point_adding_start_id][neigbourIndex]
          // "<<sqrt(dists[generateIndex -
          // continuous_point_adding_start_id][neigbourIndex]))

          auto connectionAdded1 =
            actual->visibility_node_ids.insert(std::pair<HeapNode<T>*, double>(
              neigbour, distance_act_neigh));  // addConnection(neigbour,
                                               // bestDist[neigbourIndex]);

          if (connectionAdded1.second) {
            numConnectionsAdded++;
            addedCounter++;
            connection_per_target += 1;
          } else {
            numConnectionsAlreadyAdded++;
          }
        }
      }

      for (int neigbourIndex = 0; neigbourIndex < k; ++neigbourIndex) {
        int nnindex =
          indices_rew[generateIndex - continuous_point_adding_start_id_]
                     [neigbourIndex];
        if (generated_free_positions_[nnindex]->cluster_id ==
            actual->cluster_id) {
          // INFO("skip same id gi "<<generateIndex<<" ni "<<neigbourIndex<<"
          // for cluster id "<< actual->cluster_id)
          continue;
        }

        HeapNode<T>* neigbour = generated_free_positions_[nnindex];
        bool freePath_neigh_act = isPathFreeBetweenNodes(neigbour, actual);
        if (freePath_neigh_act) {
          double distance_neigh_act = neigbour->data.distance(actual->data);
          auto connectionAdded2 = neigbour->visibility_node_ids.insert(
            std::pair<HeapNode<T>*, double>(actual, distance_neigh_act));
          if (connectionAdded2.second) {
            numConnectionsAdded++;
            addedCounter++;
            connection_per_target += 1;
          } else {
            numConnectionsAlreadyAdded++;
          }
        }
      }
      // INFO("snode "<<actual->node_id<<" gen index "<<generateIndex<<" c added
      // "<<connection_per_target<< " k is "<<k)
    }

    // INFO_RED("max_nn_used "<<max_nn_used);
    this->continuous_point_adding_start_id_ = generated_free_positions_.size();
    this->continuous_point_adding_ = false;
    // INFO("calculate_added_points end");
  } else {
    INFO("no new points");
  }
}

template<class T>
path_with_length<T> PRM<T>::plan(int start_index, int goal_index) {
  // INFO("begin PRM::plan()");

  // INFO("dijskra.findPath ");
  std::vector<int> goal_indexes;
  goal_indexes.push_back(goal_index);
  std::vector<path_with_length<T>> found_paths =
    dijskra_.findPath(start_index, goal_indexes, generated_free_positions_);

  // INFO("plan length "<<found_paths[0].length);
  // INFO("end PRM::plan()");

  return found_paths[0];
}

template<class T>
std::vector<path_with_length<T>> PRM<T>::plan(int start_index,
                                              std::vector<int> goal_indexes) {
  // INFO("begin PRM::plan()");

  std::vector<path_with_length<T>> found_paths =
    dijskra_.findPath(start_index, goal_indexes, generated_free_positions_);

  // INFO("end PRM::plan()");
  return found_paths;
}

template<class T>
std::vector<HeapNode<T>*>& PRM<T>::get_points() {
  return this->generated_free_positions_;
}
