/*
 * Dijkstra.h
 *
 *  Created on: 2. 2. 2015
 *      Author: Robert Pěnička
 */

#pragma once

#include <algorithm>
#include <vector>

#include "common.hpp"
#include "float.h"
#include "heap.hpp"

#define DIJKSTRA_INF DBL_MAX

template<typename T>
struct path_with_length {
  std::vector<HeapNode<T>*> plan;
  double length;
  int from_id;
  int to_id;
  double calc_path_length() const {
    double length_together = 0;
    // std::cout << new_path.plan[0]->id << " ";
    for (size_t nppi = 1; nppi < plan.size(); nppi++) {
      // std::cout << plan[nppi]->id << " ";
      length_together += distance(plan[nppi]->data, plan[nppi - 1]->data);
      //(new_path.plan[nppi]->data - new_path.plan[nppi - 1]->data).norm();
    }
    return length_together;
  };
  void print() {
    std::cout << "plan:" << std::endl;
    for (size_t nppi = 0; nppi < plan.size(); nppi++) {
      std::cout << plan[nppi]->id << " ";
    }
    std::cout << std::endl;
  };
};


// template<typename T>
// struct path_with_length {
//   std::vector<HeapNode<T>*> path;
//   double path_length;
// };

template<class T>
class Dijkstra {
 public:
  Dijkstra();
  virtual ~Dijkstra();
  // plan_with_length findPath(int start_index, int goal_index,
  // std::vector<HeapNode*> & connectedPoints);
  std::vector<path_with_length<T>> findPath(
    int start_index, std::vector<int> goal_indexes,
    std::vector<HeapNode<T>*>& connectedPoints);

 private:
  HeapNode<T>* expandBestNode();
  Heap<HeapNode<T>*>* heap;
};

template<class T>
std::ostream& operator<<(std::ostream& o, const HeapNode<T>& p);


template<typename T>
Dijkstra<T>::Dijkstra() {
  heap = NULL;
}

template<typename T>
Dijkstra<T>::~Dijkstra() {
  if (heap != NULL) {
    delete heap;
  }
}

template<typename T>
std::vector<path_with_length<T>> Dijkstra<T>::findPath(
  int start_index, std::vector<int> goal_indexes,
  std::vector<HeapNode<T>*>& visibility_graph) {
  // INFO("begin Dijkstra::findPath()");
  /*********** init dijkstra *************/


  for (int var = 0; var < visibility_graph.size(); ++var) {
    visibility_graph[var]->distance_from_start = DIJKSTRA_INF;
    visibility_graph[var]->previous_point = NULL;
  }
  visibility_graph[start_index]->distance_from_start = 0;
  visibility_graph[start_index]->previous_point = visibility_graph[start_index];


  // need to create heap after setting points
  heap = new Heap<HeapNode<T>*>(visibility_graph);
  bool stopped = false;
  HeapNode<T>* actualBestNode;
  std::set<int> goal_indexes_non_visited(goal_indexes.begin(),
                                         goal_indexes.end());
  int numLooped = 0;
  // int loopCounter = 0;
  // INFO("loop points");
  while (!stopped) {
    actualBestNode = expandBestNode();

    if (actualBestNode == NULL ||
        actualBestNode->distance_from_start == DIJKSTRA_INF) {
      stopped = true;
      // INFO("DIJKSTRA not found")
    }

    if (!stopped && actualBestNode->city_node) {
      // can be one of goal
      auto search = goal_indexes_non_visited.find(actualBestNode->id);
      if (search != goal_indexes_non_visited.end()) {
        goal_indexes_non_visited.erase(search);
      }

      if (goal_indexes_non_visited.empty()) {
        stopped = true;
      }
    }

    numLooped++;
    /*
    loopCounter++;
    if ( loopCounter >= 1000 ) {
      loopCounter = 0;
      //INFO("looped " << numLooped << " points");
    }
    */
  }

  delete heap;
  heap = NULL;

  std::vector<path_with_length<T>> plans_w_length;

  for (int var = 0; var < goal_indexes.size(); ++var) {
    std::vector<HeapNode<T>*> plan;
    HeapNode<T>* actualPoint = visibility_graph[goal_indexes[var]];
    // INFO("createPlan from goal " << actualPoint<< " node id
    // "<<actualPoint->node_id <<" is city "<<actualPoint->city_node);
    if (actualPoint->previous_point != NULL) {
      plan.push_back(actualPoint);
      while (actualPoint != visibility_graph[start_index]) {
        // WINFO(actualPoint << " get previous point ");
        actualPoint = actualPoint->previous_point;
        // INFO("add point " << actualPoint<< " node id "<<actualPoint->node_id
        // <<" is city "<<actualPoint->city_node);
        plan.push_back(actualPoint);
      }
      std::reverse(plan.begin(), plan.end());
      path_with_length<T> plan_w_length;
      plan_w_length.length =
        visibility_graph[goal_indexes[var]]->distance_from_start;
      plan_w_length.plan = plan;
      plan_w_length.from_id = start_index;
      plan_w_length.to_id = goal_indexes[var];
      plans_w_length.push_back(plan_w_length);
    } else {
      path_with_length<T> plan_w_length;
      plan_w_length.length = DIJKSTRA_INF;
      plan_w_length.plan = plan;
      plan_w_length.from_id = start_index;
      plan_w_length.to_id = goal_indexes[var];
      plans_w_length.push_back(plan_w_length);
    }
  }
  // INFO("end Dijkstra::findPath()");

  return plans_w_length;
}

template<typename T>
HeapNode<T>* Dijkstra<T>::expandBestNode() {
  HeapNode<T>* expandingNode = heap->pop();
  if (expandingNode != NULL) {
    for (auto connectedNode : expandingNode->visibility_node_ids) {
      double calculated_new_distance =
        expandingNode->distance_from_start + connectedNode.second;

      if (calculated_new_distance < connectedNode.first->distance_from_start) {
        // test point if better distance found
        connectedNode.first->previous_point = expandingNode;
        heap->updateCost(connectedNode.first, calculated_new_distance);
      }
    }
  }
  return expandingNode;
}
