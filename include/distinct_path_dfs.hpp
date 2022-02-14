#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <unordered_set>
#include <vector>

#include "common.hpp"
#include "dijkstra.hpp"
#include "esdf_map.hpp"
#include "tree_node.hpp"

template<typename T>
struct path_with_visited {
  std::unordered_set<HeapNode<T>*> visited_nodes;
  std::vector<HeapNode<T>*> path;
  double path_length;
};

template<typename T>
static constexpr auto comparator_path_with_length =
  [](const path_with_length<T>& a, const path_with_length<T>& b) -> bool {
  return a.length < b.length;
};

template<class T>
class DistinctPathDFS {
 public:
  DistinctPathDFS(std::vector<HeapNode<T>*> nodes, HeapNode<T>* start,
                  HeapNode<T>* end);
  std::vector<path_with_length<T>> findPaths();

 private:
  void expand(HeapNode<T>* node, path_with_visited<T> path,
              const int& max_steps);
  std::vector<HeapNode<T>*> nodes_;
  HeapNode<T>* start_;
  HeapNode<T>* end_;
  std::vector<path_with_visited<T>> distinct_paths;
};


template<class T>
DistinctPathDFS<T>::DistinctPathDFS(std::vector<HeapNode<T>*> nodes,
                                    HeapNode<T>* start, HeapNode<T>* end) {
  nodes_ = nodes;
  start_ = start;
  end_ = end;
  // INFO("DistinctPathDFS created")
}

template<class T>
std::vector<path_with_length<T>> DistinctPathDFS<T>::findPaths() {
  INFO("findPaths dfs begin")
  distinct_paths.clear();
  // for (int var = 0; var < nodes_.size(); ++var) {
  //   nodes_[var]->visited = false;
  // }

  // end_->visited = false;
  // start_->pvisited = false;

  // distinct_paths.push_back();
  // INFO("start added")
  // INFO("start is " << start_)
  // INFO("end is " << end_)
  path_with_visited<T> start_path;
  start_path.path = {};
  start_path.visited_nodes = {};
  start_path.path_length = 0;
  int max_steps = nodes_.size() / 4.0;
  expand(start_, start_path, max_steps);
  std::vector<path_with_length<T>> paths;
  for (size_t i = 0; i < distinct_paths.size(); i++) {
    path_with_length<T> pwl;
    pwl.plan = distinct_paths[i].path;
    pwl.length = distinct_paths[i].path_length;
    paths.push_back(pwl);
  }
  // INFO("findPaths dfs end")
  return paths;
}

template<class T>
void DistinctPathDFS<T>::expand(HeapNode<T>* node,
                                path_with_visited<T> path_of_node,
                                const int& max_steps) {
  // HeapNode<T>* node = path_of_node.path.back();
  // INFO("expand " << node)
  if (path_of_node.visited_nodes.size() > max_steps) {
    // INFO("too much steps")
    return;
  }

  if (path_of_node.path.size() > 0) {
    path_of_node.path_length +=
      distance(path_of_node.path.back()->data, node->data);
  }
  path_of_node.path.push_back(node);
  path_of_node.visited_nodes.insert(node);

  if (node == end_) {
    distinct_paths.push_back(path_of_node);
    return;
  }

  for (const auto& nb : node->visibility_node_ids) {
    if (path_of_node.visited_nodes.count(nb.first) > 0) {
      // already visited
      continue;
    }


    // INFO("expand recursion")
    // path_of_node.path.push_back(nb.first);
    // path_of_node.visited_nodes.insert(nb.first);
    // INFO("\t size before expand " << path_of_node.path.size())
    expand(nb.first, path_of_node, max_steps);
    // INFO("\t size after expand " << path_of_node.path.size())
  }


  // for (const auto& nb : node->visibility_node_ids) {
  //   if (nb.first->visited || nb.first == end_) {
  //     // already visited or goal
  //     continue;
  //   }
  // }
}
