#pragma once

#include <vector>

#include "drone.hpp"

template<typename T>
struct TreeNode;

template<typename T>
struct TreeNode {
  long int id;
  long int nn_id;  // where the node is stored inside vector and nn
  T data;
  std::vector<TreeNode<T> *> children;
  TreeNode<T> *parent;
  double distance_from_start;
  // static int nodeCount;
  // static std::vector<TreeNode<T> *> nodes;

  TreeNode() {
    distance_from_start = 0;
    id = -1;
    parent = NULL;
  }

  bool operator==(const TreeNode &otn) const {
    if (this->id == otn.id) {
      return true;
    } else {
      return false;
    }
  }

  struct HashFunction {
    size_t operator()(const TreeNode &tn) const {
      return std::hash<int>()(tn.id);
    }
  };

  void setParent(TreeNode<T> *parent_) { this->parent = parent_; }
  TreeNode<T> *getParent() { return this->parent; }
  void addChild(TreeNode<T> *new_child_) { children.push_back(new_child_); }
  void removeChild(TreeNode<T> *to_remove) {
    bool removed = false;
    for (int var = 0; var < children.size(); ++var) {
      if (to_remove->id == children[var]->id) {
        children.erase(children.begin() + var);
        removed = true;
        break;
      }
    }
    if (!removed) {
      ERROR("removeChild failed to find the node");
      exit(1);
    }
  }

  void save_distance_children_recursive(double saved) {
    // INFO("save_distance_children_recursive for "<<this->id);
    distance_from_start -= saved;
    // this->check_distances_to_root();
    for (auto child : children) {
      child->save_distance_children_recursive(saved);
      // child->check_distances_to_root();
    }
  }

  static void savePathToFile(std::string filename,
                             std::vector<DroneState> all_states_between) {
    INFO("save path to file " << filename);
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (myfile.is_open()) {
      myfile << to_string_raw_header(all_states_between[0]) << std::endl;
      for (size_t i = 0; i < all_states_between.size(); i++) {
        myfile << to_string_raw(all_states_between[i]) << std::endl;
      }

      myfile.close();
    }
  }

  static void savePathToFile(std::string filename, TreeNode<T> *node) {
    INFO("save path to file " << filename);
    TreeNode<T> *tosave = node;
    std::ofstream myfile;
    std::string reversed_order;

    myfile.open(filename.c_str());
    if (myfile.is_open()) {
      myfile << "nid,parentid," << to_string_raw_header(node->data)
             << std::endl;


      while (tosave != NULL) {
        long unsigned int nodeID = tosave->id;
        long unsigned int parentID;

        if (tosave->parent == NULL) {
          parentID = nodeID;
        } else {
          parentID = tosave->parent->id;
        }

        std::stringstream line;
        line << nodeID << ",";
        line << parentID << ",";
        line << to_string_raw(tosave->data);
        line << std::endl;


        // INFO("line" << line.str());
        line << reversed_order;
        reversed_order = line.str();

        tosave = tosave->parent;
      }
      myfile << reversed_order;
      myfile.close();
    }
  }

  static std::vector<T> getStatesFromStart(TreeNode<T> *node) {
    TreeNode<T> *cur_node = node;
    std::vector<T> data;
    while (cur_node != NULL) {
      long unsigned int nodeID = cur_node->id;
      long unsigned int parentID;


      if (cur_node->parent == NULL) {
        parentID = nodeID;
      } else {
        parentID = cur_node->parent->id;
      }

      data.push_back(cur_node->data);

      cur_node = cur_node->parent;
    }
    std::reverse(data.begin(), data.end());
    return data;
  }

  static void saveTreeToFile(std::string filename,
                             std::vector<std::vector<TreeNode<T> *>> &nodes) {
    INFO("save tree to file " << filename);
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (myfile.is_open()) {
      if (nodes.size() > 0 && nodes[0].size() > 0) {
        myfile << "nid,parentid," << to_string_raw_header(nodes[0][0]->data)
               << std::endl;
      }
      TreeNode<T> *node;
      long unsigned int nodeID;
      long unsigned int parentID;
      for (int var1 = 0; var1 < nodes.size(); ++var1) {
        const int s = nodes[var1].size();
        for (int var2 = 0; var2 < s; ++var2) {
          node = nodes[var1][var2];
          nodeID = node->id;
          parentID = node->parent != NULL ? node->parent->id : nodeID;
          myfile << nodeID << ",";
          myfile << parentID << ",";
          myfile << to_string_raw(node->data);
          myfile << std::endl;
        }
      }
      // saveNodeToFile(this, myfile);
      // myfile << "];" << std::endl;
      myfile.close();
    }
  }


  static void saveOpenlistToFile(
    std::string filename,
    std::vector<std::unordered_map<long unsigned int, TreeNode<T> *>>
      &open_list) {
    INFO("save open_list to file " << filename);
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (myfile.is_open()) {
      if (open_list.size() > 0 && open_list[0].size() > 0) {
        myfile << "gate,nodeid,"
               << to_string_raw_header(open_list[0].begin()->second->data)
               << std::endl;
      }

      for (int gate = 0; gate < open_list.size(); ++gate) {
        // INFO("gate" << gate);
        for (std::pair<long unsigned int, TreeNode<T> *> element :
             open_list[gate]) {
          // INFO("element.first " << element.first << " sec " <<
          // element.second);
          if (element.second != NULL) {
            myfile << gate << ",";
            myfile << element.first << ",";
            myfile << to_string_raw(element.second->data);
            myfile << std::endl;
          }
          // INFO("aft")
        }
      }
      myfile.close();
    }
  }

  static std::vector<TreeNode<T> *> backtrace_to_root(TreeNode<T> *node) {
    std::vector<TreeNode<T> *> trajectory;
    TreeNode<T> *toadd = node;

    while (toadd != NULL) {
      long unsigned int nodeID = toadd->id;
      long unsigned int parentID;
      trajectory.push_back(toadd);

      if (toadd->parent == NULL) {
        parentID = nodeID;
      } else {
        parentID = toadd->parent->id;
      }

      // INFO("time:" << toadd->data.t << " command:" << toadd->data.command)
      // INFO("line" << line.str());
      toadd = toadd->parent;
    }

    std::reverse(trajectory.begin(), trajectory.end());
    return trajectory;
  }

  /*
   //uncomment and use when big changes done
   void check_distances_to_root(double collision_distance_check) {
   //INFO("check_distances_to_root for "<<this->id);
   double dist_calc_sum = 0;
   TreeNode<T> *current = this;
   while (current->parent != NULL) {
   const double dist_between = current->parent->data.distance(current->data);
   const double saved_distance_between = current->distance_from_start -
   current->parent->distance_from_start; if (fabs(dist_between -
   saved_distance_between) > EQUALITY_ERROR) { INFO("badly saved distance
   between "<<current->parent->id<<" "<<current->id<<" diff is
   "<<fabs(dist_between - saved_distance_between)); exit(1);
   }
   if (dist_between - collision_distance_check > EQUALITY_ERROR) {
   INFO("too large dist between"<< current->parent->id<<" and "<<current->id)
   INFO("too large dist_between "<<dist_between<<" compared to
   collision_distance_check "<<collision_distance_check); exit(1);
   }
   dist_calc_sum += dist_between;
   current = current->parent;
   }
   if (fabs(dist_calc_sum - this->distance_from_start) > EQUALITY_ERROR) {

   ERROR("distance from node "<< id<< " does not correspond to the one
   calculated"); ERROR("dist_calc_sum "<<dist_calc_sum);
   ERROR("this->distance_from_start " <<this->distance_from_start);
   ERROR("diff "<<this->distance_from_start-dist_calc_sum)
   exit(1);
   }
   }
   */
};
