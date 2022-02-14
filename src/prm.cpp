/*
 * PRM.cpp
 *
 *  Created on: 1. 2. 2015
 *      Author: Robert Pěnička
 */

#include "prm.hpp"


template<>
bool PRM<Vector<3>>::isInCollision(Vector<3> object_position) {
  const double clearance = map->getClearence(object_position);
  return !std::isfinite(clearance) || clearance < min_clearance_;
  // return MeshObject::collide(&obstacles, object, object_position);
}

template<>
void PRM<Vector<3>>::fillRandomState(HeapNode<Vector<3>>* positionToFill) {
  // positionToFill->city_node = false;
  positionToFill->data = (0.5 * (Vector<3>::Ones() + Vector<3>::Random()))
                           .cwiseProduct(position_range_) +
                         min_position_;
}

template<>
void PRM<Vector<3>>::fillPoint(HeapNode<Vector<3>>* positionToFill,
                               Vector<3> point) {
  positionToFill->data = point;
}

template<>
bool PRM<Vector<3>>::isPathFreeBetweenNodes(HeapNode<Vector<3>>* actual,
                                            HeapNode<Vector<3>>* neigbour) {
  double freeBetween = true;
  const double distance_to_neigbor = (actual->data - neigbour->data).norm();
  const double distance_max_changer =
    distance_to_neigbor / collision_distance_check_;
  double t, t1, distance_after;
  // HeapNode<HeapPoint3D> position_between;
  Vector<3> object_position;
  double index = 1.0;

  for (; index * collision_distance_check_ < distance_to_neigbor;
       index += 1.0) {
    object_position = neigbour->data + index * (actual->data - neigbour->data) /
                                         distance_max_changer;
    if (isInCollision(object_position)) {
      freeBetween = false;
      break;
    }
  }
  return freeBetween;
}


template<>
bool PRM<Vector<3>>::isInCollision(HeapNode<Vector<3>>* node) {
  return isInCollision(node->data);
}
