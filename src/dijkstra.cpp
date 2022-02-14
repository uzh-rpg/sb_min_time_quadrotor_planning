/*
 * Dijkstra.cpp
 *
 *  Created on: 2. 2. 2015
 *      Author: Robert Pěnička
 */

#include "dijkstra.hpp"

template<>
std::ostream& operator<<(std::ostream& o, const HeapNode<Point2D>& p) {
  o << p.id << " x=" << p.data.x << " y=" << p.data.y;
  return o;
}
