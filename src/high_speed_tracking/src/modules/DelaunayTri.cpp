/**
 * @file DelaunayTri.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the DelaunayTri class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "modules/DelaunayTri.hpp"

/* ----------------------------- Private Methods ---------------------------- */
//超级三角形是用来包围一组节点的初始三角形，在进行Delaunay三角剖分时，这个超级三角形会被去除。
Triangle DelaunayTri::superTriangle(const std::vector<Node> &nodes) {
  // Find the coords maxs and mins
  double xmax = nodes.front().x();         //front获取链表的第一个节点
  double xmin = nodes.front().x();
  double ymax = nodes.front().y();
  double ymin = nodes.front().y();
  for (const Node &n : nodes) {
    xmax = std::max(xmax, n.x());
    xmin = std::min(xmin, n.x());
    ymax = std::max(ymax, n.y());
    ymin = std::min(ymin, n.y());
  }

  const double dx = xmax - xmin;
  const double dy = ymax - ymin;
  const double dmax = std::max(dx, dy);
  const double midx = (xmin + xmax) / 2.0;
  const double midy = (ymin + ymax) / 2.0;

  const Node n0 = Node::superTriangleNode(midx - 20 * dmax, midy - dmax);
  const Node n1 = Node::superTriangleNode(midx, midy + 20 * dmax);
  const Node n2 = Node::superTriangleNode(midx + 20 * dmax, midy - dmax);

  return {n0, n1, n2};
}

/* ----------------------------- Public Methods ----------------------------- */
// 目的为了构建一个超级大三角形，完成包含所有点的工作，做这个工作的原因保证后续三角剖分的正确性与完整性

TriangleSet DelaunayTri::compute(const std::vector<Node> &nodes) {
  if (nodes.size() < 3) return {};
  //创建一个TriangleSet对象用于存储三角剖分结果
  TriangleSet triangulation;
  //添加一个足够大以包含所有点的超级三角形到三角剖分中
  //可以确保算法在进行后续的三角剖分操作时能够准确地考虑到所有的点，并生成一个正确的Delaunay三角剖分结果。
  triangulation.insert(superTriangle(nodes));

  // 将每个节点逐个添加到三角剖分中
  for (const Node &n : nodes) {
    TriangleSet badTriangles;

    // First find all the triangles that are no longer valid due to the insertion
    for (const Triangle &t : triangulation) {
      if (t.circleContainsNode(n)) {
        badTriangles.insert(t);
      }
    }

    EdgeSet polygon;

    // Find the boundary of the polygonal hole
    for (const Triangle &t : badTriangles) {
      for (const Edge &e : t.edges) {
        bool shared = false;
        for (const Triangle &t2 : badTriangles) {
          if (&t != &t2 and t2.containsEdge(e)) {
            shared = true;
            break;
          }
        }
        if (not shared) {
          polygon.insert(e);
        }
      }
    }

    // Remove every bad triangle from the triangulation data structure
    for (const Triangle &t : badTriangles) {
      triangulation.erase(t);
    }

    // Re-triangulate the polygonal hole
    for (const Edge &e : polygon) {
      triangulation.emplace(e, n);
    }
  }

  // Remove every triangle that belonged to the super triangle
  auto it = triangulation.begin();
  while (it != triangulation.end()) {
    if (it->anyNodeInSuperTriangle()) {
      it = triangulation.erase(it);
    } else {
      it++;
    }
  }

  return triangulation;
}