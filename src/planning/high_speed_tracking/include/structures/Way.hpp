/**
 * @file Way.hpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Way class specification
 * @version 1.0
 * @date 2022-10-31
 * 
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#pragma once

// #include <as_msgs/PathLimits.h>
#include <common_msgs/HUAT_PathLimits.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>

#include "structures/Edge.hpp"
#include "structures/Node.hpp"
#include "structures/Vector.hpp"
#include "utils/Params.hpp"
#include "utils/definitions.hpp"
#include "utils/constants.hpp"

/**
 * @brief Represents a way, i.e. the centerline and track limits.
 * It is the result of the WayComputer module.
 */
class Way {
 private:
  /**
   * @brief All parameters related to the Way class.
   */
  static Params::WayComputer::Way params_;

  /**
   * @brief The Way is represented with a list of Edge(s), these are directly
   * obtained from the Delaunay triangulation. In the list, the Edge(s) are
   * sorted as their position in the centerline. Track limits are intrinsically
   * here as well.
   * 是一个存储边（Edge）的列表，表示路径（Way）。
   * `path_` 是一个 `std::list<Edge>` 类型的变量，用于存储从Delaunay三角剖分中直接获取的边。
   * 在列表中，边按照它们在中心线上的位置排序。
   * 同时，路径限制（track limits）也内在地与其关联。
   * 通过这个列表，我们可以按顺序访问路径中的各个边。每个边对象都包含有关边的信息，例如起点、终点、长度等。
   * 路径限制也可以与每个边对象关联起来，以便在路径规划和执行过程中考虑到限制条件。
   * 通过访问 `path_` 列表中的边，我们可以遍历路径的每个点，并进行路径跟踪、路径规划和其他相关操作。
   * 。边与中心线起点的距离越短，它在列表中的位置就越靠前。这样可以确保列表中的边按照它们在中心线中的顺序排列。
   */
  std::list<Edge> path_;

  /**
   * @brief The average length of all the Edge(s) stored in \a path_.
   * This attribute will be used by WayComputer to find next Edge(s).
   * `avgEdgeLen_` 表示存储在 `path_` 中的所有边的平均长度。它将被 `WayComputer` 类使用，
   * 用于查找下一个边。通过计算边的平均长度，`avgEdgeLen_` 属性提供了重要的信息，
   * 可以帮助 `WayComputer` 选择合适的下一个边以继续路径的导航或控制。
   */
  double avgEdgeLen_;

  /**
   * @brief Points at the Edge whose midpoint is closest to the car position.
   * If empty, points to \a path_.cend().
   * `closestToCarElem_` 是一个指向 `std::list<Edge>` 类型中元素的常量迭代器。它被用于追踪距离汽车位置最近的边缘的位置
   * 如果 `closestToCarElem_` 为空，即没有找到最近的边缘，则它指向 `path_.cend()`，即指向路径列表 `path_` 的结束迭代器
   * 通过使用这个迭代器，我们可以确定距离汽车位置最近的边缘，
    std::list<Edge>` 是一个链表容器，它存储了 `Edge` 对象。
    const_iterator` 是 `std::list<Edge>` 类型定义的常量迭代器，
    用于遍历链表容器中的元素，并保证遍历过程中不会修改容器中的元素。
   */
  std::list<Edge>::const_iterator closestToCarElem_;

  /**
   * @brief Size of midpoints until the car is reached. Set after
   * trimming and useful for the color minimum_midpoints.
   * 例如，车辆可能已经行驶了一段距离，
   * 然后需要估计从路径起点到达当前车辆位置之前经过的路径部分的长度（用中点数量来衡量）
   */
  uint32_t sizeToCar_;

  struct Segment {
    Point a;
    Point b;
  };

  mutable std::unordered_set<Edge> edge_set_;
  mutable bool edge_set_dirty_ = true;

  mutable std::vector<Segment> segments_;
  mutable std::unordered_map<uint64_t, std::vector<size_t>> segment_grid_;
  mutable std::vector<uint32_t> segment_seen_;
  mutable uint32_t segment_seen_token_ = 1;
  mutable bool segment_index_dirty_ = true;
  mutable double segment_cell_size_ = 1.0;

  /**
   * @brief Updates the \a closestToCarElem_ attribute accordingly.
   */
  void updateClosestToCarElem();

  /**
   * @brief Checks if line segments defined by \a AB and \a CD intersect.
   * 
   * @param[in] A 
   * @param[in] B 
   * @param[in] C 
   * @param[in] D 
   */
  static bool segmentsIntersect(const Point &A, const Point &B, const Point &C, const Point &D);

  static uint64_t cellKey(int x, int y);
  void rebuildEdgeSet_() const;
  void rebuildSegmentIndex_() const;
  void addSegmentToIndex_(const Point &a, const Point &b) const;

 public:
  /**
   * @brief Method to initialize the Singleton.
   * 
   * @param[in] params 
   */
  static void init(const Params::WayComputer::Way &params);

  /**
   * @brief Construct a new Way object.
   */
  Way();

  /**
   * @brief Consults if the Way is empty.
   */
  bool empty() const;

  /**
   * @brief Consults the number of midpoints of the Way.
   */
  size_t size() const;

  /**
   * @brief Returns last Edge.
   */
  const Edge &back() const;

  /**
   * @brief Returns the penultimate Edge.
   */
  const Edge &beforeBack() const;

  /**
   * @brief Returns first Edge.
   */
  const Edge &front() const;

  /**
   * @brief Updates the local position of all Edge(s) in the Way.
   * 
   * @param[in] tf 
   */
  void updateLocal(const Eigen::Affine3d &tf);

  /**
   * @brief Appends an Edge to the back of the Way.
   * 
   * @param[in] edge 
   */
  void addEdge(const Edge &edge);

  /**
   * @brief Trims the Way so that all Edge(s) coming after the closest to the
   * car are removed.
   * 假设有一个Way对象，表示一条道路，由以下边构成：A -> B -> C -> D -> E -> F。车辆当前位置在边C上。
   * 调用`trimByLocal()`函数后，将会删除车边C之后的所有边。因此，最终的结果是保留了车辆当前位置及其之前的边，即A -> B -> C
   */
  void trimByLocal();

  /**
   * @brief Checks if the Way closes loop when \a e is appended to the Way.
   * If \a lastPosInTrace is not NULL, it is considered last Way midpoint.
   * 
   * @param[in] e 
   * @param[in] lastPosInTrace 
   */
  bool closesLoopWith(const Edge &e, const Point *lastPosInTrace = nullptr) const;

  /**
   * @brief Makes a copy making sure that:
   * - Possible spare points are removed. (e.g. when the loop closes with the second point).
   * - First and last Edge(s) coincide (EXACTLY equals).
   */
  Way restructureClosure() const;

  /**
   * @brief Checks if the loop is closed. This means:
   * - Size is longer than a threshold
   * - Distance between first and last midpoints exceeds a threshold
   */
  bool closesLoop() const;

  /**
   * @brief Checks if Edge \a e creates an intersection (a loop) on the path.
   * O(n), n=this->size().
   * 
   * @param[in] e 
   */
  bool intersectsWith(const Edge &e) const;

  /**
   * @brief Checks if the Way contains a specific Edge \a e.
   * 
   * @param[in] e
   */
  bool containsEdge(const Edge &e) const;

  /**
   * @brief Assignment operator.
   * 
   * @param[in] way 
   */
  Way &operator=(const Way &way);

  /**
   * @brief Comparison operator. Two Way(s) will be equal if both contain
   * the same Edge(s).
   * **Note** that the midpoints (and track limits) positions may not be equal.
   * 
   * @param[in] way 
   */
  bool operator==(const Way &way) const;

  /**
   * @brief Negation of the comparison operator.
   * 
   * @param[in] way 
   */
  bool operator!=(const Way &way) const;

  /**
   * @brief Checks if the vital_num_midpoints (the n midpoints after car's position)
   * are equal in both \a *this and \a way.
   * 
   * @param[in] way 
   */
  bool quinEhLobjetiuDeLaSevaDiresio(const Way &way) const;

  /**
   * @brief Returns the average Edge length.
   */
  const double &getAvgEdgeLen() const;

  /**
   * @brief Returns the number of midpoints ahead of the car, until end of Way.
   * Does not take loop closure into account.
   */
  uint32_t sizeAheadOfCar() const;

  /**
   * @brief Returns a vector with all the midpoints in global coordinates.
   */
  std::vector<Point> getPath() const;

  /**
   * @brief Returns the track limits. First element is left track limit
   * and second is right.
   */
  Tracklimits getTracklimits() const;

  /**
  * @brief 用于返回全局坐标下的路径
  */
  std::vector<Point> getPathLocal() const;

  void deleteWayPassed();
  Point getNextPathPoint();

  /**
   * @brief 使用去除走过的点进行插值（线性）
   * @param xy 车当前坐标
  */
  std::vector<geometry_msgs::Point> getPathInterpolation(double x,double y);

    /**
   * @brief 使用去除走过的点进行插值（线性，局部坐标系）
   * @param xy 车当前坐标
  */
  // std::vector<geometry_msgs::Point> getPathInterpolationLocal(double x,double y);

  /**
   * @brief 使用所有路径点插值（线性）
  */
  std::vector<geometry_msgs::Point> getPathFullInterpolation();

  /**
   * @brief 使用所有路径点插值（线性，局部坐标系）
  */
  // std::vector<geometry_msgs::Point> getPathFullInterpolationLocal();


  /**
   * @brief Cout operator.
   * 
   * @param[in,out] os 
   * @param[in] way 
   */
  friend std::ostream &operator<<(std::ostream &os, const Way &way);
};
