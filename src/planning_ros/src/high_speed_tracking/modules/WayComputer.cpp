/**
 * @file WayComputer.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the WayComputer class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "modules/WayComputer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "planning_core/speed_profile.hpp"

namespace {
double pointDistance(const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

double pointDistance(const Point &a, const Point &b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  return std::sqrt(dx * dx + dy * dy);
}

double signedCurvature(const geometry_msgs::Point &p0,
                       const geometry_msgs::Point &p1,
                       const geometry_msgs::Point &p2) {
  const double a = pointDistance(p0, p1);
  const double b = pointDistance(p1, p2);
  const double c = pointDistance(p0, p2);
  const double denom = a * b * c;
  if (denom < 1e-6) {
    return 0.0;
  }
  const double cross = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
  return 2.0 * cross / denom;
}

double signedCurvature(const Point &p0, const Point &p1, const Point &p2) {
  const double a = pointDistance(p0, p1);
  const double b = pointDistance(p1, p2);
  const double c = pointDistance(p0, p2);
  const double denom = a * b * c;
  if (denom < 1e-6) {
    return 0.0;
  }
  const double cross = (p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x);
  return 2.0 * cross / denom;
}

void smoothPathForCurvature(std::vector<geometry_msgs::Point> &path,
                            const double curvature_limit,
                            const int max_iters,
                            const double alpha) {
  if (path.size() < 3 || max_iters <= 0 || alpha <= 0.0 || curvature_limit <= 0.0) {
    return;
  }

  const double limit = std::max(1e-6, curvature_limit);
  const double blend = std::clamp(alpha, 0.0, 1.0);

  for (int iter = 0; iter < max_iters; ++iter) {
    bool changed = false;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
      const double kappa = std::abs(signedCurvature(path[i - 1], path[i], path[i + 1]));
      if (!std::isfinite(kappa) || kappa <= limit) {
        continue;
      }

      const double mid_x = 0.5 * (path[i - 1].x + path[i + 1].x);
      const double mid_y = 0.5 * (path[i - 1].y + path[i + 1].y);
      path[i].x = (1.0 - blend) * path[i].x + blend * mid_x;
      path[i].y = (1.0 - blend) * path[i].y + blend * mid_y;
      changed = true;
    }
    if (!changed) {
      break;
    }
  }
}

void smoothPathSpatialBlend(std::vector<geometry_msgs::Point> &path,
                            const int half_window,
                            const int passes,
                            const double blend) {
  if (path.size() < 3 || passes <= 0) {
    return;
  }
  const int hw = std::max(1, half_window);
  const double beta = std::clamp(blend, 0.0, 1.0);
  if (beta <= 0.0) {
    return;
  }

  const int n = static_cast<int>(path.size());
  for (int pass = 0; pass < passes; ++pass) {
    const std::vector<geometry_msgs::Point> original = path;
    for (int i = 1; i + 1 < n; ++i) {
      const int begin = std::max(0, i - hw);
      const int end = std::min(n - 1, i + hw);
      double sx = 0.0;
      double sy = 0.0;
      int count = 0;
      for (int j = begin; j <= end; ++j) {
        sx += original[j].x;
        sy += original[j].y;
        ++count;
      }
      if (count <= 0) {
        continue;
      }
      const double avg_x = sx / static_cast<double>(count);
      const double avg_y = sy / static_cast<double>(count);
      path[i].x = (1.0 - beta) * original[i].x + beta * avg_x;
      path[i].y = (1.0 - beta) * original[i].y + beta * avg_y;
    }
  }
}

int countCurvatureViolations(const std::vector<geometry_msgs::Point> &path,
                             const double curvature_limit) {
  const size_t n = path.size();
  if (n < 3 || curvature_limit <= 0.0) {
    return 0;
  }

  std::vector<double> curvatures(n, 0.0);
  for (size_t i = 1; i + 1 < n; ++i) {
    curvatures[i] = signedCurvature(path[i - 1], path[i], path[i + 1]);
  }
  curvatures.front() = curvatures[1];
  curvatures.back() = curvatures[n - 2];

  std::vector<double> smoothed = curvatures;
  for (size_t i = 1; i + 1 < n; ++i) {
    smoothed[i] = (curvatures[i - 1] + curvatures[i] + curvatures[i + 1]) / 3.0;
  }

  int violations = 0;
  for (size_t i = 0; i < n; ++i) {
    if (std::abs(smoothed[i]) > curvature_limit) {
      ++violations;
    }
  }
  return violations;
}

int countCurvatureViolations(const std::vector<Point> &path,
                             const double curvature_limit) {
  const size_t n = path.size();
  if (n < 3 || curvature_limit <= 0.0) {
    return 0;
  }

  std::vector<double> curvatures(n, 0.0);
  for (size_t i = 1; i + 1 < n; ++i) {
    curvatures[i] = signedCurvature(path[i - 1], path[i], path[i + 1]);
  }
  curvatures.front() = curvatures[1];
  curvatures.back() = curvatures[n - 2];

  std::vector<double> smoothed = curvatures;
  for (size_t i = 1; i + 1 < n; ++i) {
    smoothed[i] = (curvatures[i - 1] + curvatures[i] + curvatures[i + 1]) / 3.0;
  }

  int violations = 0;
  for (size_t i = 0; i < n; ++i) {
    if (std::abs(smoothed[i]) > curvature_limit) {
      ++violations;
    }
  }
  return violations;
}

bool detectLoopClosureByHistory(const std::vector<Point> &path_local,
                                const double dist_threshold,
                                const double angle_threshold,
                                double &out_best_dist,
                                int &out_best_idx,
                                double &out_best_angle) {
  out_best_dist = -1.0;
  out_best_idx = -1;
  out_best_angle = -1.0;

  const size_t n = path_local.size();
  constexpr size_t kTailExclusion = 10;
  if (n < MIN_LOOP_SIZE + kTailExclusion + 1) {
    return false;
  }

  const size_t search_end = n - kTailExclusion;
  const size_t front_window = std::min(
      search_end,
      std::max<size_t>(static_cast<size_t>(MIN_LOOP_SIZE), n / 3));
  if (front_window < 2) {
    return false;
  }

  const Point &tail = path_local.back();
  double best_dist = std::numeric_limits<double>::infinity();
  size_t best_idx = 0;
  for (size_t i = 0; i < front_window; ++i) {
    const double dist = Point::dist(path_local[i], tail);
    if (dist < best_dist) {
      best_dist = dist;
      best_idx = i;
    }
  }

  if (!std::isfinite(best_dist)) {
    return false;
  }
  out_best_dist = best_dist;
  out_best_idx = static_cast<int>(best_idx);
  if (best_dist > dist_threshold) {
    return false;
  }

  const Point &tail_prev = path_local[n - 2];
  const size_t next_idx = std::min(best_idx + 1, search_end - 1);
  if (next_idx == best_idx) {
    return false;
  }
  const Vector tail_dir(tail_prev, tail);
  const Vector hist_dir(path_local[best_idx], path_local[next_idx]);
  const double angle = std::abs(hist_dir.angleWith(tail_dir));
  out_best_angle = angle;
  return std::isfinite(angle) && angle <= angle_threshold;
}
}  // namespace

/* ----------------------------- Private Methods ---------------------------- */

void WayComputer::filterTriangulation(TriangleSet &triangulation) const {
  auto it = triangulation.begin();
  while (it != triangulation.end()) {
    bool removeTriangle = false;
    // Look for edges longer than accepted  初始化一个标志位，表示是否需要移除当前的三角形
    for (const Edge &e : it->edges) {        
      //如果边长不符合要求,标记移除 
      if (e.len > this->params_.max_triangle_edge_len) {
        removeTriangle = true;
        break;  
      }
    }

    // Look for angles smaller than accepted寻找角度小于最小允许值的角
    //在没有被标记移除的三角形中找到角度不符合要求的,标记移除
    if (!removeTriangle) {
      for (const double &angle : it->angles()) {
        if (angle < this->params_.min_triangle_angle) {
          removeTriangle = true;
          break;
        }
      }
    }

    //移除所有被标记为移除的三角形
    if (removeTriangle)
      it = triangulation.erase(it);
    else
      it++;
  }
}

void WayComputer::filterMidpoints(EdgeSet &edges, const TriangleSet &triangulation) const {
  // Build a k-d tree of all circumcenters so finding matches is O(logn)
  std::vector<Point> circums(triangulation.size());             //用于存储所有三角形的外接圆心
  //通过使用`std::transform`函数，遍历`triangulation`中的每个三角形，
  //并将其外接圆心（使用`t.circumCenter()`方法获取）添加到`circums`中。
  std::transform(triangulation.begin(), triangulation.end(), circums.begin(),
                 [](const Triangle &t) -> const Point & { return t.circumCenter(); });
  KDTree circumKDTree(circums); //这句代码的意思是以`circums`中的点作为初始数据，创建一个KD树对象`circumKDTree`

  // Perform the filtering
  auto it = edges.begin();
  while (it != edges.end()) {
    Point midPoint = it->midPoint();
    KDTData<size_t> nearestCC = circumKDTree.nearest_index(midPoint);  //通过比较目标点与KD树中的点，我们可以找到离目标点最近的外接圆心
    //如果`nearestCC`有效（非空）并且中点与最近外接圆心的距离的平方大于
    //`this->params_.max_dist_circum_midPoint`的平方，
    //则说明中点距离外接圆心太远，需要将该边从`edges`中移除
    if (bool(nearestCC) and Point::distSq(circums[*nearestCC], midPoint) > pow(this->params_.max_dist_circum_midPoint, 2)) {
      it = edges.erase(it);
    } else {
      it++;
    }
  }
}

double WayComputer::getHeuristic(const Point &actPos, const Point &nextPos, const Vector &dir, const Params::WayComputer::Search &params) const {
  double distHeur = Point::dist(actPos, nextPos);  //距离

  double angle = Vector(actPos, nextPos).angleWith(dir);
  double angleHeur = -log(std::max(0.0, ((M_PI_2 - abs(angle)) / M_PI_2) - 0.2)); //这个计算过程的目的是将夹角越接近0，启发式函数值越大

  return params.heur_dist_ponderation * distHeur + (1 - params.heur_dist_ponderation) * angleHeur;
}

inline double WayComputer::avgEdgeLen(const Trace *trace) const {
  if (not trace or trace->empty())
    return this->way_.getAvgEdgeLen();
  else if (this->way_.empty())
    return trace->avgEdgeLen();
  else
    return ((trace->avgEdgeLen() * trace->size()) / (trace->size() + this->way_.size())) + ((this->way_.getAvgEdgeLen() * this->way_.size()) / (trace->size() + this->way_.size()));
}

void WayComputer::findNextEdges(std::vector<HeurInd> &nextEdges, const Trace *actTrace, const KDTree &midpointsKDT, const std::vector<Edge> &edges, const Params::WayComputer::Search &params) const {
  nextEdges.clear();

//通常情况下，nullptr被用作指针的默认值，表示该指针当前没有指向任何有效的对象或内存地址。
//在接下来的代码中，可能会根据条件或逻辑的判断来更新actEdge的值，使其指向一个有效的边对象。
  const Edge *actEdge = nullptr;  // Last valid Edge (starting point)最近有效的边（起始点）表示当前没有有效的边
  Point actPos(0, 0);                     //表示当前的位置  名为actPos的Point对象，并初始化为(0, 0)。
  Point lastPos(0, 0);                    //上一时刻的位置


  // Set actual and last position设置当前位置和上一个位置
  if (actTrace and not actTrace->empty()) {
    actEdge = &edges[actTrace->edgeInd()];                //指的是跟踪的下一个边       
    actPos = edges[actTrace->edgeInd()].midPoint();     //当前的pos就以这个边的中点进行处理
    if (actTrace->size() >= 2) {
      lastPos = edges[actTrace->before().edgeInd()].midPoint();    //前一个的中点作为这个前一时刻处理。
    }
  }
  if (not this->way_.empty()) {             //way不为空为真
    if (not actEdge) {                                  //为空表示未定义 ，可以进入
      actEdge = &this->way_.back();                 //最后一个边
      actPos = this->way_.back().midPoint();    //最后一个边理解为车辆位置
      if (this->way_.size() >= 2) {
        lastPos = this->way_.beforeBack().midPoint();  //倒数第二个作为他的上一个点
      }
    } else if (actTrace->size() < 2) {
      lastPos = this->way_.back().midPoint();
    }
  }

  // Set dir vector  //表示方向向量
  //首先判断 `actEdge` 是否存在且满足条件 `(this->way_.size() >= 2 or actTrace)`。
  //这个条件语句的目的是避免设置一个指向后方的方向向量。换句话说，如果存在有效的边 (`actEdge`)，
  //并且当前路径 `this->way_` 的大小大于等于 2 或者存在有效的追踪信息 `actTrace`，则会执行条件语句中的代码块。
  Vector dir;
  if (actEdge and (this->way_.size() >= 2 or actTrace))  // This condition avoids setting a vector pointing backwards  用于防止设置一个指向后方的方向向量
    dir = Vector(lastPos, actPos);
  else
    dir = Vector(1, 0);   //表示指向 x 轴正方向的单位向量。

  // Find all possible edges in a specified radius
 // 这段代码的目的是在指定的半径范围内找到所有可能的边
 //actPos都是基于局部坐标系来进行处理的
 //无序不重复的数据
  std::unordered_set<size_t> nextPossibleEdges = midpointsKDT.neighborhood_indices_set(actPos, params.search_radius);

  // Discard edges by specifications
  //根据规则筛选并丢弃边
  auto it = nextPossibleEdges.begin();
  while (it != nextPossibleEdges.end()) {

    const Edge &nextPossibleEdge = edges[*it]; //是在将索引集合 `nextPossibleEdges` 中的索引值 `*it` 用作 `edges` 数组的索引，以获取对应的边对象。
    // The currently-iterated Edge will be removed if any of the below conditions is true
    //如果以下任何条件成立，当前迭代的边将被移除
    //and是所有为真时，整个表达式的结果才为真
    //actEdge非空才会继续向下判断。但是我这个地方一直空
    bool removeConditions = actEdge and (
      // 1. Remove itself from being the next one
      //到了最后一个边
      nextPossibleEdge == *actEdge or
      // 2. Remove any edge whose midpoint create an angle too closed with last one
      // 角度过大的边就删除
      abs(dir.angleWith(Vector(actPos, nextPossibleEdge.midPoint()))) > params.max_angle_diff or

      // 3. [Only before appending the edge that closes the loop] Remove any edge that is already contained in the path but is not the one that closes the loop
      // 它的作用是在路径尚未闭合之前，从候选边集合中移除任何已经存在于路径中但并不是用来闭合路径的边。
      (not this->way_.closesLoopWith(nextPossibleEdge) and (not actTrace or not actTrace->isLoopClosed()) and this->way_.containsEdge(nextPossibleEdge)) or

      // 4. Remove any edge whose midpoint and lastPos are in the same side of actEdge (avoid bouncing on a track limit)
      //该条件的目的是移除处于同一侧的候选边，以避免在路径上来回反弹。通过移除这样的边，可以确保路径的平滑性和连续性
      ((this->way_.size() >= 2 or actTrace) and Vector::pointBehind(actEdge->midPoint(), lastPos, actEdge->normal()) == Vector::pointBehind(actEdge->midPoint(), nextPossibleEdge.midPoint(), actEdge->normal())) or

      // 5. Remove any edge whose length is too big or too small compared to the average edge length of the way
      //该条件的目的是过滤掉边长与路径平均边长相差过大的边，以确保路径的边长变化合理。通过移除过大或过小的边长，
      //可以防止路径在边长较大或较小的区域上出现不连续或不平滑的变化。
      (nextPossibleEdge.len < (1 - params.edge_len_diff_factor) * this->avgEdgeLen(actTrace) or nextPossibleEdge.len > (1 + params.edge_len_diff_factor) * this->avgEdgeLen(actTrace)) or

      // 6. [If not allow_intersection, only before closing the loop] Remove any Edge which appended would create an intersection
      (not params.allow_intersection and (not actTrace or not actTrace->isLoopClosed()) and not this->way_.closesLoopWith(nextPossibleEdge) and this->way_.intersectsWith(nextPossibleEdge)));

    if (removeConditions)
      it = nextPossibleEdges.erase(it);
    else
      it++;
  }

  // Get the heuristics for all possible next edges and filter them (only the
  // ones having a heuristic small enough will prevail)
  // 这段代码计算所有可能下一个边的启发式函数，并对它们进行筛选（只有启发式函数足够小的边才被保留）。
  // using HeurInd = std::pair<double, size_t>;
  std::vector<HeurInd> privilege_runner; //用于存储带有启发式函数值的可能边
  privilege_runner.reserve(nextPossibleEdges.size());
  for (const size_t &nextPossibleEdgeInd : nextPossibleEdges) {
    double heuristic = this->getHeuristic(actPos, edges[nextPossibleEdgeInd].midPoint(), dir, params);
    if (heuristic <= params.max_next_heuristic) privilege_runner.emplace_back(heuristic, nextPossibleEdgeInd);
  }

  // Copy the n best HeurInd(s) into the nextEdges vector, according to
  // cppreference.com, this is O(nlogn)
  //作用是将最好的n个HeurInd复制到nextEdges向量中
  //从小到大。
  nextEdges.resize(std::min(privilege_runner.size(), (size_t)params.max_search_options));
  std::partial_sort_copy(privilege_runner.begin(), privilege_runner.end(), nextEdges.begin(), nextEdges.end());
}

Trace WayComputer::computeBestTraceWithFinishedT(const Trace &best, const Trace &t) const {
  // The method of choosing the best trace is as follows:
  // 1. The longest trace wins.
  // 2. If the size is equal, then the trace with smallest accum heuristic wins.
  // Note that here, no trace is added to the queue.
  //函数接受两个参数，分别是当前的最佳路径 `best` 和已完成的路径 `t`。
  //在函数内部，根据以下几个规则来选择最佳路径：
//1. 如果 `t` 的长度大于 `best` 的长度，则 `t` 是更长的路径，被认为是最佳路径。
//2. 如果 `t` 的长度与 `best` 的长度相等，并且 `t` 的累积启发值（accum heuristic）小于 `best` 的累积启发值，则 `t` 是更优的路径。
//总的来说，这段代码定义了一种选择最佳路径的规则，根据路径的长度和累积启发值来判断哪条路径更好，
//然后返回更好的路径作为最佳路径。
  if (t.size() > best.size() or (t.size() == best.size() and t.sumHeur() < best.sumHeur())) {
    return t;
  } else
    return best;
}

size_t WayComputer::treeSearch(std::vector<HeurInd> &nextEdges, const KDTree &midpointsKDT, const std::vector<Edge> &edges, const Params::WayComputer::Search &params) const {
  std::queue<Trace> cua;   //用于存储正在探索的路径 队列模板
  for (const HeurInd &nextEdge : nextEdges) {                                                                       //using HeurInd = std::pair<double, size_t>;
    bool closesLoop = this->way_.closesLoopWith(edges[nextEdge.second]);        // 传进去的是第二个元素
    cua.emplace(nextEdge.second, nextEdge.first, edges[nextEdge.second].len, closesLoop);
    //将获取的边的索引`nextEdge.second`、权重`nextEdge.first`、长度`edges[nextEdge.second].len`、
    //以及闭环标志`closesLoop`作为参数，构造一个对象，并将该对象添加到`cua`容器中。
  }
  // The provisional best is the Trace with smaller heuristic, i.e. the front one
  //这个`Trace`对象被认为是当前最佳的路径。在这段代码中，它使用了先进先出的原则，将队列的第一个元素作为暂时的最佳路径。
  Trace best = cua.front();

  // This loop will realize the tree search and get the longest (& best) path.
  // The loop will stop if the tree search is completed or a time limit has
  // been exceeded.
  //1. 当队列`cua`为空时，表示所有可能的路径已经探索完毕，树搜索完成。
  // 2. 当从树搜索开始时间到当前时间的耗时超过了`params.max_treeSearch_time`时限，表示超过了设定的时间限制。
  ros::Time searchBeginTime = ros::Time::now();
  while (not cua.empty()) {
    if (ros::Time::now() - searchBeginTime > ros::Duration(params.max_treeSearch_time)) {
      ROS_WARN("[high_speed_tracking] Time limit exceeded in tree search.");
      break;
    }
    //列的第一个元素并将其存储在`Trace`类型的变量`t`中，然后将这个元素从队列中移除
    //两行代码的作用是获取队列中的第一个路径（即当前要处理的路径），并将其从队列中移除，
    //以便在下一次循环中处理下一个路径。破折号之外的其他代码部分将继续对当前路径进行处理
    Trace t = cua.front();
    cua.pop();

    bool trace_at_max_height = false;
  //通过检查当前路径的长度是否达到了设置的最大搜索树高度，
  //可以决定是否需要停止在当前子树中进一步探索，而是直接将当前路径视为完成，并进行下一步的处理。
    if (t.size() >= params.max_search_tree_height)
      trace_at_max_height = true;
    else
      this->findNextEdges(nextEdges, &t, midpointsKDT, edges, params);

    if (trace_at_max_height or nextEdges.empty()) {
      // Means that this trace is finished, should be considered as the "best"
      // trace.
      best = this->computeBestTraceWithFinishedT(best, t);
    } else {
      // Add new possible traces to the queue
      for (const HeurInd &nextEdge : nextEdges) {
        Point actPos = edges[t.edgeInd()].midPoint();
        bool closesLoop = this->way_.closesLoopWith(edges[nextEdge.second], &actPos);
        Trace aux = t;
        aux.addEdge(nextEdge.second, nextEdge.first, edges[nextEdge.second].len, closesLoop);
        cua.push(aux);
      }
    }
  }
  // The next point will be the FIRST point of the best path
  //这段代码的目的是返回最佳路径中的起点，以便后续处理可以使用该点作为起始点
  return best.first().edgeInd();
}

void WayComputer::computeWay(const std::vector<Edge> &edges, const Params::WayComputer::Search &params) {
  // Get rid of all edges from closest to car (included) to last
  //从最接近车辆的边（包括最接近的边）到最后的边都会被删除
  //通过车的状态进行处理 把路径之后的点都给删除  重新通过传进来的边规划出新的路径
  this->way_.trimByLocal();

  // Build a k-d tree of all midpoints so is cheaper to find closest points
  //这段代码用于构建一个K-D树，该树存储了一组边的中点，以便更快地找到最近的点
  std::vector<Point> midpoints(edges.size());
  //`midpoints` 中存储的就是所有边的中点
  //利用本地坐标系来进行处理的  通过中点进行了K-D树建立。
  std::transform(edges.begin(), edges.end(), midpoints.begin(),
                 [](const Edge &e) -> Point { return e.midPoint(); });
  KDTree midpointsKDT(midpoints);

  // Find the longest and with lower heuristic Trace. Search will be conducted
  // through a tree. The tree height will be limited, at that point the first
  // element of the longest with lower heuristic Trace will be added to the
  // way. The search finishes when no element can be added to the way.
  //寻找最长且启发式函数较低的路径。进行搜索操作
  //通过一棵树来进行搜索操作。在进行搜索时，树的高度将会有限制。树是一种用于组织数据的数据结构，
  //它由节点（Node）和边（Edge）组成。
  //搜索算法会按照一定规则遍历树的节点，以找到特定的目标或达到特定的条件。
  //换句话说，搜索算法将会根据一定的规则在树上进行搜索，但是搜索的深度有限制。
  //当搜索达到树的高度限制时，算法会停止搜索，不再继续向下探索更深的节点。
  //在路径中添加具有最长且启发式函数较低的元素。当无法添加更多的元素到路径时，搜索结束
  //在搜索过程中，程序会持续地选择路径上具有最佳属性的元素进行添加。
  //在搜索的过程中，在每一步中，程序会评估路径上的元素，根据设定的启发式函数值和路径的长度来找到具有最佳属性的元素。
  //这个最佳属性可以是最长的路径或是启发式函数值最低的路径。

  // using HeurInd = std::pair<double, size_t>;
  // std::pair是C++标准库中的一个模板类，它可以存储两个不同类型的值。
  // 在这种情况下，HeurInd是一个由一个double类型的值和一个size_t类型的值组成的pair。
  std::vector<HeurInd> nextEdges;  

  // Get first set of possible Edges
  //获取第一组可能的边。
  //最终边储存在nextEdges中
  // 将nullptr传递给findNextEdges方法的第二个参数actTrace。nullptr是一个空指针常量，但是得看trace的具体定义。
  //Trace 类的作用是提供了一个用于记录和操作路径追踪信息的通用工具，使其更方便地用于路径规划等应用场景。

  //midpointsKDT是一个KDTree对象，用于存储中点的信息；
  //edges是一个存储边的向量  就是我进行了角度和外接圆判断后的边集合
  //params是一个Params::WayComputer::Search类型的对象，表示搜索的参数。
  this->findNextEdges(nextEdges, nullptr, midpointsKDT, edges, params);

  // Main outer loop, every iteration of this loop will involve adding one
  // midpoint to the path.
  while (ros::ok() and not nextEdges.empty() and (!params.max_way_horizon_size or this->way_.sizeAheadOfCar() <= params.max_way_horizon_size)) {
    // nextEdges 各种删除条件后备选的边   
    // midpointsKDT 存放各种中点边的kdt
    // edges 三角剖分去除这个过大角度和边长的边
    // params参数。
    size_t nextEdgeInd = this->treeSearch(nextEdges, midpointsKDT, edges, params);
    // Append the new Edge
    this->way_.addEdge(edges[nextEdgeInd]);

    // Check for loop closure
    if (this->way_.closesLoop()) {
      this->wayToPublish_ = this->way_.restructureClosure();
      this->loopClosedRaw_ = true;
      return;
    }
  //两次调用`findNextEdges`函数的目的是确保在每次循环迭代中，`nextEdges`向量都包含了当前路径状态下的可能边。通过在每个循环迭代中更新`nextEdges`向量，
  //可以确保路径搜索是基于最新状态的，并继续寻找最佳路径，直到满足循环条件终止循环执行。
    // Get next set of possible edges
    this->findNextEdges(nextEdges, nullptr, midpointsKDT, edges, params);
  }
  this->loopClosedRaw_ = false;
  this->wayToPublish_ = this->way_;
}

void WayComputer::fillPathDynamics(autodrive_msgs::HUAT_PathLimits &msg) const {
  const size_t n = msg.path.size();
  msg.curvatures.assign(n, 0.0);
  msg.target_speeds.assign(n, 0.0);

  if (n == 0) {
    return;
  }

  const auto &speed_cfg = this->params_.speed;
  const double kappa_limit = std::max(1e-6, speed_cfg.curvature_limit);
  const double warn_limit_scale = std::max(1.0, speed_cfg.curvature_warn_limit_scale);
  const double kappa_warn_limit = kappa_limit * warn_limit_scale;
  if (speed_cfg.curvature_smoothing_enable && n >= 3) {
    smoothPathForCurvature(msg.path,
                           kappa_limit,
                           speed_cfg.curvature_smoothing_max_iters,
                           speed_cfg.curvature_smoothing_alpha);
  }

  const int warn_min_points = std::max(1, speed_cfg.curvature_warn_min_points);
  const double warn_ratio = std::max(0.0, speed_cfg.curvature_warn_ratio);
  auto compute_curvatures_and_violations = [&]() -> int {
    if (n < 3) {
      std::fill(msg.curvatures.begin(), msg.curvatures.end(), 0.0);
      return 0;
    }
    for (size_t i = 1; i + 1 < n; ++i) {
      msg.curvatures[i] = signedCurvature(msg.path[i - 1], msg.path[i], msg.path[i + 1]);
    }
    msg.curvatures.front() = msg.curvatures[1];
    msg.curvatures.back() = msg.curvatures[n - 2];

    std::vector<double> smoothed = msg.curvatures;
    for (size_t i = 1; i + 1 < n; ++i) {
      smoothed[i] = (msg.curvatures[i - 1] + msg.curvatures[i] + msg.curvatures[i + 1]) / 3.0;
    }
    msg.curvatures.swap(smoothed);

    int violations = 0;
    for (size_t i = 0; i < n; ++i) {
      if (std::abs(msg.curvatures[i]) > kappa_warn_limit) {
        ++violations;
      }
    }
    return violations;
  };

  int kappa_violations = compute_curvatures_and_violations();
  double violation_ratio = static_cast<double>(kappa_violations) / static_cast<double>(n);
  const bool severe_violation =
      (kappa_violations >= warn_min_points) &&
      (violation_ratio >= std::max(0.05, warn_ratio * 0.6));

  if (speed_cfg.curvature_smoothing_enable && n >= 3 && severe_violation) {
    const std::vector<geometry_msgs::Point> path_backup = msg.path;
    const std::vector<double> curvature_backup = msg.curvatures;
    const int base_iters = std::max(1, speed_cfg.curvature_smoothing_max_iters);
    const int extra_iters = std::min(8, std::max(2, kappa_violations / warn_min_points));
    const int adaptive_iters = base_iters + extra_iters;
    const double adaptive_alpha =
        std::min(0.65, std::max(speed_cfg.curvature_smoothing_alpha, 0.25) + 0.20);
    const double adaptive_limit = std::max(1e-6, kappa_limit * 0.95);

    smoothPathForCurvature(msg.path, adaptive_limit, adaptive_iters, adaptive_alpha);
    const int adaptive_violations = compute_curvatures_and_violations();
    if (adaptive_violations <= kappa_violations) {
      if (adaptive_violations < kappa_violations) {
        ROS_INFO_THROTTLE(1.0,
                          "[high_speed_tracking] Curvature smoothing improved violations %d -> %d.",
                          kappa_violations, adaptive_violations);
      }
      kappa_violations = adaptive_violations;
      violation_ratio = static_cast<double>(kappa_violations) / static_cast<double>(n);
    } else {
      msg.path = path_backup;
      msg.curvatures = curvature_backup;
    }
  }

  auto try_extra_smoothing = [&](double limit_scale, int extra_iters, double alpha) {
    const std::vector<geometry_msgs::Point> path_backup = msg.path;
    const std::vector<double> curvature_backup = msg.curvatures;
    const int prev_violations = kappa_violations;
    const double prev_ratio = violation_ratio;

    const double scaled_limit = std::max(1e-6, kappa_limit * limit_scale);
    smoothPathForCurvature(msg.path, scaled_limit, extra_iters, alpha);
    const int new_violations = compute_curvatures_and_violations();
    const double new_ratio = static_cast<double>(new_violations) / static_cast<double>(n);

    if (new_violations <= prev_violations) {
      if (new_violations < prev_violations) {
        ROS_INFO_THROTTLE(1.0,
                          "[high_speed_tracking] Extra curvature smoothing improved violations %d -> %d.",
                          prev_violations, new_violations);
      }
      kappa_violations = new_violations;
      violation_ratio = new_ratio;
    } else {
      msg.path = path_backup;
      msg.curvatures = curvature_backup;
      kappa_violations = prev_violations;
      violation_ratio = prev_ratio;
    }
  };

  auto try_spatial_blend_smoothing = [&](int half_window, int passes, double blend) {
    const std::vector<geometry_msgs::Point> path_backup = msg.path;
    const std::vector<double> curvature_backup = msg.curvatures;
    const int prev_violations = kappa_violations;
    const double prev_ratio = violation_ratio;

    smoothPathSpatialBlend(msg.path, half_window, passes, blend);
    const int new_violations = compute_curvatures_and_violations();
    const double new_ratio = static_cast<double>(new_violations) / static_cast<double>(n);

    if (new_violations <= prev_violations) {
      if (new_violations < prev_violations) {
        ROS_INFO_THROTTLE(
            1.0,
            "[high_speed_tracking] Spatial curvature smoothing improved violations %d -> %d (window=%d passes=%d).",
            prev_violations, new_violations, half_window, passes);
      }
      kappa_violations = new_violations;
      violation_ratio = new_ratio;
    } else {
      msg.path = path_backup;
      msg.curvatures = curvature_backup;
      kappa_violations = prev_violations;
      violation_ratio = prev_ratio;
    }
  };

  if (speed_cfg.curvature_smoothing_enable && n >= 3) {
    const bool still_warn =
        (kappa_violations >= warn_min_points) &&
        (violation_ratio >= std::max(0.05, warn_ratio * 0.8));
    if (still_warn) {
      const int base_iters = std::max(1, speed_cfg.curvature_smoothing_max_iters);
      const double alpha_stage2 =
          std::min(0.75, std::max(speed_cfg.curvature_smoothing_alpha, 0.30) + 0.25);
      try_extra_smoothing(0.90, base_iters + 10, alpha_stage2);

      const bool still_bad =
          (kappa_violations >= warn_min_points) &&
          (violation_ratio >= std::max(0.05, warn_ratio * 0.8));
      if (still_bad) {
        const double alpha_stage3 = std::min(0.80, alpha_stage2 + 0.10);
        try_extra_smoothing(0.85, base_iters + 16, alpha_stage3);
      }

      const bool still_severe =
          (kappa_violations >= warn_min_points) &&
          (violation_ratio >= std::max(0.05, warn_ratio * 0.8));
      if (still_severe) {
        try_spatial_blend_smoothing(2, 2, 0.45);
      }
      const bool still_high =
          (kappa_violations >= warn_min_points) &&
          (violation_ratio >= std::max(0.05, warn_ratio * 0.8));
      if (still_high) {
        try_spatial_blend_smoothing(3, 2, 0.55);
      }
    }
  }

  if (kappa_violations >= warn_min_points && violation_ratio >= warn_ratio) {
    ROS_WARN_THROTTLE(1.0,
                      "[high_speed_tracking] Curvature exceeds warning limit %.3f (base %.3f x %.2f) at %d/%zu points (ratio=%.3f).",
                      kappa_warn_limit, kappa_limit, warn_limit_scale,
                      kappa_violations, n, violation_ratio);
  }

  // Mode-blended speed cap (SAFE_LAP → FAST_LAP transition)
  const double safe_cap = std::max(0.0, speed_cfg.speed_cap_safe);
  const double fast_cap = std::max(0.0, speed_cfg.speed_cap_fast);
  const double blend = std::clamp(this->modeSpeedBlend_, 0.0, 1.0);
  const double v_cap = std::max(0.0, safe_cap + blend * (fast_cap - safe_cap));

  // Convert to Point2D for shared speed profile module
  std::vector<planning_core::Point2D> pts(n);
  for (size_t i = 0; i < n; ++i) {
    pts[i].x = msg.path[i].x;
    pts[i].y = msg.path[i].y;
  }

  planning_core::SpeedProfileParams sp;
  sp.speed_cap = v_cap;
  sp.max_lateral_acc = speed_cfg.max_lateral_acc;
  sp.max_accel = speed_cfg.max_accel;
  sp.max_brake = speed_cfg.max_brake;
  sp.min_speed = speed_cfg.min_speed;
  sp.curvature_epsilon = speed_cfg.curvature_epsilon;
  sp.current_speed = std::isfinite(this->CarState.V) ? static_cast<double>(this->CarState.V) : 0.0;

  planning_core::ComputeSpeedProfile(pts, msg.curvatures, sp, msg.target_speeds);

  // Sanitize curvatures (speed profile already sanitizes target_speeds)
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(msg.curvatures[i])) {
      msg.curvatures[i] = 0.0;
    }
  }
}

void WayComputer::updateLapMode(bool loop_closed_now) {
  const auto &speed_cfg = this->params_.speed;
  const int close_frames = std::max(1, speed_cfg.loop_close_debounce_frames);
  const int open_frames = std::max(1, speed_cfg.loop_open_debounce_frames);
  const bool allow_reopen = speed_cfg.loop_allow_reopen;
  const int hold_frames = std::max(0, speed_cfg.mode_min_hold_frames);
  const int transition_frames = std::max(1, speed_cfg.mode_transition_frames);

  if (loop_closed_now) {
    ++this->loopCloseDebounceCount_;
    this->loopOpenDebounceCount_ = 0;
  } else {
    ++this->loopOpenDebounceCount_;
    this->loopCloseDebounceCount_ = 0;
  }

  if (this->modeHoldFrames_ > 0) {
    --this->modeHoldFrames_;
  }

  const LapMode prev_mode = this->lapMode_;
  if (this->lapMode_ == LapMode::MAP_BUILD_SAFE) {
    if (this->loopCloseDebounceCount_ >= close_frames) {
      this->lapMode_ = LapMode::FAST_LAP;
      this->modeHoldFrames_ = hold_frames;
      this->loopCloseDebounceCount_ = 0;
      this->loopOpenDebounceCount_ = 0;
    }
  } else {
    if (allow_reopen &&
        this->modeHoldFrames_ == 0 &&
        this->loopOpenDebounceCount_ >= open_frames) {
      this->lapMode_ = LapMode::MAP_BUILD_SAFE;
      this->modeHoldFrames_ = hold_frames;
      this->loopCloseDebounceCount_ = 0;
      this->loopOpenDebounceCount_ = 0;
    }
  }

  this->isLoopClosed_ = (this->lapMode_ == LapMode::FAST_LAP);

  const double target_blend = (this->lapMode_ == LapMode::FAST_LAP) ? 1.0 : 0.0;
  const double blend_step = 1.0 / static_cast<double>(transition_frames);
  if (this->modeSpeedBlend_ < target_blend) {
    this->modeSpeedBlend_ = std::min(target_blend, this->modeSpeedBlend_ + blend_step);
  } else if (this->modeSpeedBlend_ > target_blend) {
    this->modeSpeedBlend_ = std::max(target_blend, this->modeSpeedBlend_ - blend_step);
  }

  if (prev_mode != this->lapMode_) {
    const char *new_mode = (this->lapMode_ == LapMode::FAST_LAP) ? "FAST_LAP" : "MAP_BUILD_SAFE";
    ROS_INFO_STREAM("[high_speed_tracking] Lap mode switched to " << new_mode
                    << " (loop_closed_now=" << (loop_closed_now ? "true" : "false") << ")");
  }
}

/* ----------------------------- Public Methods ----------------------------- */

WayComputer::WayComputer(const Params::WayComputer &params) : params_(params), currentStamp_(0) {
  Way::init(params.way);
  this->generalFailsafe_.initGeneral(this->params_.search, this->params_.general_failsafe_safetyFactor, this->params_.failsafe_max_way_horizon_size);
}

//作用是获取车辆在全局坐标系下的姿态，获取TF，然后通过TF完成全局转局部的方案。
void WayComputer::stateCallback(const autodrive_msgs::HUAT_CarState::ConstPtr &ins) {
  CarState.car_state = ins->car_state;
  CarState.header = ins->header;
  CarState.V = ins->V;
  pose.position.x = ins->car_state.x;
  pose.position.y = ins->car_state.y;
  pose.position.z = 0;
  tf::Quaternion qAux;
  qAux.setRPY(0.0, 0.0, ins->car_state.theta);                                                                
  tf::quaternionTFToMsg(qAux, pose.orientation);                                                         
  tf::poseMsgToEigen(pose, this->localTf_);                             //localTf_ 我在获取了全局姿态下的位资后。
  this->localTf_ = this->localTf_.inverse();         
  this->localTfValid_ = true;
}

void WayComputer::update(TriangleSet &triangulation, const ros::Time &stamp) {
  if (not this->localTfValid_) {
    ROS_WARN("[high_speed_tracking] CarState not being received.");
    return;
  }

  // #0: Update last way (this will be used to calculate the raplan flag).
  //     And update stamp.
  //根据代码，`this->way_` 可以认为是一个不断累加的全局路径，而 `this->lastWay_` 用于存储上一次的全局路径
  this->lastWay_ = this->way_;                    //Way构造函数直接赋值  表示路径
  this->lastStamp_ = stamp;
  this->currentStamp_ = stamp;

  // #1: Remove all triangles which we know will not be part of the track.用于从三角测量中移除那些不会成为轨迹的三角形
  // 实际上里面的运算还是一个局部坐标系上的点进行操作。
  this->filterTriangulation(triangulation);
  last_filtered_triangulation_ = triangulation;
  last_triangle_count_ = triangulation.size();

  // #2: Extract all midpoints without repetitions, do that through an EdgeSet
  // so no midpoint is got twice.
  //通过使用一个`EdgeSet`来提取所有的中点（midpoints），并且确保没有重复的中点被获取两次
  //using EdgeSet = std::unordered_set<Edge>;
  // `std::unordered_set` 实现了一个无序集合 (unordered set)  其中元素的类型是 `Edge`
  EdgeSet edgeSet;            //可以方便地存储并检索一组不重复的边对象
  for (const Triangle &t : triangulation) {
    for (const Edge &e : t.edges) {
      edgeSet.insert(e);   //调用 insert() 函数，将边e插入到edgeSet中，如果 e已经存在于集合中，则不会重复插入，刚好解决了中点不被获取两次的问题
    }
  }

  // #3: Filter the midpoints. Only the ones having a circumcenter near, will be
  // left.
  //只保留那些与三角形外接圆心距离较近的中点。
  this->filterMidpoints(edgeSet, triangulation);
  last_filtered_edges_ = edgeSet;
  last_edge_count_ = edgeSet.size();

  // Convert this set to a vector
  //`edgeSet`中的边被转换成了一个向量`edgeVec`，这可能是为了方便后续处理，例如对边进行排序或其他操作。
  std::vector<Edge> edgeVec;
  edgeVec.reserve(edgeSet.size());
  for (const Edge &e : edgeSet) {
    edgeVec.push_back(e);
  }

  // #4: Update all local positions (way and edges) with car tf、
  //在这个步骤中，我们将使用车辆的tf（变换矩阵）来更新所有本地位置，包括路径和边缘。
  //作用只是更新了局部点  通过车辆状态和全局坐标。
  // 通过全局去转局部
  this->way_.updateLocal(this->localTf_);         
  for (const Edge &e : edgeVec) {
    e.updateLocal(this->localTf_);                        
  }

  const auto &speed_cfg = this->params_.speed;
  const size_t min_publish_size = static_cast<size_t>(std::max(1, speed_cfg.min_publish_path_size));

  auto compute_adaptive_level = [&](const int short_streak, const bool force_retry) -> int {
    if (!speed_cfg.search_adaptive_enable) return 0;
    const int trigger_frames = std::max(1, speed_cfg.search_adaptive_trigger_frames);
    const int step_frames = std::max(1, speed_cfg.search_adaptive_step_frames);
    const int max_level = std::max(0, speed_cfg.search_adaptive_max_level);
    if (max_level == 0) return 0;
    if (short_streak < trigger_frames && !force_retry) return 0;
    const int effective = std::max(0, short_streak - trigger_frames);
    int level = 1 + (effective / step_frames);
    if (force_retry) {
      level += 1;
    }
    return std::min(level, max_level);
  };

  auto apply_adaptive_search = [&](Params::WayComputer::Search &search_cfg, const int level) {
    if (level <= 0) return;
    const double radius_step = std::max(0.0, speed_cfg.search_adaptive_radius_step);
    const double heuristic_step = std::max(0.0, speed_cfg.search_adaptive_heuristic_step);
    const double radius_limit = (speed_cfg.search_adaptive_radius_max > 0.0)
                                    ? speed_cfg.search_adaptive_radius_max
                                    : this->params_.search.search_radius;
    const double heuristic_limit = (speed_cfg.search_adaptive_heuristic_max > 0.0)
                                       ? speed_cfg.search_adaptive_heuristic_max
                                       : this->params_.search.max_next_heuristic;
    search_cfg.search_radius = std::min(
        radius_limit,
        this->params_.search.search_radius + radius_step * static_cast<double>(level));
    search_cfg.max_next_heuristic = std::min(
        heuristic_limit,
        this->params_.search.max_next_heuristic + heuristic_step * static_cast<double>(level));
  };

  auto adaptive_min_publish_threshold = [&](const int level) -> size_t {
    const size_t base = min_publish_size;
    if (!speed_cfg.search_adaptive_enable || level <= 0) {
      return base;
    }
    const int scaled_floor = std::max(1, (speed_cfg.min_publish_path_size * 2) / 5);
    const size_t floor = static_cast<size_t>(
        std::max<int>(static_cast<int>(MIN_FAILSAFE_WAY_SIZE) + 1,
                      scaled_floor));
    const size_t relax = static_cast<size_t>(level);
    if (base <= relax) {
      return floor;
    }
    return std::max(floor, base - relax);
  };

  auto stable_short_path_threshold = [&](const size_t threshold) -> size_t {
    size_t relaxed = threshold;
    if (this->hasStableWay_) {
      const size_t relax_pts = static_cast<size_t>(
          std::max(0, speed_cfg.short_path_stable_relax_points));
      if (relax_pts > 0) {
        if (relaxed > relax_pts) {
          relaxed -= relax_pts;
        } else {
          relaxed = 0;
        }
      }
    }
    return std::max(
        relaxed,
        static_cast<size_t>(std::max<int>(1, static_cast<int>(MIN_FAILSAFE_WAY_SIZE) + 1)));
  };

  // #5: Perform the search through the midpoints in order to obtain a way.
  //     When short-path streak appears, expand search radius and heuristic threshold.
  int adaptive_level = compute_adaptive_level(this->shortPathStreak_, false);
  if (speed_cfg.search_adaptive_enable && !this->hasStableWay_) {
    const int bootstrap_level = std::clamp(speed_cfg.search_adaptive_bootstrap_level,
                                           0,
                                           std::max(0, speed_cfg.search_adaptive_max_level));
    adaptive_level = std::max(adaptive_level, bootstrap_level);
  }
  if (adaptive_level != this->adaptiveSearchLevel_) {
    ROS_INFO_STREAM_THROTTLE(1.0,
                             "[high_speed_tracking] Adaptive search level "
                                 << this->adaptiveSearchLevel_ << " -> " << adaptive_level
                                 << " (short_streak=" << this->shortPathStreak_ << ").");
    this->adaptiveSearchLevel_ = adaptive_level;
  }

  Params::WayComputer::Search primary_search = this->params_.search;
  apply_adaptive_search(primary_search, adaptive_level);
  this->computeWay(edgeVec, primary_search);

  size_t min_publish_size_adaptive = adaptive_min_publish_threshold(adaptive_level);
  size_t short_path_threshold = stable_short_path_threshold(min_publish_size_adaptive);
  bool short_path = this->wayToPublish_.size() < short_path_threshold;
  if (short_path && speed_cfg.search_adaptive_enable && speed_cfg.search_adaptive_retry_on_short && !this->loopClosedRaw_) {
    int retry_level = compute_adaptive_level(this->shortPathStreak_, true);
    if (speed_cfg.search_adaptive_enable && !this->hasStableWay_) {
      const int bootstrap_retry = std::clamp(speed_cfg.search_adaptive_bootstrap_level + 1,
                                             0,
                                             std::max(0, speed_cfg.search_adaptive_max_level));
      retry_level = std::max(retry_level, bootstrap_retry);
    }
    if (retry_level > adaptive_level) {
      Params::WayComputer::Search retry_search = this->params_.search;
      apply_adaptive_search(retry_search, retry_level);
      this->computeWay(edgeVec, retry_search);
      min_publish_size_adaptive = adaptive_min_publish_threshold(retry_level);
      short_path_threshold = stable_short_path_threshold(min_publish_size_adaptive);
      short_path = this->wayToPublish_.size() < short_path_threshold;
      if (!short_path) {
        ROS_INFO_THROTTLE(1.0,
                          "[high_speed_tracking] Short-path recovered by adaptive retry (level=%d).",
                          retry_level);
      }
    }
  }

  // #6: Check failsafe(s)
  //仅当“前方可用路径过短”且当前没有可用hold缓冲时，触发通用failsafe。
  bool ahead_path_short = this->way_.sizeAheadOfCar() < MIN_FAILSAFE_WAY_SIZE;
  if (ahead_path_short && speed_cfg.search_adaptive_enable && !this->loopClosedRaw_) {
    const int rescue_level = std::clamp(
        std::max(adaptive_level + 1, compute_adaptive_level(this->shortPathStreak_, true)),
        0, std::max(0, speed_cfg.search_adaptive_max_level));
    if (rescue_level > adaptive_level) {
      Params::WayComputer::Search rescue_search = this->params_.search;
      apply_adaptive_search(rescue_search, rescue_level);
      this->computeWay(edgeVec, rescue_search);
      min_publish_size_adaptive = adaptive_min_publish_threshold(rescue_level);
      short_path_threshold = stable_short_path_threshold(min_publish_size_adaptive);
      short_path = this->wayToPublish_.size() < short_path_threshold;
      ahead_path_short = this->way_.sizeAheadOfCar() < MIN_FAILSAFE_WAY_SIZE;
      if (!ahead_path_short) {
        ROS_INFO_THROTTLE(1.0,
                          "[high_speed_tracking] Ahead-path recovered by adaptive rescue (level=%d).",
                          rescue_level);
      }
    }
  }

  const bool can_hold_now = speed_cfg.hold_last_valid_path && this->hasStableWay_ && this->holdFramesRemaining_ > 0;
  if (this->params_.general_failsafe && !this->loopClosedRaw_ && ahead_path_short && !can_hold_now) {
    ROS_WARN_THROTTLE(1.0,
                      "[high_speed_tracking] GENERAL FAILSAFE ACTIVATED (ahead=%u, publish=%zu).",
                      static_cast<unsigned int>(this->way_.sizeAheadOfCar()),
                      this->wayToPublish_.size());
    this->computeWay(edgeVec, this->generalFailsafe_);
    short_path_threshold = stable_short_path_threshold(min_publish_size_adaptive);
    short_path = this->wayToPublish_.size() < short_path_threshold;
  }

  bool loop_closed_strict = this->loopClosedRaw_;
  bool loop_closed_history = false;
  bool loop_closed_stable_source = false;
  double loop_hist_dist = -1.0;
  int loop_hist_idx = -1;
  double loop_hist_angle = -1.0;

  std::vector<Point> way_local_for_loop = this->way_.getPathLocal();
  if (!this->loopClosedRaw_ &&
      this->hasStableWay_ &&
      way_local_for_loop.size() < MIN_LOOP_SIZE) {
    if (this->lastStableWay_.closesLoop()) {
      this->loopClosedRaw_ = true;
      loop_closed_strict = true;
      loop_closed_stable_source = true;
      ROS_INFO_THROTTLE(
          1.0,
          "[high_speed_tracking] Geometric loop closure by stable-path strict check.");
    } else {
      const std::vector<Point> stable_way_local = this->lastStableWay_.getPathLocal();
      if (stable_way_local.size() > way_local_for_loop.size()) {
        way_local_for_loop = stable_way_local;
        loop_closed_stable_source = true;
      }
    }
  }

  if (!this->loopClosedRaw_) {
    const double hist_dist_threshold =
        std::max(0.2, this->params_.way.max_dist_loop_closure * 1.10);
    const double hist_angle_threshold =
        std::min(M_PI_2, this->params_.way.max_angle_diff_loop_closure + 0.25);
    loop_closed_history = detectLoopClosureByHistory(way_local_for_loop,
                                                     hist_dist_threshold,
                                                     hist_angle_threshold,
                                                     loop_hist_dist,
                                                     loop_hist_idx,
                                                     loop_hist_angle);
    if (loop_closed_history) {
      this->loopClosedRaw_ = true;
      ROS_INFO_THROTTLE(
          1.0,
          "[high_speed_tracking] Geometric loop closure by %s history candidate (dist=%.3f idx=%d angle=%.3f).",
          loop_closed_stable_source ? "stable-path" : "current-path",
          loop_hist_dist,
          loop_hist_idx,
          loop_hist_angle);
    }
  }

  this->updateLapMode(this->loopClosedRaw_);

  if (speed_cfg.loop_diag_enable) {
    const std::vector<Point> way_local = this->way_.getPathLocal();
    const size_t wn = way_local.size();
    double dist_front_back = -1.0;
    double dist_back_hist_min = -1.0;
    int min_hist_idx = -1;
    if (wn >= 2) {
      dist_front_back = Point::dist(way_local.front(), way_local.back());
      if (wn > 12) {
        double best = std::numeric_limits<double>::infinity();
        const Point &tail = way_local.back();
        for (size_t i = 0; i + 10 < wn; ++i) {
          const double d = Point::dist(way_local[i], tail);
          if (d < best) {
            best = d;
            min_hist_idx = static_cast<int>(i);
          }
        }
        if (std::isfinite(best)) {
          dist_back_hist_min = best;
        }
      }
    }
    ROS_INFO_THROTTLE(
        1.0,
        "[high_speed_tracking] loop_diag raw=%d strict=%d hist=%d stable_src=%d way_size=%zu dist_front_back=%.3f dist_back_hist_min=%.3f min_hist_idx=%d hist_dist=%.3f hist_idx=%d hist_angle=%.3f",
        this->loopClosedRaw_ ? 1 : 0,
        loop_closed_strict ? 1 : 0,
        loop_closed_history ? 1 : 0,
        loop_closed_stable_source ? 1 : 0,
        wn,
        dist_front_back,
        dist_back_hist_min,
        min_hist_idx,
        loop_hist_dist,
        loop_hist_idx,
        loop_hist_angle);
  }

  const std::vector<Point> quality_path = this->wayToPublish_.getPathLocal();
  const double quality_curvature_limit = std::max(
      1e-6, speed_cfg.curvature_limit * std::max(1.0, speed_cfg.curvature_warn_limit_scale));
  const int kappa_violations = countCurvatureViolations(
      quality_path, quality_curvature_limit);
  const double kappa_ratio = quality_path.empty()
                                 ? 0.0
                                 : static_cast<double>(kappa_violations) /
                                       static_cast<double>(quality_path.size());
  const bool bad_curvature_quality =
      !short_path &&
      (kappa_violations >= std::max(1, speed_cfg.curvature_warn_min_points)) &&
      (kappa_ratio >= std::max(0.0, speed_cfg.curvature_warn_ratio));

  if (!short_path && !bad_curvature_quality) {
    this->lastStableWay_ = this->wayToPublish_;
    this->hasStableWay_ = true;
    this->holdFramesRemaining_ = std::max(0, speed_cfg.hold_last_valid_max_frames);
  } else if (speed_cfg.hold_last_valid_path && this->hasStableWay_ &&
             this->holdFramesRemaining_ > 0) {
    const size_t degraded_path_size = this->wayToPublish_.size();
    this->wayToPublish_ = this->lastStableWay_;
    this->way_ = this->lastStableWay_;
    --this->holdFramesRemaining_;
    if (bad_curvature_quality) {
      ROS_WARN_THROTTLE(
          1.0,
          "[high_speed_tracking] Holding last stable path (curvature quality: %d/%zu > %.3f, remaining_hold=%d).",
          kappa_violations, quality_path.size(),
          std::max(0.0, speed_cfg.curvature_warn_ratio), this->holdFramesRemaining_);
    } else {
      ROS_WARN_THROTTLE(
          1.0,
          "[high_speed_tracking] Holding last stable path (short path: %zu < %zu, remaining_hold=%d).",
          degraded_path_size, short_path_threshold, this->holdFramesRemaining_);
    }
  } else if (short_path || bad_curvature_quality) {
    if (bad_curvature_quality) {
      ROS_WARN_THROTTLE(
          1.0,
          "[high_speed_tracking] Curvature quality degraded without hold fallback (%d/%zu exceeds ratio %.3f).",
          kappa_violations, quality_path.size(),
          std::max(0.0, speed_cfg.curvature_warn_ratio));
    } else {
      ROS_WARN_THROTTLE(
          1.0,
          "[high_speed_tracking] Short path without hold fallback (%zu < %zu).",
          this->wayToPublish_.size(), short_path_threshold);
    }
    this->holdFramesRemaining_ = 0;
  }

  if (short_path || bad_curvature_quality) {
    this->shortPathStreak_ = std::min(this->shortPathStreak_ + 1, 1000000);
  } else {
    this->shortPathStreak_ = 0;
  }

  // 可视化已移除
}

const bool &WayComputer::isLoopClosed() const {
  return this->isLoopClosed_;
}

void WayComputer::writeWayToFile(const std::string &file_path) const {
  std::ofstream oStreamToWrite(file_path);
  oStreamToWrite << this->wayToPublish_;
  oStreamToWrite.close();
}

const bool &WayComputer::isLocalTfValid() const {
  return this->localTfValid_;
}

const Eigen::Affine3d &WayComputer::getLocalTf() const {
  return this->localTf_;
}

std::vector<Point> WayComputer::getPath() const {
  return this->wayToPublish_.getPath();
}

Tracklimits WayComputer::getTracklimits() const {
  return this->wayToPublish_.getTracklimits();
}

autodrive_msgs::HUAT_PathLimits WayComputer::getPathLimits() const  {
  autodrive_msgs::HUAT_PathLimits res;
  res.stamp = this->lastStamp_;

  // res.replan indicates if the Way is different from last iteration's
  // 判断当前的way_是否与上一次迭代的lastWay_不同，如果不同则将res的replan成员变量设置为true，否则设置为false。
  res.replan = this->way_ != this->lastWay_;

  // Fill path
  std::vector<Point> path = this->wayToPublish_.getPathLocal();   //给的是全局的
  res.path.reserve(path.size());
  for (const Point &p : path) {
    res.path.push_back(p.gmPoint());
  }

  // Fill Tracklimits
  Tracklimits tracklimits = this->wayToPublish_.getTracklimits();
  res.tracklimits.stamp = res.stamp;
  res.tracklimits.left.reserve(tracklimits.first.size());
  for (const Node &n : tracklimits.first) {
    res.tracklimits.left.push_back(n.cone());
  }
  for (const Node &n : tracklimits.second) {
    res.tracklimits.right.push_back(n.cone());
  }

  // res.tracklimits.replan indicates if the n midpoints in front of the car
  // have varied from last iteration
  res.tracklimits.replan = this->way_.quinEhLobjetiuDeLaSevaDiresio(this->lastWay_);
  this->fillPathDynamics(res);
  return res;
}

/**
 * @brief 通过不同的x传出不同的路径
 * @param x 默认为0
 * @param x 0：获取全部的全局坐标系下的路径
 * @param x 1：对车到下一目标点中间进行插值
 * @param x 2：局部路径插值
 * @param x 3：全路径插值
 * @param x 4：局部路径插值(局部坐标)
 * @param x 5：全路径插值(局部坐标)
 * @param x 6：局部坐标系下的路径
*/
autodrive_msgs::HUAT_PathLimits WayComputer::getPathLimitsGlobal(int x)  {
  autodrive_msgs::HUAT_PathLimits res;
  res.stamp = this->currentStamp_;
  std::vector<Point> path;
  Point nextPoint;
  // res.replan indicates if the Way is different from last iteration's// res.replan指示方法是否与上次迭代不同
  res.replan = (this->way_ != this->lastWay_);
 
  // Fill path  //填充路径
  switch (x) {
    case 0://获取全部的全局坐标系下的路径
      path = this->wayToPublish_.getPath();
      res.path.reserve(path.size());
      for (const Point &p : path) {
        res.path.push_back(p.gmPoint());
      }
      break;
    case 1://对车到下一目标点中间进行插值
      nextPoint = this->wayToPublish_.getNextPathPoint();
      path = this->wayToPublish_.getPath();
      res.path.reserve(10);
      double diffX,diffY;
      diffX = (nextPoint.x - pose.position.x)/10.0;
      diffY = (nextPoint.y - pose.position.y)/10.0;
      for(int i = 0;i<10;i++){
        geometry_msgs::Point p;
        p.x = pose.position.x + (i + 1) * diffX;
        p.y = pose.position.y + (i + 1) * diffY;
        p.z = 0;
        res.path.push_back(p);
      }
      break;
    case 2://局部路径插值
    res.path = this->wayToPublish_.getPathInterpolation(pose.position.x,pose.position.y);
    break;
    case 3://全路径插值
      res.path = this->wayToPublish_.getPathFullInterpolation();
      break;
    case 4: // 局部路径插值(局部坐标)接口尚未实现，兼容回退到2
      ROS_WARN_THROTTLE(1.0, "[high_speed_tracking] mode=4 is not implemented. Fallback to mode=2.");
      res.path = this->wayToPublish_.getPathInterpolation(pose.position.x, pose.position.y);
      break;
    case 5: // 全路径插值(局部坐标)接口尚未实现，兼容回退到3
      ROS_WARN_THROTTLE(1.0, "[high_speed_tracking] mode=5 is not implemented. Fallback to mode=3.");
      res.path = this->wayToPublish_.getPathFullInterpolation();
      break;
    case 6:
      path = this->wayToPublish_.getPathLocal();
      res.path.reserve(path.size());
      for (const Point &p : path) {
        res.path.push_back(p.gmPoint());
      }
      break;
    default:
      ROS_WARN_THROTTLE(1.0, "[high_speed_tracking] Unknown path mode=%d. Fallback to mode=2.", x);
      res.path = this->wayToPublish_.getPathInterpolation(pose.position.x, pose.position.y);
      break;
  }

  
 

  // Fill Tracklimits
  Tracklimits tracklimits = this->wayToPublish_.getTracklimits();
  res.tracklimits.stamp = res.stamp;
  res.tracklimits.left.reserve(tracklimits.first.size());
  for (const Node &n : tracklimits.first) {
    res.tracklimits.left.push_back(n.cone());
  }
  for (const Node &n : tracklimits.second) {
    res.tracklimits.right.push_back(n.cone());
  }

  // res.tracklimits.replan indicates if the n midpoints in front of the car
  // have varied from last iteration
  res.tracklimits.replan = this->way_.quinEhLobjetiuDeLaSevaDiresio(this->lastWay_);
  this->fillPathDynamics(res);
  return res;
}


autodrive_msgs::HUAT_CarState WayComputer::getCarState(){
  return CarState;
}

size_t WayComputer::lastTriangleCount() const {
  return last_triangle_count_;
}

size_t WayComputer::lastEdgeCount() const {
  return last_edge_count_;
}

const TriangleSet &WayComputer::lastFilteredTriangulation() const {
  return last_filtered_triangulation_;
}

const EdgeSet &WayComputer::lastFilteredEdges() const {
  return last_filtered_edges_;
}

const Way &WayComputer::wayForVisualization() const {
  return wayToPublish_;
}
