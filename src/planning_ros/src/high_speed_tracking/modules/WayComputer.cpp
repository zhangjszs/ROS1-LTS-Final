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
      this->isLoopClosed_ = true;
      return;
    }
  //两次调用`findNextEdges`函数的目的是确保在每次循环迭代中，`nextEdges`向量都包含了当前路径状态下的可能边。通过在每个循环迭代中更新`nextEdges`向量，
  //可以确保路径搜索是基于最新状态的，并继续寻找最佳路径，直到满足循环条件终止循环执行。
    // Get next set of possible edges
    this->findNextEdges(nextEdges, nullptr, midpointsKDT, edges, params);
  }
  this->isLoopClosed_ = false;
  this->wayToPublish_ = this->way_;
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

  // #5: Perform the search through the midpoints in order to obtain a way
  //     using normal parameters.
  // 是使用中点进行搜索，以便获取一条路径 (`way`)，使用正常的参数进行搜索。
  // 具体来说，这段代码执行了通过中点进行搜索以获取路径的操作。
  // 这里面的边已经是过滤后的边  通过这里边的中点来进行规划中线。
  this->computeWay(edgeVec, this->params_.search);

  // #6: Check failsafe(s)
  //总的来说，这段代码的目的是检查是否需要启用一般失败安全机制，即当前路径长度不足且没有闭环时预先计算一条新的路径，以保证安全性。
  //如果满足条件，则输出警告信息并调用 `computeWay()` 方法生成新路径。
  if (this->params_.general_failsafe and this->way_.sizeAheadOfCar() < MIN_FAILSAFE_WAY_SIZE and !this->isLoopClosed_) {
    ROS_WARN("[high_speed_tracking] GENERAL FAILSAFE ACTIVATED!");
    this->computeWay(edgeVec, this->generalFailsafe_);
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
