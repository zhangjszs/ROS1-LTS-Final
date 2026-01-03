/**
 * @file Way.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Way class member functions implementation
 * @version 1.0
 * @date 2022-20-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Way.hpp"
double limit = 400.0;
double post_group_x[20];
double post_group_y[20];
/* ----------------------------- Private Methods ---------------------------- */

Params::WayComputer::Way Way::params_;

//  它的功能是更新`way`对象中与车辆最接近的元素的位置
void Way::updateClosestToCarElem()
{
  auto smallestDWFIt = this->path_.cbegin(); // 将`smallestDWFIt`迭代器指向路径中的第一个元素位置
  if (!this->path_.empty())
  {
    // 并将其设置为路径中第一个元素中点到（0，0）也就是定义的车辆原点的平方。
    double smallestDWF = Point::distSq(this->path_.front().midPoint());

    // Find closest element to way's first in this
    // 实际就是计算每一个点跟原点之间的平方  然后跟第一个点与原点的距离相比  遍历一遍取最小的
    // 如果该距离平方小于或等于`smallestDWF`，则更新`smallestDWF`为该距离平方，
    // 同时更新`smallestDWFIt`为当前迭代器`it`。
    for (auto it = this->path_.cbegin(); it != this->path_.cend(); it++)
    {
      Point point = it->midPoint();
      double distSqWithFirst = Point::distSq(it->midPoint());
      if (distSqWithFirst <= smallestDWF)
      {
        if (point.x > 0)
        {
          smallestDWF = distSqWithFirst;
          smallestDWFIt = it;
        }
      }
    }
  }
  // ，将`smallestDWFIt`赋值给`closestToCarElem_`成员变量，表示与车辆最近的元素在路径中的位置
  this->closestToCarElem_ = smallestDWFIt;
}

bool Way::segmentsIntersect(const Point &A, const Point &B, const Point &C, const Point &D)
{
  return Point::ccw(A, C, D) != Point::ccw(B, C, D) and Point::ccw(A, B, C) != Point::ccw(A, B, D);
}

/* ----------------------------- Public Methods ----------------------------- */

void Way::init(const Params::WayComputer::Way &params)
{
  params_ = params;
}

Way::Way()
    : avgEdgeLen_(0.0)
{
  closestToCarElem_ = this->path_.cend(); // 返回的是指向 `path_` 列表的尾部的常量迭代器。这个迭代器表示列表的结束位置，用于表示迭代范围的终点。它指向的是最后一个元素之后的位置，因此不指向任何有效的元素。
  sizeToCar_ = 0;
}

bool Way::empty() const
{
  return this->path_.empty();
}

size_t Way::size() const
{
  return this->path_.size();
}

const Edge &Way::back() const
{
  return this->path_.back();
}

const Edge &Way::beforeBack() const
{
  ROS_ASSERT(this->size() >= 2);
  return *(++this->path_.rbegin());
}

const Edge &Way::front() const
{
  return this->path_.front();
}

void Way::updateLocal(const Eigen::Affine3d &tf)
{
  for (Edge &e : this->path_)
  {
    e.updateLocal(tf);
  }
  // Update closest
  this->updateClosestToCarElem();
}

void Way::addEdge(const Edge &edge)
{
  this->path_.push_back(edge);
  if (this->path_.size() == 1)
    closestToCarElem_ = this->path_.cbegin();
  this->avgEdgeLen_ += (edge.len - this->avgEdgeLen_) / this->size();
}

void Way::trimByLocal()
{
  if (this->size() < 2)
    return;
  // 它将最接近车辆的边存储在变量`closestToCarElem`中。
  auto closestToCarElem = this->closestToCarElem_; // way如果无数据，则构造函数写到了结束位置  指向距离车最近位置的边

  if (closestToCarElem != std::prev(this->path_.cend()))
  {
    // Erase all elements after closest element (included)
    // 如果`closestToCarElem`不是`path_`中的最后一个元素，则删除`closestToCarElem`之后的所有元素，
    // 包括`closestToCarElem`本身。这将删除距离车辆较远的边。
    // 删除 `path_` 中最接近 `closestToCarElem` 的元素之后（包括最接近的元素本身），也就是删除靠后部分的元素。
    this->path_.erase(++closestToCarElem, this->path_.cend());
  }

  // Recalculate the mean edge length attribute
  // 重新计算平均边长属性（`avgEdgeLen_`）的。代码的逻辑如下
  // 其中 `it->len` 是当前迭代元素的边长。这个公式可以理解为每次新增的边长与当前平均边长的偏差除以迭代次数 `t`。
  // 将这个偏差累加到当前平均边长上，得到新的平均边长。
  size_t t = 1;
  this->avgEdgeLen_ = 0.0;
  for (auto it = this->path_.cbegin(); it != this->path_.cend(); it++)
  {
    this->avgEdgeLen_ += (it->len - this->avgEdgeLen_) / t;
    t++;
  }

  // Recalculate sizeToCar_ attribute计算 `sizeToCar_` 属性的值
  this->sizeToCar_ = this->size();
}

bool Way::closesLoop() const
{
  return this->size() >= MIN_LOOP_SIZE and Point::distSq(this->front().midPoint(), this->back().midPoint()) <= params_.max_dist_loop_closure * params_.max_dist_loop_closure;
}

// 闭环检查
//. 路径的长度（this->size()）加1是否大于等于预定义的最小闭环长度（MIN_LOOP_SIZE）。
//  b. 路径中第一个点的中间点（this->front().midPoint()）与给定边的中间点（e.midPoint()）
// 之间的距离的平方是否小于等于"params_.max_dist_loop_closure"的平方（即距离条件是否满足）。
// c. 使用路径中的第一个点的中间点和路径中的第二个点的中间点，构造两个向量，
// 并计算这两个向量之间的夹角的绝对值（即角度差）。
// 然后，检查角度差是否小于等于"params_.max_angle_diff_loop_closure"（即角度条件是否满足）。
// 20. 如果以上所有条件都满足，则返回true，否则返回false。
bool Way::closesLoopWith(const Edge &e, const Point *lastPosInTrace) const
{
  Point actPos = lastPosInTrace ? *lastPosInTrace : this->back().midPoint();
  return
      // Must have a minimum size
      this->size() + 1 >= MIN_LOOP_SIZE and
      // Distance conditions are met
      Point::distSq(this->front().midPoint(), e.midPoint()) <= params_.max_dist_loop_closure * params_.max_dist_loop_closure and
      // Check closure angle with first point
      abs(Vector(this->front().midPoint(), (++this->path_.begin())->midPoint()).angleWith(Vector(actPos, e.midPoint()))) <= params_.max_angle_diff_loop_closure;
}

Way Way::restructureClosure() const
{
  Way res = *this;
  if (res.front() != res.back())
  {
    // Assume last Edge is the one that closes the loop
    double distClosestWithLast = Point::distSq(res.front().midPoint(), res.back().midPoint());
    auto closestWithLastIt = res.path_.begin();

    for (auto it = res.path_.begin(); it != std::prev(res.path_.end(), 20); it++)
    { // The 20 is for safety
      double distWithLast = Point::distSq(res.back().midPoint(), it->midPoint());
      if (distWithLast <= distClosestWithLast)
      {
        distClosestWithLast = distWithLast;
        closestWithLastIt = it;
      }
    }

    // We remove all edges that would cause our "loop" not to be a loop
    // (all that are before the edge that closes the loop)
    res.path_.erase(res.path_.begin(), closestWithLastIt);
  }
  // Here last midpoint (now the closest to the first one) will be replaced
  // by the first one. Making sure that they are EXACTLY the same (id and value).
  if (res.front() == res.back() or
      Vector::pointBehind(res.back().midPointGlobal(), res.front().midPointGlobal(), Vector(res.beforeBack().midPointGlobal(), res.back().midPointGlobal())) or
      Point::dist(res.back().midPointGlobal(), res.front().midPointGlobal()) < SAME_MIDPOINT_DIST_THRESHOLD)
  {
    res.path_.pop_back();
  }
  res.path_.push_back(res.front());

  return res;
}

bool Way::intersectsWith(const Edge &e) const
{
  if (this->size() <= 2)
    return false;
  Point s1p1, s1p2;
  const Point s2p1 = this->back().midPoint();
  const Point s2p2 = e.midPoint();
  auto it1 = std::next(this->path_.crbegin());
  auto it2 = std::next(it1);
  while (it2 != this->path_.crend())
  {
    s1p1 = it1->midPoint();
    s1p2 = it2->midPoint();
    if (this->segmentsIntersect(s1p1, s1p2, s2p1, s2p2))
      return true;
    it1++;
    it2++;
  }
  return false;
}

bool Way::containsEdge(const Edge &e) const
{
  for (const Edge &edge : this->path_)
  {
    if (e == edge)
      return true;
  }
  return false;
}

std::vector<Point> Way::getPath() const
{
  std::vector<Point> res;
  res.reserve(this->path_.size());
  for (const Edge &e : this->path_)
  {
    res.push_back(e.midPointGlobal());
  }
  return res;
}

std::vector<Point> Way::getPathLocal() const
{
  std::vector<Point> res;
  res.reserve(this->path_.size());
  for (const Edge &e : this->path_)
  {
    res.push_back(e.midPoint());
  }
  return res;
}

Tracklimits Way::getTracklimits() const
{
  Tracklimits res;
  res.first.reserve(this->size());
  res.second.reserve(this->size());
  Point pAnt = this->empty() ? Point(0, 0) : Point(-50, this->front().midPointGlobal().y); // This only works in a global, the -50 is arbitrary

  const Node *left, *firstLeft;
  const Node *right, *firstRight;

  size_t edgeInd = 0;
  for (const Edge &e : this->path_)
  {
    // if (this->closesLoop() and edgeInd == this->size() - 1) break;  // Last Edge is (can be) same as first => nothing to append
    Vector pAntPAct(pAnt, e.midPointGlobal());

    // Check the side of both Nodes of the Edge
    if (Vector::pointBehind(e.n0.pointGlobal(), e.midPointGlobal(), pAntPAct.rotClock()))
    {
      left = &e.n0;
      right = &e.n1;
    }
    else
    {
      left = &e.n1;
      right = &e.n0;
    }

    // Save first Nodes
    if (edgeInd == 0)
    {
      firstLeft = left;
      firstRight = right;
    }

    // Only append those Nodes that:
    // - have not been appended before
    // - have a position behind the last one
    // - [only for the 3 lasts (if loop closure)] have a position in front of the first one
    if (res.first.empty() or
        (*left != res.first.back() and
         !Vector::pointBehind(left->pointGlobal(), res.first.back().pointGlobal(), pAntPAct) and
         (!this->closesLoop() or edgeInd < this->size() - 3 or !Vector::pointBehind(res.first.front().pointGlobal(), left->pointGlobal(), pAntPAct))))
    {
      if (*left == *firstLeft)
      {
        res.first.push_back(*firstLeft);
      }
      else
        res.first.push_back(*left);
    }
    if (res.second.empty() or
        (*right != res.second.back() and
         !Vector::pointBehind(right->pointGlobal(), res.second.back().pointGlobal(), pAntPAct) and
         (!this->closesLoop() or edgeInd < this->size() - 3 or !Vector::pointBehind(res.second.front().pointGlobal(), right->pointGlobal(), pAntPAct))))
    {
      if (*right == *firstRight)
      {
        res.second.push_back(*firstRight);
      }
      else
        res.second.push_back(*right);
    }

    pAnt = e.midPointGlobal();
    edgeInd++;
  }
  return res;
}

Way &Way::operator=(const Way &way)
{
  this->path_ = std::list<Edge>(way.path_);
  this->avgEdgeLen_ = way.avgEdgeLen_;
  this->sizeToCar_ = way.sizeToCar_;
  this->updateClosestToCarElem(); // A copy of the attribute would be unsafe
  return *this;
}

bool Way::operator==(const Way &way) const
{
  if (this->size() != way.size())
    return false;
  auto thisIt = this->path_.cbegin();
  auto paramIt = way.path_.cbegin();

  while (thisIt != this->path_.cend())
  {
    if (*thisIt != *paramIt)
      return false;
    thisIt++;
    paramIt++;
  }

  return true;
}

bool Way::operator!=(const Way &way) const
{
  return not(*this == way);
}

bool Way::quinEhLobjetiuDeLaSevaDiresio(const Way &way) const
{
  // Replan every time the emptiness changes
  if (this->empty() != way.empty())
    return true;
  // Find the common starting point (replan if not found)
  auto wayIt = way.closestToCarElem_;
  auto thisIt = this->closestToCarElem_;

  while (wayIt != way.path_.cend())
  {
    if (*wayIt == *thisIt)
    {
      // Common starting point found!
      break;
    }
    wayIt++;
  }

  // Case no starting point has been found
  if (*wayIt != *thisIt)
    return true;

  // Compare one by one the vital_num_midpoints
  int midpoint_num = 0;
  while (wayIt != way.path_.cend() and thisIt != this->path_.cend())
  {
    if (midpoint_num >= this->params_.vital_num_midpoints)
      return false;
    if (*wayIt != *thisIt)
      return true;
    midpoint_num++;
    wayIt++;
    thisIt++;
  }

  // Case where we reach the end of one of the two Way(s), we could check if
  // one is longer than the other (within these vital_num_midpoints) and return
  // accordingly, but this case is better covered by the planner itself (it will
  // replan when it runs out of midpoints).
  return false;
}

const double &Way::getAvgEdgeLen() const
{
  return this->avgEdgeLen_;
}

uint32_t Way::sizeAheadOfCar() const
{
  return this->path_.size() - this->sizeToCar_;
}

std::ostream &operator<<(std::ostream &os, const Way &way)
{
  Tracklimits tracklimits = way.getTracklimits();
  for (const Node &n : tracklimits.first)
  {
    os << n.pointGlobal().x << ' ' << n.pointGlobal().y << ' ' << 0 << ' ' << n.id << std::endl;
  }
  for (const Node &n : tracklimits.second)
  {
    os << n.pointGlobal().x << ' ' << n.pointGlobal().y << ' ' << 1 << ' ' << n.id << std::endl;
  }
  return os;
}

void Way::deleteWayPassed()
{
  if (this->size() < 2)
    return;
  auto closestToCarElem = this->closestToCarElem_;

  if ((closestToCarElem != std::prev(this->path_.cbegin())))
  {
    this->path_.erase(this->path_.cbegin(), closestToCarElem);
  }
}

Point Way::getNextPathPoint()
{
  auto closestToCarElem = this->closestToCarElem_;
  Point point;
  point.x = closestToCarElem->midPointGlobal().x;
  point.y = closestToCarElem->midPointGlobal().y;
  return point;
}

std::vector<geometry_msgs::Point> Way::getPathInterpolation(double x, double y)
{
  deleteWayPassed();
  std::vector<geometry_msgs::Point> res;
  auto it = this->path_.cbegin();
  geometry_msgs::Point p;
  int size = path_.size();
  auto lastIt = it;
  it++;
  //对车的位置到前面两个点的的拟合(进行判断,距离钱前面的第一个点的距离过近就会规划出一个虚拟点来规划曲线)
  if (pow(it->midPointGlobal().x - x, 2) + pow(it->midPointGlobal().y - y, 2) < pow(it->midPointGlobal().x - lastIt->midPointGlobal().x, 2) + pow(it->midPointGlobal().y - lastIt->midPointGlobal().y, 2) / 4)
  {
    size++;
    // 这是虚拟点的x,y值画出来就是车到第二个点的中点与第一个点到第二点中点的连线的中点
    // 其代替车前的第一个点,其相对与直接利用第一个点来说更方便拟合而且防止折角过大的出现
    double helpmidass_x = ((it->midPointGlobal().x + x) / 2 + (it->midPointGlobal().x + lastIt->midPointGlobal().x) / 2) / 2;
    double helpmidass_y = ((it->midPointGlobal().y + y) / 2 + (it->midPointGlobal().y + lastIt->midPointGlobal().y) / 2) / 2;
    Eigen::MatrixXd A(3, 3); // 使用Eigen库创建一个3x3的矩阵A，用于构建线性方程组
    Eigen::VectorXd b(3);    // 创建一个3维向量b，作为线性方程组的右侧
    // 下面也都是对选择使用在xoy或者使用yox的判断
    bool start_x = false, start_y = false, turn_y = false, bend = true, turn_y2 = false, use_x = true;
    if (x > helpmidass_x && helpmidass_x > it->midPointGlobal().x || x < helpmidass_x && helpmidass_x < it->midPointGlobal().x)
      start_x = true;
    if (y < helpmidass_y && helpmidass_y < it->midPointGlobal().y || y > helpmidass_y && helpmidass_y > it->midPointGlobal().y)
      start_y = true;
    if (abs(it->midPointGlobal().x - helpmidass_x) * 2 < abs(it->midPointGlobal().y - helpmidass_y))
      turn_y = true;
    if (abs(helpmidass_x - x) > abs(helpmidass_y - y))
      turn_y = false;
    if (abs(it->midPoint().y - helpmidass_y) < 1 || abs(helpmidass_y - y) < 1)
      turn_y2 = true;
    // 需要提前对需要利用的点的信息赋值,不能在if-else内进行赋值
    double A_0 = 0;
    double A_1 = 0;
    double A_2 = 0;
    double b_0 = 0;
    double b_1 = 0;
    double b_2 = 0;
    // 判定,选择什么样的坐标系拟合,对需要的点的信息区别赋值
    if ((start_x) && !(start_y && turn_y) || turn_y2)
    {
      A_0 = x;
      A_1 = helpmidass_x;
      A_2 = it->midPointGlobal().x;
      b_0 = y;
      b_1 = helpmidass_y;
      b_2 = it->midPointGlobal().y;
    }
    else
    {
      A_0 = y;
      A_1 = helpmidass_y;
      A_2 = it->midPointGlobal().y;
      b_0 = x;
      b_1 = helpmidass_x;
      b_2 = it->midPointGlobal().x;
      use_x = false;
    }
    // 对之前的矩阵的信息赋值
    b(0) = b_0;
    b(1) = b_1;
    b(2) = b_2;
    for (int m = 0; m < 3; m++)
    {
      A(0, m) = pow(A_0, m);
      A(1, m) = pow(A_1, m);
      A(2, m) = pow(A_2, m);
    }
    // 拟合一个简单的3项式
    Eigen::VectorXd yn = A.colPivHouseholderQr().solve(b); // 构建线性方程组的系数矩阵A，这里使用x的幂次来构建，是为了拟合一个多项式
    double A_d = (A_2 - A_0) / 30.0;                       // 两点之间分为30份，以方便对曲线拟合数据的输出
                                                           // 从车到虚拟点分段,满足控制的需要,也是展现曲线的拟合(使用之前得到的拟合多项式信息)
    for (int k = 0; k < 30; ++k)
    {
      double Point_A = A_0 + k * A_d;
      double Point_b = 0;
      for (int m = 0; m < yn.size(); ++m)
      {
        Point_b += yn[m] * pow(Point_A, m);
      }
      // 确定最后输出的x与y
      if (use_x)
      {
        p.x = Point_A;
        p.y = Point_b;
      }
      else
      {
        p.x = Point_b;
        p.y = Point_A;
      }
      res.push_back(p);
    }
  }
  else
  {
    // 其实可以把全部的都搞成上面到处理形式,但是说的是没必要的话用这个的会更简单直接
    it--;
    double diffX = (it->midPointGlobal().x - x) / 20.0;
    double diffY = (it->midPointGlobal().y - y) / 20.0;

    for (int i = 0; i < 20; i++)
    {
      p.x = x + diffX * i;
      p.y = y + diffY * i;
      res.push_back(p);
    }
   }
  lastIt = it;
  auto change = it;
  it++;
  // 判断识别到的点的数量能不能给我用来拟合
  if (size > 2)
  {
    // 循环多次进行拟合,从车前的点开始,每三个点构造一个多项式用来给与车近的两个点之间用
    for (int i = 0; i < size - 2; i++)
    {

      bool increasing_x = true, decreasing_x = true, increasing_y = true, decreasing_y = true, turn_y = false, bend = true, turn_y2 = false;
      // 这个地方的都是对选择xoy或者选择yox的bool类型的初始化
      // 其中bend是对弯曲程度的判定，是对一些比较离谱的拟合曲线的删除
      for (int j = 0; j < 3; ++j)
      {
        for (size_t j = 0; j < 3; ++j)
        {
          // 这个地方的都是对选择xoy或者选择yox进行拟合的判断
          if (it->midPointGlobal().x < lastIt->midPointGlobal().x)
            increasing_x = false;
          if (it->midPointGlobal().x > lastIt->midPointGlobal().x)
            decreasing_x = false;
          if (it->midPointGlobal().y < lastIt->midPointGlobal().y)
            increasing_y = false;
          if (it->midPointGlobal().y > lastIt->midPointGlobal().y)
            decreasing_y = false;
          if (abs(it->midPointGlobal().x - lastIt->midPointGlobal().x) * 2 < abs(it->midPointGlobal().y - lastIt->midPointGlobal().y))
            turn_y = true;
          if (abs(it->midPointGlobal().x - lastIt->midPointGlobal().x) > abs(it->midPointGlobal().y - lastIt->midPointGlobal().y))
            turn_y = false;
          if (abs(it->midPoint().y - lastIt->midPoint().y) < 1)
            turn_y2 = true;
          if (j == 1 && size == 3)
            break;
          lastIt++;
          it++;
        }
        lastIt = change; // 其中bend是对弯曲程度的判定，是对一些比较离谱的拟合曲线的删除
        it = lastIt;
        it++;

        if ((increasing_x && !decreasing_x || !increasing_x && decreasing_x) && !((increasing_y && !decreasing_y || !increasing_y && decreasing_y) && turn_y) || turn_y2)
        {
          Eigen::MatrixXd A(3, 3);
          Eigen::VectorXd b(3);
          int n = 0;
          // 得到轨迹上最短两点的距离,最长的肯定用不了,还是用最短的距离合理
          limit = std::min((pow(lastIt->midPointGlobal().x - it->midPointGlobal().x, 2) + pow(lastIt->midPointGlobal().y - it->midPointGlobal().y, 2)), limit);
          for (int j = i; j <= i + 2; j++)
          {
            b(n) = lastIt->midPointGlobal().y;
            for (int m = 0; m < 3; m++)
              A(n, m) = pow(lastIt->midPointGlobal().x, m);
            n++;
            lastIt++;
          }
          Eigen::VectorXd yn = A.colPivHouseholderQr().solve(b); // 拟合多项式
          lastIt = change;
          double x0 = lastIt->midPointGlobal().x; // 第一个点
          double x1 = it->midPointGlobal().x;     // 第二个点
          double dx = (x1 - x0) / 20.0;           // 分段的每两个点的x的差(这个是xoy的曲线拟合)
          double ynext;
          // 第一个点与第二个点之间插值
          for (int k = 0; k < 20; ++k)
          {
            p.x = x0 + k * dx;
            p.y = 0;
            for (int m = 0; m < yn.size(); ++m)
            {
              p.y += yn[m] * pow(p.x, m);
            }
            // 判断两点还有拟合出来的东西的合理性,不合理就断开
            if (k > 0 && (pow(p.y - ynext, 2) + pow(dx, 2) > limit / 50))
            {
              bend = false;
            }
            ynext = p.y;
          }
          if (!bend){ // 不合理
            break;}
          // 合理就正常插值
          for (int k = 0; k < 20; ++k)
          {
            p.x = x0 + k * dx;
            p.y = 0;
            for (int m = 0; m < yn.size(); ++m)
            {
              p.y += yn[m] * pow(p.x, m);
            }
            if(i>0&&(pow(p.y - p.y + post_group_y[k], 2) + pow(dx, 2) < limit / 100)){
            p.x = (p.x + post_group_x[k]) / 2;
            p.y = (p.y + post_group_y[k]) / 2;}
            res.push_back(p);
          }
          it++;
          double x2 = it->midPointGlobal().x;
          for (int k = 0; k < 20; ++k)
          {
            p.x = x1 + k * (x2 - x1);
            p.y = 0;
            for (int m = 0; m < yn.size(); ++m)
            {
              p.y += yn[m] * pow(p.x, m);
            }
            if (i == size - 3&& (pow(p.y - post_group_y[k], 2) + pow(dx, 2) < limit / 50))
            {
              res.push_back(p);
            }
           post_group_x[k] = p.x;
           post_group_y[k] = p.y;
          }
        }
        else
        {
          // 这个都跟上面的一样,就是用的在yox上的拟合
          Eigen::MatrixXd A(3, 3);
          Eigen::VectorXd b(3);
          int n = 0;
          for (int j = 0; j < 3; ++j)
          {
            b(n) = lastIt->midPointGlobal().x;
            for (int m = 0; m < 3; ++m)
              A(n, m) = pow(lastIt->midPointGlobal().y, m);
            n++;
            lastIt++;
          }
          Eigen::VectorXd yn = A.colPivHouseholderQr().solve(b);
          lastIt = change;
          double y0 = lastIt->midPointGlobal().y;
          double y1 = it->midPointGlobal().y;
          double dy = (y1 - y0) / 20.0; // 计算每个插入点之间的y增量
          double xnext;
          for (int k = 0; k < 20; ++k)
          {
            p.y = y0 + k * dy;
            p.x = 0;
            for (int m = 0; m < yn.size(); ++m)
            {
              p.x += yn[m] * pow(p.y, m);
            }
            if (k > 0 && (pow(p.x - xnext, 2) + pow(dy, 2) > limit / 50))
            {
              bend = false;
            }
            xnext = p.x;
          }
          if (!bend)
            break;
          for (int k = 0; k < 20; ++k) // 包括原始的两个点
          {
            p.y = y0 + k * dy;
            p.x = 0; // 重置p.y
            for (int m = 0; m < yn.size(); ++m)
            {
              p.x += yn[m] * pow(p.y, m);
            }
             if(i>0 &&(pow(p.x - post_group_x[k], 2) + pow(dy, 2) < limit / 100)){
            p.x = (p.x + post_group_x[k]) / 2;
            p.y = (p.y + post_group_y[k]) / 2;}
            res.push_back(p);
          }
           it++;
          double y2 = it->midPointGlobal().y;
          for (int k = 0; k < 20; ++k)
          {
            p.y = y1 + k * (y2 - y1);
            p.x = 0;
            for (int m = 0; m < yn.size(); ++m)
            {
              p.x += yn[m] * pow(p.y, m);
            }
            if (i == size - 3&&(pow(p.x - post_group_x[k], 2) + pow(dy, 2) < limit / 50))
            {
              res.push_back(p);
            }
           post_group_x[k] = p.x;
           post_group_y[k] = p.y;
          }
        }
        if (!bend)
          break;
        change++;
        lastIt = change;
        it = lastIt;
        it++;
      }
      return res;
    }
    return res;
  }
}

std::vector<geometry_msgs::Point> Way::getPathFullInterpolation()
{
  std::vector<geometry_msgs::Point> res;
  auto it = this->path_.cbegin();
  auto lastIt = it;
  auto change = it;
  geometry_msgs::Point p;
  it++;
  if (path_.size() > 3)
  {
    for (int i = 0; i < path_.size() - 2; i++)
    {

      bool increasing_x = true, decreasing_x = true, increasing_y = true, decreasing_y = true, turn_y = false;

      for (size_t j = 0; j < 3; ++j)
      {
        if (it->midPointGlobal().x < lastIt->midPointGlobal().x)
          increasing_x = false;
        if (it->midPointGlobal().x > lastIt->midPointGlobal().x)
          decreasing_x = false;
        if (it->midPointGlobal().y < lastIt->midPointGlobal().y)
          increasing_y = false;
        if (it->midPointGlobal().y > lastIt->midPointGlobal().y)
          decreasing_y = false;
        if (abs(it->midPointGlobal().x - lastIt->midPointGlobal().x) * 2 < abs(it->midPointGlobal().y - lastIt->midPointGlobal().y))
          turn_y = true;
        if (abs(it->midPointGlobal().x - lastIt->midPointGlobal().x) > abs(it->midPointGlobal().y - lastIt->midPointGlobal().y))
          turn_y = false;
        lastIt++;
        it++;
      }
      lastIt = change;
      it = lastIt;
      it++;

      if ((increasing_x && !decreasing_x || !increasing_x && decreasing_x) && !((increasing_y && !decreasing_y || !increasing_y && decreasing_y) && turn_y))
      {
        Eigen::MatrixXd A(3, 3);
        Eigen::VectorXd b(3);
        int n = 0;
        for (int j = i; j <= i + 2; j++)
        {
          b(n) = lastIt->midPointGlobal().y;
          for (int m = 0; m < 3; m++)
            A(n, m) = pow(lastIt->midPointGlobal().x, m);
          n++;
          lastIt++;
        }
        Eigen::VectorXd yn = A.colPivHouseholderQr().solve(b);
        lastIt = change;
        double x0 = lastIt->midPointGlobal().x;
        double x1 = it->midPointGlobal().x;
        double dx = (x1 - x0) / 20.0;
          for (int k = 0; k < 20; ++k)
          {
            p.x = x0 + k * dx;
            p.y = 0;
            for (int m = 0; m < yn.size(); ++m)
            {
              p.y += yn[m] * pow(p.x, m);
            }
            if(i>0){
            p.x = (p.x + post_group_x[k]) / 2;
            p.y = (p.y + post_group_y[k]) / 2;}
            res.push_back(p);
          }
          it++;
          double x2 = it->midPointGlobal().x;
          for (int k = 0; k < 20; ++k)
          {
            p.x = x1 + k * (x2 - x1);
            p.y = 0;
            for (int m = 0; m < yn.size(); ++m)
            {
              p.y += yn[m] * pow(p.x, m);
            }
            if (i == path_.size()  - 3)
            {
              res.push_back(p);
            }
           post_group_x[k] = p.x;
           post_group_y[k] = p.y;
          }
      }
      else
      {
        Eigen::MatrixXd A(3, 3);
        Eigen::VectorXd b(3);
        int n = 0;
        for (int j = 0; j < 3; ++j)
        {
          b(n) = lastIt->midPointGlobal().x;
          for (int m = 0; m < 3; ++m)
            A(n, m) = pow(lastIt->midPointGlobal().y, m);
          n++;
          lastIt++;
        }
        Eigen::VectorXd yn = A.colPivHouseholderQr().solve(b);
        lastIt = change;
        double y0 = lastIt->midPointGlobal().y;
        double y1 = it->midPointGlobal().y;
        double dy = (y1 - y0) / 20.0; // 计算每个插入点之间的y增量
        for (int k = 0; k < 20; ++k)  // 包括原始的两个点
        {
          p.y = y0 + k * dy;
          p.x = 0; // 重置p.y
          for (int m = 0; m < yn.size(); ++m)
          {
            p.x += yn[m] * pow(p.y, m);
          }
             if(i>0){
            p.x = (p.x + post_group_x[k]) / 2;
            p.y = (p.y + post_group_y[k]) / 2;}
          res.push_back(p);
        }
          it++;
          double y2 = it->midPointGlobal().y;
          for (int k = 0; k < 20; ++k)
          {
            p.y = y1 + k * (y2 - y1);
            p.x = 0;
            for (int m = 0; m < yn.size(); ++m)
            {
              p.x += yn[m] * pow(p.y, m);
            }
            if (i == path_.size()  - 3)
            {
              res.push_back(p);
            }
           post_group_x[k] = p.x;
           post_group_y[k] = p.y;
          }
      }
      change++;
      lastIt = change;
      it = lastIt;
      it++;
    }
    return res;
  }
  return res;
}