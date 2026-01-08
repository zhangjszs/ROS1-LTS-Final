/**
 * @file Node.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Node class member functions implementation
 * @version 1.0
 * @date 2022-10-31
 *
 * @copyright Copyright (c) 2022 BCN eMotorsport
 */

#include "structures/Node.hpp"

const uint32_t Node::SUPERTRIANGLE_BASEID;
uint32_t Node::superTriangleNodeNum = 0;

/* ----------------------------- Private Methods ---------------------------- */

Node::Node(const double &x, const double &y)
    : point_(x, y), id(SUPERTRIANGLE_BASEID + superTriangleNodeNum), belongsToSuperTriangle_(true) {
  superTriangleNodeNum++;
  superTriangleNodeNum %= 3;
}

/* ----------------------------- Public Methods ----------------------------- */

Node::Node(const double &x, const double &y, const double &xGlobal, const double &yGlobal, const uint32_t &id)
    : point_(x, y), pointGlobal_(xGlobal, yGlobal), id(id), belongsToSuperTriangle_(false) {
  if (this->id >= (1 << HASH_SHIFT_NUM) - 3) ROS_ERROR("[high_speed_tracking] Cone ID is above the allowed threshold, see utils/constants.hpp/HASH_SHIFT_NUM");
}

Node::Node(const common_msgs::HUAT_cone &c)
    : Node(c.position_baseLink.x, c.position_baseLink.y, c.position_global.x, c.position_global.y, c.id) {}

const double &Node::x() const {
  return this->point_.x;
}

const double &Node::y() const {
  return this->point_.y;
}

bool Node::operator==(const Node &n) const {
  return n.id == this->id;
}

bool Node::operator!=(const Node &n) const {
  return not(*this == n);
}

Node Node::superTriangleNode(const double &x, const double &y) {
  return Node(x, y);
}

const bool &Node::belongsToSuperTriangle() const {
  return belongsToSuperTriangle_;
}

void Node::updateLocal(const Eigen::Affine3d &tf) const {
  this->point_ = this->pointGlobal().transformed(tf);
}

const Point &Node::point() const {
  return this->point_;
}

const Point &Node::pointGlobal() const {              //构造函数可直接赋值
  return this->pointGlobal_;
}

double Node::distSq(const Point &p) const {
  return (this->x() - p.x) * (this->x() - p.x) + (this->y() - p.y) * (this->y() - p.y);
}

double Node::angleWith(const Node &n0, const Node &n1) const {
  return abs(Vector(this->point(), n0.point()).angleWith(Vector(this->point(), n1.point())));
}

common_msgs::HUAT_cone Node::cone() const {
  common_msgs::HUAT_cone res;
  res.id = this->id;
  res.position_global.x = this->pointGlobal().gmPoint().x;
  res.position_global.y = this->pointGlobal().gmPoint().y;
  res.position_baseLink.x = this->point().gmPoint().x;
  res.position_baseLink.y = this->point().gmPoint().y;

  res.type = 4;  // None
  return res;
}

std::ostream &operator<<(std::ostream &os, const Node &n) {
  os << "N(" << n.x() << ", " << n.y() << ")";
  return os;
}