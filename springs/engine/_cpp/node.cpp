#include <iostream>
#include <cmath>
#include <sys/types.h>

#include "node.h"
#include "link.h"


// bound on how much a node position can change from one update to another
#define max_translation          2.0
#define max_translation_squared (max_translation * max_translation)


namespace springs {

  Node::Node() {}
  Node::Node(double dt, double x, double y, double mass, double friction, bool fixed)
    : dt(dt), x(x), y(y), v_x(0), v_y(0), friction(friction), colliding(false), _fixed(fixed) {
    set_mass(mass);
    _dt_sq = dt * dt;
  }

  double Node::mass() {
    return _mass;
  }

  void Node::set_mass(const double mass) {
    _mass     = mass;
    _inv_mass = _fixed ? 0.0 : 1.0 / _mass;
    for (auto& link: links) {
      link->_update();
    }
  }

  double Node::inv_mass() { return _inv_mass; }

  bool Node::fixed() { return _fixed; }

  void Node::set_fixed(const bool fixed) {
    if (_fixed != fixed) {
      _inv_mass = fixed ? 0.0 : 1.0 / _mass;
    }
    _fixed = fixed;
  }

  inline void Node::update_position() {
    x_prev = x;
    y_prev = y;
    if (_fixed) {
      v_x = 0;
      v_y = 0;
    } else {
      // bounding large velocities to avoid instabilities
      double translation_squared = _dt_sq * (v_x * v_x + v_y * v_y);
      if (translation_squared > max_translation_squared) {
          double translation = sqrt(translation_squared);
          v_x *= max_translation / translation;
          v_y *= max_translation / translation;
      }
      x += dt * v_x;
      y += dt * v_y;
    }
  }

  void Node::translate(const double dx, const double dy) {
    x += dx;
    y += dy;
  }
}
