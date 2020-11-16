#include <iostream>
#include <algorithm>

#include "link.h"

// # define DEBUG_BUILD

namespace springs {

  Link::Link() {}

  Link::Link(double dt, Node* node_a, Node* node_b, double stiffness,
             double damping_ratio, bool actuated, double max_impulse)
  : node_a(node_a), node_b(node_b), actuated(actuated), expand_factor(1.0), max_impulse(max_impulse),
    _dt(dt), _stiffness(stiffness), _damping_ratio(damping_ratio)
  {
    node_a->links.push_back(this);
    node_b->links.push_back(this);

    relax_length  = _distance_unit_vector();
    assert (relax_length > 0);
    max_length    = relax_length;

    _update();
  }

  double Link::length() {
    double d_x = node_b->x - node_a->x;
    double d_y = node_b->y - node_a->y;
    return sqrt(d_x*d_x + d_y*d_y);
  }

  inline double Link::dt() {
    return _dt;
  }

  inline void Link::set_dt(double value) {
    _dt = value;
    _update();
  }

  double Link::stiffness() {
    return _stiffness;
  }

  void Link::set_stiffness(double value) {
    _stiffness = value;
    _update();
  }

  double Link::damping_ratio() {
    return _damping_ratio;
  }

  void Link::set_damping_ratio(double value) {
    _damping_ratio = value;
    _update();
  }

  double Link::frequency() {
    return _frequency;
  }

  void Link::set_frequency(double value) {
    double _mass = 1 / (node_a->inv_mass() + node_b->inv_mass());
    _frequency = sqrt(_stiffness / _mass) / 6.28318530718;  // omega / 2π
    _update();
  }

  void Link::contract(double value) {
    expand_factor = value;
  }

  void Link::relax() {
    expand_factor = 1.0;
  }


  // Physic updates

  inline void Link::_update() {
    active = !(node_a->fixed() && node_b->fixed());
    if (active) {
      _inv_mass = node_a->inv_mass() + node_b->inv_mass();
      _mass = _inv_mass == 0 ? 0.0 : 1.0 / _inv_mass;

      double omega = sqrt(_stiffness * _inv_mass);
      _frequency = omega / 6.28318530718;  // omega / 2π
      double _damping = 2 * _mass * _damping_ratio * omega;
      _gamma = 1.0 / (_dt * (_damping + _dt * _stiffness));

      _inv_mass += _gamma;
      _mass = 1 / _inv_mass;
      _impulse = 0.0;
    }
  }

  inline void Link::_update_velocities(const double impulse) {
    if (impulse != 0) {
      double P_x = impulse * _u_x;
      double P_y = impulse * _u_y;

      if (!node_a->fixed()) {
        node_a->v_x -= P_x * node_a->inv_mass();
        node_a->v_y -= P_y * node_a->inv_mass();
      }

      if (!node_b->fixed()) {
        node_b->v_x += P_x * node_b->inv_mass();
        node_b->v_y += P_y * node_b->inv_mass();
      }
    }
  }

  // Return the distance *and* update the unit vector
  inline double Link::_distance_unit_vector() {
    double d_x = node_b->x - node_a->x;
    double d_y = node_b->y - node_a->y;
    double d   = sqrt(d_x*d_x + d_y*d_y);
    if (d > 0) {
      _u_x = d_x / d;
      _u_y = d_y / d;
    }
    return d;
  }

  inline double Link::prestep() {
    if (active) {
      double d = _distance_unit_vector();
      if (d > 0.0) {
        double diff_d = d - expand_factor * relax_length;
        _bias = diff_d * _dt * _stiffness * _gamma;
        _update_velocities(_impulse);
        return _impulse;
      }
    }
    return 0.0;
  }

  // Relative velocity, projected onto the link's direction
  inline double Link::_relative_velocity() {
    double vBA_x = node_b->v_x - node_a->v_x;
    double vBA_y = node_b->v_y - node_a->v_y;
    return _u_x * vBA_x + _u_y * vBA_y;
  }

  inline double Link::substep() {
    if (active) {
      double v_r = _relative_velocity();
      double impulse = - _mass * (v_r + _bias + _gamma * _impulse);
      _impulse += impulse;
      _update_velocities(impulse);
      return impulse;
    }
    return 0.0;
  }

  double Link::force() {
    return _impulse / _dt;
  }
}
