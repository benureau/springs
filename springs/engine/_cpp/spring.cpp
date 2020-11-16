#include <iostream>
#include <algorithm>
#include <math.h>

#include "link.h"

namespace springs {
  //TODO: make this inherit from Link

  Spring::Spring() {}

  Spring::Spring(double dt, Node* node_a, Node* node_b, double stiffness,
             double damping_ratio, bool actuated, double max_impulse)
  : Link(dt, node_a, node_b, stiffness, damping_ratio, actuated, max_impulse)
  {
      _update();
  }

  // Physic updates

  inline void Spring::_update() {
    active = !(node_a->fixed() && node_b->fixed());
    if (active) {
      _inv_mass = node_a->inv_mass() + node_b->inv_mass();
      _mass = 1 / _inv_mass;

      double omega = sqrt(_stiffness * _inv_mass);
      _frequency = omega / 6.28318530718;  // omega / 2π
      _damping = 2 * _mass * _damping_ratio * omega;
      _impulse = 0.0;
    }
  }

  inline double Spring::prestep() {
    _v_substep = 0;
    double d = _distance_unit_vector();
    if (d > 0.0) {
      double diff_d = expand_factor * relax_length - d;
      _bias = diff_d * _stiffness * _dt;
      _impulse = _bias;
      _update_velocities(_impulse);
      return _impulse;
    }
    return 0.0;
  }

  inline double Spring::substep() {
    if (active) {
      double v_rn = _relative_velocity();
      double v_drag = _dt * _damping * (_v_substep - v_rn);
      _v_substep = v_rn + v_drag;
      double impulse = v_drag;
      _impulse += impulse;
      _update_velocities(impulse);
      return impulse;
    }
    return 0.0;
  }
}
