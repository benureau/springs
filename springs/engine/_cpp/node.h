#ifndef NODE_H
#define NODE_H

#include <vector>
#include <cmath>


using namespace std;

namespace springs {
  // because cluster does not support C++17.
  double clamp(double lower, double v, double upper) {
    return fmax(lower, fmin(v, upper));
  }

  class Link;

  class Node {
    public:
      double dt;
      double x, y;           // position
      double x_prev, y_prev; // position at the previous timestep
      double v_x, v_y;       // velocity
      double friction;
      bool   colliding;       // a collision was detected during the last timestep.

      // Links connected to the node
      // because links precompute values that may depend on a node parameter (mass, friction),
      // we want to call their `_update()` each time we change the value of a node parameter.
      vector<Link*> links;

      Node();
      Node(double dt, double x, double y, double mass, double friction, bool fixed);

      double mass();
      void set_mass(const double mass);
      double inv_mass();

      bool fixed();
      void set_fixed(const bool fixed);

      void update_position();
      void translate(const double dx, const double dy);

    private:
      double _mass;
      double _inv_mass;
      double _fixed;
      double _dt_sq;
  };
}

#endif
