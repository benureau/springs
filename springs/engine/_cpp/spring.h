#ifndef SPRING_H
#define SPRING_H


#include "link.h"

namespace springs {

  class Spring: public Link {

    public:
      Spring();
      Spring(double dt, Node* node_a, Node* node_b,
             double stiffness, double damping_ratio, bool actuated, double max_impulse);

      double prestep();
      double substep();
      void  _update();

    protected:
      double _damping;
  };
}

#endif
