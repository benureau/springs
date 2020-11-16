#ifndef LINK_H
#define LINK_H


namespace springs {

  class Link {

    public:
      Link();
      Link(double dt, Node* node_a, Node* node_b,
           double stiffness, double damping_ratio, bool actuated, double max_impulse);

      Node* node_a;
      Node* node_b;

      bool actuated, active;
      double expand_factor, relax_length, max_impulse;
      double max_length; // max *achieved* length.

      double length();

      double dt();
      void set_dt(double);

      double stiffness();
      void set_stiffness(double value);

      double damping_ratio();
      void set_damping_ratio(double value);

      double frequency();
      void set_frequency(double value);

      void contract(double expand_factor);
      void relax();

      double prestep();
      double substep();
      void  _update();

      double force();

    protected:
      void _update_velocities(const double impulse);
      // Return the distance *and* update the unit vector
      double _distance_unit_vector();
      double _relative_velocity();

      double _mass, _inv_mass;
      double _u_x, _u_y;

      double _dt, _stiffness, _damping_ratio, _frequency;
      double _bias, _gamma, _impulse, _v_substep;
  };
}

#endif
