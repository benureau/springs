#ifndef SPACE_H
#define SPACE_H

#include <vector>
#include <sys/types.h>

#include "node.h"
#include "link.h"
#include "rect.h"
#include "trig.h"
#include "sensors.h"


using namespace std;

namespace springs {
  class Space {
    public:
      Space(double dt, uint n_substep, double gravity_x, double gravity_y,
               double restitution_threshold);
      ~Space();

      uint n_substep;
      int ticks;
      double gravity_x, gravity_y, t;
      double restitution_threshold;

      vector<Link*> links;
      vector<Spring*> springs;
      vector<Node*> nodes;
      vector<Rect*> rects;
      vector<Triangle*> triangles;

      SensorHub* sensors;

      double dt();
      void set_dt(double);

      Node* add_node(double x, double y, double mass, double friction, double fixed);

      Link* add_link(Node* node_a, Node* node_b, double stiffness, double damping_ratio,
                     bool actuated, double max_impulse);
      Spring* add_spring(Node* node_a, Node* node_b, double spring, double damping_ratio,
                         bool actuated, double max_impulse);

      Rect* add_rect(double xL, double xR, double yB, double yT, double restitution);
      Triangle* add_triangle(const double xA, const double yA, const double xB, const double yB,
                             const double xC, const double yC, const double restitution);

      AngleSensor* add_angle_sensor(Node* origin, Node* satellite);
      AngleSensor* add_relative_angle_sensor(Node* origin, Node* satellite, AngleSensor* sensor);
      TouchSensor* add_touch_sensor(vector<Node*> nodes);
      AngularVelocitySensor* add_angular_velocity_sensor(AngleSensor* sensor);

      void step();

    protected:
      double _dt;
      CollisionDetector* collision_detector;
      TriangleCollisionDetector* triangle_collision_detector;
  };
}

#endif
