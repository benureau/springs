#include <iostream>

#include "space.h"


namespace springs {

  double MODULO = 1e-10;

  Space::Space(double dt, uint n_substep_, double gravity_x_, double gravity_y_,
               double restitution_threshold_) {
    set_dt(dt);
    n_substep = n_substep_;
    gravity_x = gravity_x_;
    gravity_y = gravity_y_;
    restitution_threshold = restitution_threshold_;

    collision_detector = new CollisionDetector(-1, -1);
    triangle_collision_detector = new TriangleCollisionDetector(-1, -1);
    sensors = new SensorHub();

    ticks = 0;
    t = 0.0;
  }

  Space::~Space() {
    for (auto& node: nodes) { delete node; }
    for (auto& link: links) { delete link; }
    for (auto& rect: rects) { delete rect; }
  }

  double Space::dt() {
    return _dt;
  }

  void Space::set_dt(double value) {
    _dt = value;
    for (auto& node: nodes) { node->dt = value; }
    for (auto& link: links) { link->set_dt(value); }
  }

  Node* Space::add_node(double x, double y, double mass, double friction, double fixed) {
    Node* node = new Node(_dt, x, y, mass, friction, fixed);
    nodes.push_back(node);
    return node;
  }

  Link* Space::add_link(Node* node_a, Node* node_b, double stiffness, double damping_ratio,
                        bool actuated, double max_impulse) {
    Link* link = new Link(_dt, node_a, node_b, stiffness, damping_ratio, actuated, max_impulse);
    links.push_back(link);
    return link;
  }

  Spring* Space::add_spring(Node* node_a, Node* node_b, double stiffness, double damping_ratio,
                            bool actuated, double max_impulse) {
    Spring* spring = new Spring(_dt, node_a, node_b, stiffness, damping_ratio, actuated, max_impulse);
    springs.push_back(spring);
    return spring;
  }

  Rect* Space::add_rect(double xL, double xR, double yB, double yT, double restitution) {
    Rect* rect = new Rect(xL, xR, yB, yT, restitution);
    rects.push_back(rect);
    collision_detector->add_rect(rect);
    return rect;
  }

  Triangle* Space::add_triangle(const double xA, const double yA, const double xB, const double yB,
                                const double xC, const double yC, const double restitution) {
    Triangle* triangle = new Triangle(xA, yA, xB, yB, xC, yC, restitution);
    triangles.push_back(triangle);
    triangle_collision_detector->add_triangle(triangle);
    return triangle;
  }

  AngleSensor* Space::add_angle_sensor(Node* origin, Node* satellite) {
    AngleSensor* sensor = new AngleSensor(origin, satellite);
    sensors->add_sensor(sensor);
    return sensor;
  }

  AngleSensor* Space::add_relative_angle_sensor(Node* origin, Node* satellite, AngleSensor* ref_sensor) {
    AngleSensor* sensor = new AngleSensor(origin, satellite, ref_sensor);
    sensors->add_sensor(sensor);
    return sensor;
  }

  TouchSensor* Space::add_touch_sensor(vector<Node*> nodes) {
    TouchSensor* sensor = new TouchSensor(nodes);
    sensors->add_sensor(sensor);
    return sensor;
  }

  AngularVelocitySensor* Space::add_angular_velocity_sensor(AngleSensor* angle_sensor) {
    AngularVelocitySensor* sensor = new AngularVelocitySensor(angle_sensor, dt());
    sensors->add_sensor(sensor);
    return sensor;
  }

  void Space::step() {
    // std::cout << "step cpp\n";
    for (auto& node: nodes) {
      node->colliding = false;
      if (!node->fixed()) {
        node->v_x += gravity_x * _dt;
        node->v_y += gravity_y * _dt;
      }
    }

    for (auto& link: links) { link->prestep(); }
    for (auto& spring: springs) { spring->prestep(); }

    vector<Collision> collisions;
    collision_detector->detect_collisions(nodes, restitution_threshold, collisions);

    vector<Contact> contacts;
    triangle_collision_detector->detect_collisions(nodes, restitution_threshold, contacts);

    for (uint k = 0; k < n_substep; k++) {
      for (auto& collision: collisions) { collision.substep(); }
      for (auto& contact: contacts) { contact.substep(); }
      for (auto& link: links) { link->substep(); }
      for (auto& spring: springs) { spring->substep(); }
    }

    for (auto& node: nodes) { node->update_position(); }

    t += _dt;
    ticks += 1;
  }
}
