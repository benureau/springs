#include <numeric>
#include <cmath>
#include <sys/types.h>
#include <iostream>

#include "sensors.h"


namespace springs {

    Sensor::Sensor() { _initialized = false; }
    Sensor::~Sensor() = default;
    double Sensor::update() { _value = -12345678.0; return _value; }
    double Sensor::value() { return _value; }

    SensorHub::SensorHub() {}

    // vector<double> SensorHub::sensor_values() {
    //     vector<double> values;
    //     for (auto& sensor: sensors) {
    //         values.push_back(sensor->value());
    //     }
    //     return values;
    // }

    void SensorHub::add_sensor(Sensor* sensor) {
        sensors.push_back(sensor);
    }

    AngleSensor::AngleSensor(Node* origin, Node* satellite)
      : origin(origin), satellite(satellite), ref_sensor(NULL) {
       _reference_angle = 0.0;
       _reference_angle = update();
    }

    AngleSensor::AngleSensor(Node* origin, Node* satellite, AngleSensor* ref_sensor)
      : origin(origin), satellite(satellite), ref_sensor(ref_sensor) {
       _reference_angle = 0.0;
       _reference_angle = update();
    }

    double AngleSensor::update() {
        double old_value = _value;
        _value = atan2(satellite->y - origin->y, satellite->x - origin->x) - _reference_angle;
        if (ref_sensor != NULL) { _value -= ref_sensor->value(); }
        if (_initialized) {
          _value += round((old_value - _value) / 6.28318530718) * 6.28318530718;
        }
        _initialized = true;
        return _value;
    }

    AngularVelocitySensor::AngularVelocitySensor(AngleSensor* sensor, double dt)
      : sensor(sensor), dt(dt) {
        _value = 0.0;
        _previous_angle = sensor->value();
    }

    double AngularVelocitySensor::update() {
        double new_angle = sensor->value();
        _value = 0.5 * _value + 0.5 * (new_angle - _previous_angle) / dt;
        _previous_angle = new_angle;
        return _value;
    }

    TouchSensor::TouchSensor(vector<Node*> nodes)
      : nodes(nodes) { _value = update(); }

    TouchSensor::~TouchSensor() { nodes.clear(); }

    double TouchSensor::update() {
        bool colliding = false;
        for (auto& node: nodes) { colliding = colliding || node->colliding; }
        _value = colliding ? 0.0 : 1.0;
        return _value;
    }
}
