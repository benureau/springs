#ifndef SENSORS_H
#define SENSORS_H

#include <vector>
#include <sys/types.h>
#include <cmath>

#include "node.h"


using namespace std;

namespace springs {
    class Sensor {
        public:
            Sensor();
            ~Sensor();

            double update();   // FIXME: should be virtual
            double value();   // FIXME: should be virtual

          protected:
              double _value;
              bool   _initialized;
    };

    class SensorHub {
        public:
            SensorHub();
            ~SensorHub();

            vector<Sensor*> sensors;
            // vector<double> sensor_values();

            void add_sensor(Sensor* sensor);
    };

    class AngleSensor : public Sensor {
        public:
            AngleSensor(Node* origin, Node* satellite);
            AngleSensor(Node* origin, Node* satellite, AngleSensor* ref_sensor);
            ~AngleSensor();

            double update();

            Node* origin;
            Node* satellite;
            AngleSensor* ref_sensor;

        protected:
            double _reference_angle;
    };

    /*  Angular Velocity sensor
     *
     */
    class AngularVelocitySensor : public Sensor {
        public:
            AngularVelocitySensor(AngleSensor* sensor, double dt);
            ~AngularVelocitySensor();

            double update();

            AngleSensor * sensor;
            double dt;
        protected:
            double _previous_angle;
    };

    /*  Touch sensor: return 0.0 if any of the sensors is colliding with the environment.
     *  1.0 otherwise.
     */
    class TouchSensor : public Sensor {
        public:
            TouchSensor(vector<Node*> nodes);
            ~TouchSensor();

            double update();

            vector<Node*> nodes;
    };

}

#endif
