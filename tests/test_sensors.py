import math
import random

import springs


def test_angle_sensor():

    space = springs.create_space(dt=0.01, gravity=-100.0, engine='cpp')

    def create_link(space, mass, stiffness, damping_ratio):
        a = space.add_node(500, 300, mass=1.0, fixed=True)
        b = space.add_node(700, 300, mass=mass)
        spring = space.add_link(a, b, stiffness, damping_ratio=damping_ratio, actuated=False)
        return a, b

    origin, satellite = create_link(space, 1.0, 100000.0, 1.0)
    fixed = space.add_node(700, 300, mass=1.0, fixed=True)
    # spring = space.add_link(origin, c, 1.0, damping_ratio=1.0, actuated=False)

    sensor = space.add_angle_sensor(origin, satellite)
    fixed_sensor = space.add_angle_sensor(origin, fixed)
    sensor2 = space.add_angle_sensor(origin, satellite, sensor)
    sensor3 = space.add_angle_sensor(origin, satellite, fixed_sensor)
    fixed_sensor2 = space.add_angle_sensor(origin, fixed, sensor)
    vel_sensor = space.add_angular_velocity_sensor(sensor)

    old_value = sensor.value
    # springs.create_display(space, width=2000, height=2000).run()
    for t in range(1000):
        space.step()
        new_value = sensor.value

        print(t, "{} <? {}".format(old_value, new_value), vel_sensor.value)
        if t < 523:
            assert -math.pi < new_value < old_value
            assert vel_sensor.value <= 0.0
        if t > 524:
            assert -math.pi < old_value < new_value
            assert vel_sensor.value >= 0.0

        old_value = new_value
        assert sensor2.value == 0.0
        assert sensor3.value == sensor.value
        assert fixed_sensor2.value == - sensor.value

        assert space.sensor_values() == [sensor.value, fixed_sensor.value, sensor2.value, sensor3.value, fixed_sensor2.value, vel_sensor.value]

def test_touch_sensor1():

    space = springs.create_space(dt=0.01, gravity=-100.0, engine='cpp')

    a = space.add_node(100, 300, 1.0, fixed=False)
    b = space.add_node(150, 350, 1.0, fixed=False)
    ground = space.add_rect(0, 1000, -100, 0, restitution=0.0)

    sensor1 = space.add_touch_sensor([a])
    sensor2 = space.add_touch_sensor([a, b])

    assert sensor1.value == sensor2.value == 1.0
    for t in range(1000):
        space.step()
        assert sensor1.value <= sensor2.value
    assert sensor1.value == sensor2.value == 0.0

def test_touch_sensor2():

    space = springs.create_space(dt=0.01, gravity=-100.0, engine='cpp')

    nodes, sensors = [], []
    for i in range(100):
        nodes.append(space.add_node(100 + i, 100 + i, 1.0, fixed=False))
        sensors.append(space.add_touch_sensor(nodes))

    ground = space.add_rect(0, 1000, -100, 0, restitution=0.0)

    assert all(sensor.value == 1.0 for sensor in sensors)
    for t in range(1000):
        space.step()
        for i in range(len(sensors) - 1):
            assert sensors[i].value <= sensors[i+1].value
        assert space.sensor_values() == [sensor.value for sensor in sensors]
    assert all(sensor.value == 0.0 for sensor in sensors)


if __name__ == '__main__':
    test_angle_sensor()
    test_touch_sensor1()
    test_touch_sensor2()
