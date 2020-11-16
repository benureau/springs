import sys


def test_box2d(verbose=False):
    """Test that the Box2D library works as expected."""
    try:
        import Box2D as b2

        world = b2.b2World()
        body_a = world.CreateDynamicBody(position=(0.0, 0.0), linearDamping=1.0,
                                         allowSleep=False, fixedRotation=True)
        body_a.CreateCircleFixture(radius=1.0, density=1.0, friction=0.0, maskBits=0)


        body_b = world.CreateDynamicBody(position=(10.0, 0.0), linearDamping=1.0,
                                         allowSleep=False, fixedRotation=True)
        body_b.CreateCircleFixture(radius=1.0, density=1.0, friction=0.0, maskBits=0)


        bond = world.CreateDistanceJoint(bodyA=body_a, bodyB=body_b, localAnchorA=(0, 0), localAnchorB=(0, 0), length=20)
        if verbose:
            print('length       ', bond.length)
            print('frequency    ', bond.frequency)
            print('dampingRatio ', bond.dampingRatio)

    except ImportError:
        print('could not import Box2D, skipping test.', file=sys.stderr)


if __name__ == '__main__':
    test_box2d()
