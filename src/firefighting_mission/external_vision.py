from __future__ import print_function

def model_pose(message, model_name):
    """Return pose paired with model_name, or None for incomplete messages."""
    try:
        index = message.name.index(model_name)
        return message.pose[index]
    except (ValueError, IndexError):
        return None


def model_state(message, model_name):
    """Return pose and world-frame twist for one model, or None."""
    try:
        index = message.name.index(model_name)
        return message.pose[index], message.twist[index]
    except (ValueError, IndexError):
        return None


def world_vector_to_body(orientation, vector):
    """Rotate a world-frame vector through inverse body-to-world quaternion."""
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm <= 0.0:
        raise ValueError('orientation quaternion must be non-zero')
    x /= norm
    y /= norm
    z /= norm
    w /= norm

    vx = float(vector.x)
    vy = float(vector.y)
    vz = float(vector.z)
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y + z * w) * vy
        + 2.0 * (x * z - y * w) * vz,
        2.0 * (x * y - z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z + x * w) * vz,
        2.0 * (x * z + y * w) * vx
        + 2.0 * (y * z - x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    )


def due_for_publish(last_sample_time, sample_time, period):
    """Return whether a sample taken at ``sample_time`` should be forwarded.

    The bridge throttles inside the ``/gazebo/model_states`` callback rather
    than on a separate timer, so the stamp it writes belongs to the sample it
    sends.  A timer decouples the two by up to one period, and PX4 fuses
    external vision with ``EKF2_EV_DELAY`` at zero, so that offset shows up
    as innovation and makes EKF2 reset to vision.
    """
    period = float(period)
    if period <= 0.0:
        raise ValueError('period_must_be_positive')
    if last_sample_time is None:
        return True
    elapsed = float(sample_time) - float(last_sample_time)
    if elapsed < 0.0:
        # Simulated time restarted; never stall waiting for the old clock.
        return True
    # Sim-time subtraction lands just under the period often enough that an
    # exact compare drops every other sample and halves the feed rate.
    return elapsed >= period - 1e-9


def _finite(value):
    value = float(value)
    return value == value and value not in (float('inf'), float('-inf'))


def sample_is_usable(pose, twist):
    """Return whether a Gazebo sample is safe to feed to EKF2.

    Gazebo reports NaN pose and velocity for a model whose physics has not
    settled, and a single NaN reaches EKF2 as an unusable observation: the
    filter resets to external vision on every frame and the local position
    never becomes valid.  One campaign run logged 717 resets against 9 for a
    healthy run, so a sample that is not finite has to be dropped rather
    than forwarded.
    """
    values = (pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z,
              pose.orientation.w,
              twist.linear.x, twist.linear.y, twist.linear.z,
              twist.angular.x, twist.angular.y, twist.angular.z)
    if not all(_finite(value) for value in values):
        return False
    norm = sum(float(value) ** 2 for value in
               (pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)) ** 0.5
    return norm > 0.5
