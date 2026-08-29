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
