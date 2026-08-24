from __future__ import division, print_function

import cv2


def _text(value, name, default=''):
    return getattr(value, name, default) if value is not None else default


def draw_avoidance_overlay(image, status, obstacles):
    output = image.copy()
    state = _text(status, 'state', 'WAIT_STEREO')
    side = _text(status, 'selected_side', '')
    left = float(_text(status, 'left_clearance_m', 0.0))
    right = float(_text(status, 'right_clearance_m', 0.0))
    reason = _text(status, 'reason', '')
    color = (0, 255, 255)
    cv2.putText(output, '%s %s' % (state, side), (12, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
    cv2.putText(output, 'L %.2fm  R %.2fm  OBS %d' %
                (left, right, len(obstacles or ())), (12, 48),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
    if reason:
        cv2.putText(output, reason, (12, 70), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 180, 255), 1, cv2.LINE_AA)
    return output


def compose_mission_view(target_image, front_image, status, obstacles,
                         inset_scale=0.36):
    if target_image is None and front_image is None:
        return None
    if target_image is None:
        return draw_avoidance_overlay(front_image, status, obstacles)
    output = target_image.copy()
    if front_image is not None:
        height, width = output.shape[:2]
        inset_width = max(1, int(width * inset_scale))
        inset_height = max(1, int(front_image.shape[0] *
                                  inset_width / float(front_image.shape[1])))
        inset_height = min(inset_height, max(1, height // 2))
        inset = cv2.resize(front_image, (inset_width, inset_height))
        inset = draw_avoidance_overlay(inset, status, obstacles)
        x_value = width - inset_width - 8
        y_value = 8
        output[y_value:y_value + inset_height,
               x_value:x_value + inset_width] = inset
        cv2.rectangle(output, (x_value - 1, y_value - 1),
                      (x_value + inset_width, y_value + inset_height),
                      (0, 255, 255), 2)
    return output
