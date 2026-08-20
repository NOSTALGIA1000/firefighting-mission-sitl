from __future__ import division, print_function

from collections import deque, namedtuple

import cv2
import numpy as np


Detection = namedtuple('Detection', 'target_class confidence box')
HAZARD_CLASSES = frozenset(('flammable', 'explosive', 'toxic', 'distractor'))


def _gray(image):
    if image.ndim == 2:
        result = image
    else:
        result = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.equalizeHist(result.astype(np.uint8))


def annotation_color(target_class):
    if target_class == 'person':
        return (255, 0, 0)
    if target_class in HAZARD_CLASSES and target_class != 'distractor':
        return (0, 0, 255)
    return (0, 220, 255)


def annotate(image, box, target_class, confidence, phase='', elapsed=0.0,
             payload_state='READY'):
    output = image.copy()
    if box is not None:
        x, y, width, height = box
        color = annotation_color(target_class)
        cv2.rectangle(output, (x, y), (x + width, y + height), color, 2)
        cv2.putText(output, '%s %.2f' % (target_class, confidence),
                    (x, max(12, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    color, 1, cv2.LINE_AA)
    cv2.putText(output, '%s  %.1fs  %s' % (phase, elapsed, payload_state),
                (8, output.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (255, 255, 255), 1, cv2.LINE_AA)
    return output


class StableDetector(object):
    def __init__(self, window=5, required=4):
        if required > window:
            raise ValueError('required confirmations cannot exceed window')
        self.window = int(window)
        self.required = int(required)
        self.current_class = ''
        self.history = deque(maxlen=self.window)

    @property
    def confirmation_count(self):
        return sum(1 for matched in self.history if matched)

    def update(self, target_class, matched):
        if target_class != self.current_class:
            self.current_class = target_class
            self.history.clear()
        self.history.append(bool(matched))
        return (len(self.history) == self.window and
                self.confirmation_count >= self.required)


class TemplatePerception(object):
    def __init__(self, templates, threshold=0.72,
                 scales=(0.75, 1.0, 1.25)):
        self.templates = dict((name, _gray(image))
                              for name, image in templates.items())
        self.threshold = float(threshold)
        self.scales = tuple(float(scale) for scale in scales)

    @staticmethod
    def _allowed(phase):
        if 'HAZARD' in phase:
            return HAZARD_CLASSES
        if 'PERSON' in phase:
            return frozenset(('person',))
        return frozenset()

    def detect(self, image, phase):
        gray = _gray(image)
        allowed = self._allowed(phase)
        best = Detection('none', 0.0, None)
        for label, template in self.templates.items():
            if label not in allowed:
                continue
            for scale in self.scales:
                candidate = cv2.resize(template, None, fx=scale, fy=scale,
                                       interpolation=cv2.INTER_AREA)
                height, width = candidate.shape[:2]
                if height > gray.shape[0] or width > gray.shape[1]:
                    continue
                scores = cv2.matchTemplate(gray, candidate,
                                           cv2.TM_CCOEFF_NORMED)
                _, confidence, _, origin = cv2.minMaxLoc(scores)
                if confidence > best.confidence:
                    best = Detection(label, float(confidence),
                                     (origin[0], origin[1], width, height))
        if best.confidence < self.threshold:
            return Detection('none', best.confidence, None)
        return best


def load_templates(directory):
    templates = {}
    for label in ('flammable', 'explosive', 'toxic', 'person', 'distractor'):
        image = cv2.imread('%s/%s.png' % (directory.rstrip('/'), label),
                           cv2.IMREAD_GRAYSCALE)
        if image is not None:
            templates[label] = image
    if not templates:
        raise ValueError('no readable PNG templates in %s' % directory)
    return templates
