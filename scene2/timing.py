"""Exclusive phase timing; nested planning/motion is never counted twice."""
from collections import defaultdict
from contextlib import contextmanager
from functools import wraps
import time


class PhaseTimer:
    def __init__(self, log, clock=time.monotonic):
        self.log, self.clock = log, clock
        self.stack = []
        self.totals = defaultdict(float)
        self.calls = defaultdict(int)

    @contextmanager
    def measure(self, phase, operation):
        frame = dict(start=self.clock(), children=0.)
        self.stack.append(frame)
        ok = False
        try:
            yield
            ok = True
        finally:
            duration = self.clock() - frame['start']
            self.stack.pop()
            exclusive = max(0., duration - frame['children'])
            self.totals[phase] += exclusive
            self.calls[phase] += 1
            if self.stack:
                self.stack[-1]['children'] += duration
            self.log('phase_timing', phase=phase, operation=operation,
                     duration_s=duration, exclusive_s=exclusive, success=ok)

    def summary(self):
        return {key: dict(exclusive_s=value, calls=self.calls[key])
                for key, value in sorted(self.totals.items())}


def timed(phase):
    def decorate(function):
        @wraps(function)
        def call(self, *args, **kwargs):
            timer = getattr(self, 'timing', None)
            if timer is None:
                return function(self, *args, **kwargs)
            with timer.measure(phase, function.__qualname__):
                return function(self, *args, **kwargs)
        return call
    return decorate
