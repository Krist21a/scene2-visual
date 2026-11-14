import unittest
from unittest.mock import Mock
from scene2.timing import PhaseTimer


class TimingTests(unittest.TestCase):
    def test_nested_failed_work_is_accounted_once(self):
        log=Mock();clock=Mock(side_effect=[0.,2.,5.,8.])
        timer=PhaseTimer(log,clock)
        with self.assertRaises(ValueError):
            with timer.measure('motion','outer'):
                with timer.measure('planning','inner'):
                    raise ValueError('IK failed')
        self.assertEqual(timer.summary(),{'motion':dict(exclusive_s=5.,calls=1),'planning':dict(exclusive_s=3.,calls=1)})
        self.assertFalse(log.call_args.kwargs['success'])
        self.assertEqual(timer.stack,[])
