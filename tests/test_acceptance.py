import copy
import unittest
from tools.acceptance import validate_round


class AcceptanceTests(unittest.TestCase):
    def evidence(self):
        counts=dict(A=2,B=2,C=2)
        result=dict(source_sha256='frozen',seed=0,continued_scene=False,visual_success=True,
                    requested_count=6,initial_bin_counts=dict.fromkeys('ABC',0),
                    final_bin_counts=counts,completed=[dict(kind=k) for k in 'AABBCC'],elapsed_s=2700.)
        manifest=dict(source_sha256='frozen',arguments=dict(seed=0,prepared=False,resume_held=None))
        independent=dict(verified=True,expected=counts,frames=[dict(stamp=t,counts=counts) for t in range(6,11)])
        video=dict(premature_exit=False,sha256='video',started_wall_s=0.,policy_launch_wall_s=1.,
                   probe=dict(format=dict(duration='2702')))
        events=[dict(event='final_check',stamp=t,counts=counts,stable=t) for t in range(1,6)]
        return result,manifest,independent,video,'frozen',0,events

    def test_fresh_complete_round_passes(self):
        validate_round(*self.evidence())

    def test_rejects_resumed_mixed_source_stale_or_incomplete_evidence(self):
        variants=[(0,'continued_scene',True),(0,'source_sha256','changed'),
                  (0,'initial_bin_counts',dict(A=1,B=0,C=0)),(0,'elapsed_s',3011.924),
                  (0,'visual_success',False),(3,'premature_exit',True)]
        for index,key,value in variants:
            with self.subTest(key=key):
                args=self.evidence();args[index][key]=value
                with self.assertRaises(ValueError):validate_round(*args)
        args=self.evidence();args[2]['frames'][0]['stamp']=5
        with self.assertRaisesRegex(ValueError,'newer'):validate_round(*args)
        args=self.evidence();args[6][3]['stamp']=3
        with self.assertRaisesRegex(ValueError,'Repeated'):validate_round(*args)
        args=self.evidence();args[0]['completed'].pop()
        with self.assertRaisesRegex(ValueError,'ledger'):validate_round(*args)
