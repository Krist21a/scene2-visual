import unittest
from unittest.mock import Mock,patch
from scene2.task import Sorter
from scene2.control import Stop
from scene2.perception import Bin,Part


class FinalCheckTests(unittest.TestCase):
    def sorter(self):
        sorter=Sorter.__new__(Sorter)
        sorter.initial_counts={'B':1};sorter.completed=[{'kind':'B'}]
        sorter.scene=Mock();sorter.detector=Mock();sorter.save=Mock();sorter.log=Mock()
        return sorter

    def observation(self,count):
        part=Part('B',[.5,0,.03],[1,0],[.05,.04],.05,[],1.,1.)
        return [part]*count,{'B':Bin('B',[.5,0],[.4,-.1],[.6,.1],0,.15,1.)}

    @patch('scene2.task.time.sleep')
    def test_requires_five_consecutive_distinct_frames_after_count_loss(self,sleep):
        sorter=self.sorter()
        sorter.scene.read.side_effect=[{'stamp':s} for s in [1,1,2,3,4,5,6,7,8]]
        sorter.detector.detect.side_effect=[self.observation(n) for n in [2,2,1,2,2,2,2,2]]
        self.assertEqual(sorter.confirm_final(),{'B':2})
        self.assertEqual([call.kwargs['stable'] for call in sorter.log.call_args_list],[1,2,0,1,2,3,4,5])

    @patch('scene2.task.time.sleep')
    @patch('scene2.task.time.monotonic',side_effect=[0,1,21])
    def test_past_ledger_cannot_override_missing_final_object(self,clock,sleep):
        sorter=self.sorter();sorter.scene.read.return_value={'stamp':1}
        sorter.detector.detect.return_value=self.observation(1)
        with self.assertRaisesRegex(Stop,'five-frame'):sorter.confirm_final()
