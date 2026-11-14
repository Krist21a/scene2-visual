import unittest
from bootstrap import ROOT
from scene2.task import checkpoint_counts


class ResumeLedgerTests(unittest.TestCase):
    def test_latest_cumulative_ledger_is_counted_once(self):
        rows=[dict(event='initial_occupancy',counts=dict(A=0,B=0,C=0)),
              dict(event='completed',ledger=[dict(kind='B')]),
              dict(event='completed',ledger=[dict(kind='B'),dict(kind='C')])]
        self.assertEqual(checkpoint_counts(rows),({'A':0,'B':1,'C':1},True))
        self.assertEqual(rows[0]['counts']['C'],0)

    def test_held_object_is_not_a_completed_placement(self):
        rows=[dict(event='initial_occupancy',counts=dict(A=2,B=2,C=1)),
              dict(event='held_transport_resumed',initial_counts=dict(A=2,B=2,C=1)),
              dict(event='gripper',close=True)]
        self.assertEqual(checkpoint_counts(rows),({'A':2,'B':2,'C':1},False))
