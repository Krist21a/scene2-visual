import unittest
from tools.workstation import validate_runtime


class RuntimeGuardTests(unittest.TestCase):
    def setUp(self):
        self.expected={'/runtime/humanoid_VR_hand_control':'reference-A',
                       '/runtime/libhumanoid_controllers.so':'wrist-C',
                       '/runtime/task.info':'frozen-task'}
        self.receipt={'files':dict(self.expected),'loaded':{
            'humanoid_VR_hand_control':['reference-A'],
            'libhumanoid_controllers.so':['wrist-C']}}

    def test_verified_files_and_loaded_processes_pass(self):
        validate_runtime(self.receipt,self.expected,require_running=True)

    def test_original_runtime_cannot_silently_run_scene2(self):
        self.receipt['files']['/runtime/libhumanoid_controllers.so']='original'
        with self.assertRaisesRegex(RuntimeError,'A\\+C runtime'):
            validate_runtime(self.receipt,self.expected)

    def test_replacing_disk_file_does_not_validate_an_old_loaded_process(self):
        self.receipt['loaded']['libhumanoid_controllers.so']=['deleted']
        with self.assertRaisesRegex(RuntimeError,'Running process'):
            validate_runtime(self.receipt,self.expected,require_running=True)

    def test_running_check_requires_both_artifacts(self):
        self.receipt['loaded'].pop('humanoid_VR_hand_control')
        validate_runtime(self.receipt,self.expected)
        with self.assertRaisesRegex(RuntimeError,'Running process'):
            validate_runtime(self.receipt,self.expected,require_running=True)
