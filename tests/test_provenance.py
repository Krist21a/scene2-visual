import tempfile
import unittest
from pathlib import Path
from scene2.provenance import source_manifest,verify_source_tree


class SourceDeploymentTests(unittest.TestCase):
    def tree(self,root):
        for name in ('run_scene2.py','run.sh','bootstrap.py','vendor/scene1/config.yaml'):
            path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('baseline')

    def test_removed_fixture_is_pruned_but_run_evidence_is_preserved(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);self.tree(root);expected=source_manifest(root)
            stale=root/'tests/fixtures/c-release-pivot-c20.json'
            stale.parent.mkdir(parents=True);stale.write_text('{}')
            evidence=root/'outputs/round/frame.npz';evidence.parent.mkdir(parents=True);evidence.write_bytes(b'RGB-D')
            cache=root/'scene2/__pycache__/old.pyc';cache.parent.mkdir(parents=True);cache.write_bytes(b'old cache')
            self.assertNotEqual(source_manifest(root),expected)
            self.assertEqual(verify_source_tree(root,expected,prune_stale=True),expected)
            self.assertFalse(stale.exists());self.assertFalse(cache.exists())
            self.assertEqual(evidence.read_bytes(),b'RGB-D')

    def test_read_only_check_rejects_extra_sources_without_changing_them(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);self.tree(root);expected=source_manifest(root)
            stale=root/'scene2/old.py';stale.parent.mkdir();stale.write_text('old policy')
            with self.assertRaisesRegex(RuntimeError,'extra=.*old.py'):
                verify_source_tree(root,expected)
            self.assertTrue(stale.exists())

    def test_wrong_code_cannot_be_hidden_by_pruning(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);self.tree(root);expected=source_manifest(root)
            (root/'run_scene2.py').write_text('different code')
            with self.assertRaisesRegex(RuntimeError,'changed=.*run_scene2.py'):
                verify_source_tree(root,expected,prune_stale=True)


if __name__=='__main__':unittest.main()
