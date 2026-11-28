"""Recovery JSONL must survive actual-output closure, beyond selected-file QA."""
from pathlib import Path
import tempfile,unittest
from check_restore_closure import binding,verify_sealed_outputs

class RestoreClosureRegression(unittest.TestCase):
 def test_missing_or_corrupt_recovery_metadata_cannot_pass_restored_dependency_closure(self):
  with tempfile.TemporaryDirectory()as folder:
   root=Path(folder);required=root/'review/resume/observed_launcher_stdout.jsonl';required.parent.mkdir(parents=True)
   required.write_bytes(b'{"case_id":"saved_N","returncode":0}\n')
   seal=dict(files={'review/resume/observed_launcher_stdout.jsonl':binding(required)})
   self.assertEqual(len(verify_sealed_outputs(root,seal)),1)
   required.unlink()
   with self.assertRaisesRegex(AssertionError,'missing'):verify_sealed_outputs(root,seal)
   required.write_bytes(b'wrong bytes')
   with self.assertRaisesRegex(AssertionError,'differ'):verify_sealed_outputs(root,seal)
   model=root/'campaign/runs/example_N/source_snapshot/config/model.yaml';model.parent.mkdir(parents=True);model.write_bytes(b'radius: .12\n')
   needed=dict(files={'campaign/runs/example_N/source_snapshot/config/model.yaml':binding(model)})
   self.assertEqual(len(verify_sealed_outputs(root,needed)),1)
   model.unlink()
   with self.assertRaisesRegex(AssertionError,'missing'):verify_sealed_outputs(root,needed)
   model.write_bytes(b'radius: .99\n')
   with self.assertRaisesRegex(AssertionError,'differ'):verify_sealed_outputs(root,needed)

if __name__=='__main__':unittest.main()
