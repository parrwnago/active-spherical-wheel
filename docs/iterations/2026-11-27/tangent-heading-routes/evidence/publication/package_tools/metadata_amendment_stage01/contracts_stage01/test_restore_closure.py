"""Recovery JSONL must survive actual-output closure, beyond selected-file QA."""
from pathlib import Path
import tempfile,unittest
from check_restore_closure import binding,verify_sealed_outputs

class RestoreClosureRegression(unittest.TestCase):
 def test_missing_or_corrupt_recovery_jsonl_cannot_pass_sealed_output_closure(self):
  with tempfile.TemporaryDirectory()as folder:
   root=Path(folder);required=root/'review/resume/observed_launcher_stdout.jsonl';required.parent.mkdir(parents=True)
   required.write_bytes(b'{"case_id":"saved_N","returncode":0}\n')
   seal=dict(files={'review/resume/observed_launcher_stdout.jsonl':binding(required)})
   self.assertEqual(len(verify_sealed_outputs(root,seal)),1)
   required.unlink()
   with self.assertRaisesRegex(AssertionError,'missing'):verify_sealed_outputs(root,seal)
   required.write_bytes(b'wrong bytes')
   with self.assertRaisesRegex(AssertionError,'differ'):verify_sealed_outputs(root,seal)

if __name__=='__main__':unittest.main()
