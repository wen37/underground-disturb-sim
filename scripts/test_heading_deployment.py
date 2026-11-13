"""Exercise deployment backup, retained results, drift detection and both layouts."""
import contextlib,importlib.util,io,json,subprocess,sys,tempfile,unittest
from pathlib import Path
import deploy_heading_workspace as deployment

class DeploymentTests(unittest.TestCase):
    def test_deployment_preserves_results_and_detects_edits(self):
        with tempfile.TemporaryDirectory() as directory:
            ws=Path(directory);target=ws/'src/coal_heading_scene'
            (target/'results/old').mkdir(parents=True)
            (target/'results/old/keep.txt').write_text('experiment evidence')
            (target/'references').mkdir()
            (target/'references/template_README.md').write_text('historical template')
            (target/'references/THIRD_PARTY.md').write_text('provenance')
            (target/'runtime').mkdir()
            (target/'runtime/old_script.py').write_text('historical code')
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertTrue(deployment.deploy(ws))
                self.assertTrue(deployment.deploy(ws,check=True))
            self.assertEqual((target/'results/old/keep.txt').read_text(),'experiment evidence')
            self.assertEqual((target/'references/THIRD_PARTY.md').read_text(),'provenance')
            self.assertFalse((target/'references/template_README.md').exists())
            self.assertFalse((target/'runtime/old_script.py').exists())
            self.assertTrue(list((target/'backups').rglob('old_script.py')))
            code="import paths,dust_model; print(dust_model.__file__)"
            # Resolve using the actual consumer import sequence, not a duplicate resolver.
            code="import paths,sys; sys.path.insert(0,str(paths.FIXED)); "+code
            output=subprocess.check_output([sys.executable,'-c',code],cwd=target/'runtime',text=True)
            self.assertIn(str(ws/'lidar_fixed_distance_test/dust_model.py'),output)
            modified=target/'runtime/heading_math.py';modified.write_text(modified.read_text()+'\n# local edit\n')
            with contextlib.redirect_stdout(io.StringIO()):self.assertFalse(deployment.deploy(ws,check=True))
            self.assertTrue(modified.read_text().endswith('# local edit\n'))

    def test_reject_external_destination_symlink(self):
        with tempfile.TemporaryDirectory() as directory, tempfile.TemporaryDirectory() as outside:
            ws=Path(directory);(ws/'src').mkdir()
            (ws/'src/coal_heading_scene').symlink_to(outside,target_is_directory=True)
            with self.assertRaises(ValueError):deployment.deploy(ws)
            self.assertEqual(list(Path(outside).iterdir()),[])

if __name__=='__main__':unittest.main(verbosity=2)
