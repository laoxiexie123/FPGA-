"""V3.1 top entry: run the current LCD/EC11 integration suite.

The copied tb_top.v and old top_* reports are historical V3 SW-key evidence,
not compatible with the new top interface. Current evidence is ui_test_results.json.
"""
from pathlib import Path
import runpy

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).with_name('run_ui_tests.py')), run_name='__main__')
