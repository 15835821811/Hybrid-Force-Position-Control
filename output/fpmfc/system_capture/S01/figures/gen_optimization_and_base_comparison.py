from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[5]))
from v6_mujoco.system_capture.paper_bridge.figures import run
if __name__ == '__main__': run('optimization_and_base_comparison')
