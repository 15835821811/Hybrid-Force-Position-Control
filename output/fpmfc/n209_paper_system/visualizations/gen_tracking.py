from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
from v6_mujoco.feasible_capture.visualization import plot_run
if __name__ == '__main__':
    plot_run(sys.argv[1] if len(sys.argv)>1 else 'R01_V2_nominal', 'tracking')
