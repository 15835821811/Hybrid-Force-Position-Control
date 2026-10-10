from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[5]/'tools'))
from s02_visualization import plot_run
if __name__=='__main__':plot_run(sys.argv[1] if len(sys.argv)>1 else 'E0_C2', 'detumbling')
