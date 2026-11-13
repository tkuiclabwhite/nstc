import os
import sys

# 不用先 colcon build 就能在桌機直接跑 pytest
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)
