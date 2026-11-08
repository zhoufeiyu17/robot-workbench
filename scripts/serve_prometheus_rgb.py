#!/usr/bin/env python3
"""Subscribe to existing raw RGB; no camera/controller construction."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robot_workbench.prometheus_rgb import main
if __name__=='__main__':main()
