"""Machine-readable preflight used by launch diagnostics."""
import argparse
from dataclasses import asdict
import json
import sys
from . import select_rendering

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mode", choices=("auto", "gpu", "cpu"), default="auto")
parser.add_argument("--purpose", choices=("gazebo", "rviz"), default="gazebo")
parser.add_argument("--gui", choices=("true", "false"), default="false")
parser.add_argument("--timeout", type=float, default=8.0)
arguments = parser.parse_args()
try:
    selection = select_rendering(arguments.mode, gui=arguments.gui == "true",
                                 purpose=arguments.purpose, timeout=arguments.timeout)
    result = asdict(selection)
    result.pop("environment")  # Never print the complete inherited environment.
    print(json.dumps(result, ensure_ascii=False))
except (RuntimeError, ValueError) as error:
    print(json.dumps({"error": str(error)}, ensure_ascii=False))
    sys.exit(1)
