#!/usr/bin/env python3.11
import argparse
from pathlib import Path

import mypy.main

parser = argparse.ArgumentParser()
parser.add_argument('path', nargs='?', default=Path(__file__).parent / 'rosys')
args = parser.parse_args()

mypy.main.main(args=[str(args.path)])
