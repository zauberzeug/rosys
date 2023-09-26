#!/usr/bin/env python3.11
import argparse
import json
from pathlib import Path

from pylint import lint

settings_path = Path(__file__).parent / '.vscode' / 'settings.json'
settings = json.loads('\n'.join(line.split('//')[0] for line in settings_path.read_text().splitlines()))

parser = argparse.ArgumentParser()
parser.add_argument('path', nargs='?', default=Path(__file__).parent / 'rosys')
args = parser.parse_args()

lint.Run([str(args.path)] + settings['pylint.args'])
