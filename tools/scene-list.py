#!/usr/bin/env python3
"""Print the installed scene selectors and their descriptions."""
import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('console',Path(__file__).with_name('web-sim-console.py'))
console=importlib.util.module_from_spec(spec);spec.loader.exec_module(console)
for scene in console.SCENES:
    print(f"{scene['key']:18} {scene['name']} · {scene['description']}")
