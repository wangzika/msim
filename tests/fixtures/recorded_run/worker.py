"""Trivial process used to demonstrate the recorded-run interface."""
import argparse
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--scenario', required=True)
parser.add_argument('--seed', required=True)
parser.add_argument('--output', required=True)
args = parser.parse_args()
Path(args.output, 'example.txt').write_text('seed=' + args.seed + '\n')
