#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
if ! command -v clang-tidy >/dev/null 2>&1; then
  echo 'clang-tidy is required for the lint check' >&2
  exit 1
fi
cmake -S . -B build -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
find core/src tests/unit -type f -name '*.cpp' -print0 |
  xargs -0 -r clang-tidy -p build
