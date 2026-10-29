#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../.."
if ! command -v clang-format >/dev/null 2>&1; then
  echo 'clang-format is required for the format check' >&2
  exit 1
fi
find core tests -type f \( -name '*.cpp' -o -name '*.hpp' \) -print0 |
  xargs -0 -r clang-format --dry-run --Werror
