#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
for version in v31 v32; do
  "${CXX:-c++}" -std=c++11 -Wall -Wextra -Werror -pedantic -I"tests/$version" "tests/$version/host_tests.cpp" -o "$build_dir/$version"
  "$build_dir/$version"
done
