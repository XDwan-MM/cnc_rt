#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
flavor="${1:-debug}"
extra=()
case "$flavor" in
    debug) build="$root/build/h1-tests" ;;
    production) build="$root/build/h1-tests-production"; extra+=("CONFIG+=h1_production_business") ;;
    *) echo "usage: $0 [debug|production]" >&2; exit 2 ;;
esac
mkdir -p "$build"
cd "$build"
qmake "$root/test/test.pro" DEFINES+=RT_HAL_H1_TEST "DESTDIR=$build/bin" "OBJECTS_DIR=$build/obj" \
    "TEST_SOURCES=$root/test/test_motion/tst_hal_h1_spindle.cpp $root/test/test_motion/tst_spindle_zero_speed_authorization.cpp $root/test/test_motion/tst_spindle_velocity_planner.cpp $root/test/test_motion/tst_emcmot.cpp" \
    "${extra[@]}"
make -j4
cd "$root"
# Test main uses its own NcDataTestRt shared memory and the memory PDO backend.
status=0
timeout 45s "$build/bin/tests" > "$build/run.log" 2>&1 || status=$?
cat "$build/run.log"
if [[ "$status" != 0 ]]; then exit "$status"; fi
# Some repository test-runtime cleanup paths override the process exit code.
if grep -q -E '\[  FAILED  \]|: Failure' "$build/run.log"; then exit 1; fi
grep -q '\[  PASSED  \]' "$build/run.log"

if [[ "$flavor" == production ]]; then
    grep -q -F '[       OK ] HalH1SpindleTest.ProductionM29CommandEntersPreparationThroughAutoMst' "$build/run.log"
    grep -q -F '[       OK ] HalH1SpindleTest.ProductionM29ReturnCommandUsesNoOrientationPreparation' "$build/run.log"
fi
