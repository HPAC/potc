#!/bin/bash

set -e
set -u
set -x

target=intel_cpu
potc_args="ri"
potentials="tersoff sw eam adp eim"
runs="regular intel-double intel-mixed intel-single"
# intel-single intel-mixed"

patch_source () {
  make yes-USER-INTEL
  cp $potc/src/intel_intrinsics_potc.h .
}

patch_input () {
  true
}

runall () {
  run $1 regular ""
  run $1 intel-double "-pk intel 0 mode double -sf intel"
  run $1 intel-single "-pk intel 0 mode single -sf intel"
  run $1 intel-mixed  "-pk intel 0 mode mixed -sf intel"
}

source common-test.sh
