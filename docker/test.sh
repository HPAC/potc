#!/bin/bash

set -e
set -u
set -x

target=serial
potc_args="r"

patch_source () {
  sed -i 's/-g -O3/-g -O3 -ffast-math/' MAKE/Makefile.serial
}

patch_input () {
  true
}

runall () {
  run $1 regular ""
}

source common-test.sh

