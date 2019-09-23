#!/bin/bash

set -e
set -u
set -x

base=$(mktemp -d -p $PWD)
echo $base
time cp -r $PWD/nodocker/lammps $base
lammps=$base/lammps potc=$PWD/.. tmp=$base ./test.sh
rm -r $base
