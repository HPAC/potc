#!/bin/bash

set -e
set -u
set -x

base=$(mktemp -d -p $TEMP)
echo $base
mkdir $base/lammps
time cp -rp $PWD/nodocker/lammps/src $base/lammps
mkdir $base/lammps/lib
time cp -rp $PWD/nodocker/lammps/lib/kokkos $base/lammps/lib
lammps=$base/lammps potc=$PWD/.. tmp=$base ./$1.sh
rm -r $base
