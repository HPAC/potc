#!/bin/bash

set -e
set -u
set -x

target=kokkos_cuda_mpi
potc_args="rk"
potentials="adp eim rebo tersoff eam sw"
runs="regular kokkos"

gpu_arch="${gpu_arch:-Pascal60}"

patch_source () {
  make yes-KOKKOS
  #sed -i 's/Kepler35/Volta70/' MAKE/OPTIONS/Makefile.kokkos_cuda_mpi
  sed -i "s/Kepler35/${gpu_arch}/" MAKE/OPTIONS/Makefile.kokkos_cuda_mpi
}

patch_input () {
  if [ "$2" == "kokkos" ]; then
    cat $3 | sed -e 's/run\s*100/replicate 4 4 4\nrun 1000/' > ${3}.old
    mv ${3}.old $3
    cat $3
    if [[ "$1" == tersoff* ]]; then
      mv $3 ${3}.old
      cat <(echo "package kokkos newton on neigh half") ${3}.old > $3
    fi
    if [[ "$1" == sw* ]]; then
      mv $3 ${3}.old
      cat <(echo "package kokkos newton on") ${3}.old > $3
    fi
    if [[ "$1" == eam* ]]; then
      mv $3 ${3}.old
      cat <(echo "package kokkos newton on") ${3}.old > $3
    fi
  fi
}

runall () {
  run $1 regular ""
  if [[ "$1" == *gen ]] || [[ "$1" == *tersoff* ]] || [[ "$1" == *eam* ]] || [[ "$1" == *sw* ]]; then
    run $1 kokkos "-kokkos on gpus 1 -sf kk"
  fi
}

source common-test.sh
