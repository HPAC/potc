#!/bin/bash

set -e
set -u
set -x

outertmp="${tmp:-/tmp}"
iterations=3

tmp=$outertmp/all
mkdir $tmp
potc_suffix=""
source $1

do_ref=0

tmp=$outertmp/essential
mkdir $tmp
potc_suffix="E"
source $1

for suffix in E O S I a d f F r; do
  tmp=$outertmp/all-but-$suffix
  mkdir $tmp
  potc_suffix=" $suffix"
  source $1
  # tmp=$outertmp/essential-plus-$suffix
  # mkdir $tmp
  # potc_suffix="E $suffix"
  # source $1
done
