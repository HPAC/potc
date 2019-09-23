tmp="${tmp:-/tmp}"
potc="${potc:-/potc}"
lammps="${lammps:-/lammps}"
old=$(pwd)
do_ref="${do_ref:-1}"

cd $potc

git status
date

cd $lammps/src

potentials="${potentials:-eam tersoff sw adp eim rebo}"
runs="${runs:-regular}"
parallel="${parallel:-40}"
iterations="${iterations:-1}"
potc_suffix="${potc_suffix:-}"

for potential in $potentials; do
  $potc/build/potc $potc/potentials/${potential}.potc $potential ${potc_args}${potc_suffix}
done

patch_source

make $target -j$parallel
lmp=$lammps/src/lmp_$target
cd $potc/test/inputs

run () {
  cp in.$1 $tmp/in.${1}.$2
  patch_input $1 $2 $tmp/in.${1}.$2
  for it in $(seq 1 $iterations); do
    $lmp -log $tmp/log.$2.$1.$it -in $tmp/in.${1}.${2} $3 || true
    awk '/Loop/ {a=0;} {if (a==1) print $0;} /Step/ {a=1;}' $tmp/log.$2.$1.$it > $tmp/thermo.$2.$1.$it
    awk '/Loop/ { print $4 }' $tmp/log.$2.$1.$it > $tmp/perf.$2.$1.$it
  done
  cp $tmp/thermo.$2.${1}.1 $tmp/thermo.$2.$1
  for it in $(seq 1 $iterations); do
    diff $tmp/thermo.$2.$1 $tmp/thermo.$2.$1.$it
  done
  for it in $(seq 1 $iterations); do
    cat $tmp/perf.$2.$1.* | sort -g | awk '{a[NR] = $1} END {print (a[int((NR-1)/2)+1]+a[int((NR+0.9999)/2)+1])/2}' > $tmp/perf.$2.$1
    cat $tmp/perf.$2.$1.* | sort -g | awk '{a[NR] = $1} END {print a[NR] - a[1]}' > $tmp/range.$2.$1
  done
}

speedup () {
  for run in $runs; do
    # cat perf.regular.$1
    # cat perf.regular.$2
    cat perf.${run}.$1 perf.${run}.${1}.gen | perl -l -e 'print <STDIN>/<STDIN>' || true
    cat perf.regular.$1 perf.${run}.${1} | perl -l -e 'print <STDIN>/<STDIN>' || true
    cat perf.regular.$1 perf.${run}.${1}.gen | perl -l -e 'print <STDIN>/<STDIN>' || true
    #cat perf.regular.$2 perf.regular.$1 | perl -l -e 'print <STDIN>/<STDIN>'
  done
}

for potential in $potentials; do
  if [ "$do_ref" == "1" ]; then
    runall $potential
  fi
  runall ${potential}.gen
done

cd $tmp

if [ "$do_ref" == "1" ]; then

  for potential in $potentials; do
    for run in $runs; do
      diff thermo.regular.$potential  thermo.${run}.${potential} || true
      diff thermo.regular.$potential  thermo.${run}.${potential}.gen || true
    done
  done
  
  for potential in $potentials; do
    speedup $potential || true
  done

fi

cd $old
