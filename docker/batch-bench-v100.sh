#!/usr/local_rwth/bin/zsh
#SBATCH --mem-per-cpu=1024M
#SBATCH --job-name=v100_potc_bench
#SBATCH --output=results/v100/log-%J.txt
#SBATCH --exclusive
#SBATCH --time=1:00:00
#SBATCH --partition=c18g
#SBATCH --account=nova0013
#SBATCH --gres=gpu:volta:2

set -e
set -u
set -x

module switch intel gcc/6
module switch intelmpi openmpi
module load cuda

base=$(mktemp -d -p $TEMP)
echo $base
mkdir $base/lammps
time cp -rp $PWD/nodocker/lammps/src $base/lammps
mkdir $base/lammps/lib
time cp -rp $PWD/nodocker/lammps/lib/kokkos $base/lammps/lib
mkdir $PWD/results/v100/$SLURM_JOB_ID
lammps=$base/lammps potc=$PWD/.. tmp=$PWD/results/v100/$SLURM_JOB_ID gpu_arch=Volta70 ./benchmark.sh test-kokkos.sh
rm -r $base
