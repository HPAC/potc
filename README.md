# PotC - The Potential Compiler

PotC is a project to automatically generate [LAMMPS](https://lammps.sandia.gov) `pair_syle` implementations from a simple DSL.
That DSL specifies the energy expression as well as any parameters or functions used in that expression.

# Build

PotC is built using `cmake`.
It requires GTest.
The final binary is called `potc`.

    mkdir build; cd $_
    cmake $POTC_PATH
    make

# Input

The input can use a number of constructs.
Some ready-made potentials and parametrizations are given in the `potentials/` folder.
Here is a quick example (a simple [Lennard-Jones](https://en.wikipedia.org/wiki/Lennard-Jones_potential) potential):

    # this reads a file that contains two scalars:
    # first epsilon, then sigma
    # like so: 1.00 2.00
    parameter epsilon = file(1);
    parameter sigma = file(2);
    energy sum(i : all_atoms) sum(j: neighbors(i, 3 * sigma)) 
      4 * epsilon * ((sigma / r) ^ 12 - (sigma / r) ^ 6);

# Invoking

PotC is invoked from the command line with three parameters:
The input file (see above), the final name of the `pair_style` and the mode, which can be `r` for a regular implementation, `i` for a [`USER-INTEL`](https://lammps.sandia.gov/doc/Speed_intel.html) implementation or `k` for a [`KOKKOS`](https://lammps.sandia.gov/doc/Speed_kokkos.html) implementation.

    ./potc $PATH_TO_INPUT $NAME $MODE

It will generate files `pair_$NAME_gen.cpp` and `pair_$NAME_gen.h` in the current working directory.
For the accelerated modes, the file names will also have the appropriate suffix.
These files can be dropped into a LAMMPS source tree and build automatically along with LAMMPS.
