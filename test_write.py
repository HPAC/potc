import sys
import os

lmp_dir = os.path.abspath('lammps-30Mar18')

sys.path.append(os.path.join(lmp_dir, 'python'))
print('PID', os.getpid()) # in case of debugging

import lammps
l = lammps.lammps(name="serial")

import ase
import ase.calculators.lammpslib as ase_lmp
import ase.build
import numpy as np

cmds = {'ref':
        ["pair_style tersoff",
         "pair_coeff * * %s/potentials/Si.tersoff Si" % lmp_dir],
        'our':
        ["pair_style pot/gen 3.2",
         "pair_coeff * * potentials/Si.potc-param Si"]}

default_header = ['units metal', 'atom_style atomic', 'atom_modify map array sort 0 0'] 

lammps = {}
for name, cmd in cmds.items(): # collect calculators for all the different code variants
    header = default_header
    lammps[name] = ase_lmp.LAMMPSlib(
        lmpcmds=cmd, log_file='test.log', lammps_name='serial', keep_alive=True, lammps_header=header)

# Si = ase.build.bulk('Si', cubic=True)
# #H = ase.Atom('H', position=C.cell.diagonal()/2)
# #CH = C + H
# #print("Energy ", CH.get_potential_energy())
# Si.set_calculator(lammps['our'])
# print("Energy ", Si.get_potential_energy())
# FON = lammps['our'].calculate_numerical_forces(Si, d=0.00001)
# FRN = lammps['ref'].calculate_numerical_forces(Si, d=0.00001)
# FR = lammps['ref'].get_forces(Si)
# FO = lammps['our'].get_forces(Si)
# 
# print(np.max(np.abs(FON - FO)))
# print(np.max(np.abs(FRN - FR)))
# print(np.max(np.abs(FR - FO)))

from ase.lattice.cubic import Diamond
from ase.md.velocitydistribution import MaxwellBoltzmannDistribution
from ase.md.verlet import VelocityVerlet
from ase import units

size = 3
    
# Set up a crystal
atoms = Diamond(directions=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                          symbol='Si',
                          size=(size, size, size),
                          pbc=True)

# Describe the interatomic interactions with the Effective Medium Theory
atoms.set_calculator(lammps['ref'])

np.random.seed(42)
# Set the momenta corresponding to T=300K
MaxwellBoltzmannDistribution(atoms, 1000 * units.kB)

# We want to run MD with constant energy using the VelocityVerlet algorithm.
dyn = VelocityVerlet(atoms, 1 * units.fs)  # 5 fs time step.

def printenergy(a):
    """Function to print the potential, kinetic and total energy"""
    epot = a.get_potential_energy() / len(a)
    ekin = a.get_kinetic_energy() / len(a)
    delta = np.max(np.abs(a.get_forces() - lammps['our'].get_forces(a)))
    print('Energy per atom: Epot = %.3feV  Ekin = %.3feV (T=%3.0fK)  '
          'Etot = %.3feV d=%.3e' % (epot, ekin, ekin / (1.5 * units.kB), epot + ekin, delta))

from ase.io.trajectory import Trajectory
traj = Trajectory('example.traj', 'w', atoms)
dyn.attach(traj.write, interval=1)

# Now run the dynamics
printenergy(atoms)
for i in range(100):
    dyn.run(1)
    printenergy(atoms)
