import sys
import os
import ase
import ase.calculators.lammpslib as ase_lmp
from ase.io.trajectory import Trajectory
import numpy as np

lmp_dir = os.path.abspath('lammps-30Mar18')

sys.path.append(os.path.join(lmp_dir, 'python'))
print('PID', os.getpid()) # in case of debugging
if len(sys.argv) > 1:
  input()

cmds = {'ref':
        ["pair_style tersoff",
         "pair_coeff * * %s/potentials/Si.tersoff Si" % lmp_dir],
        'our':
        ["pair_style pot/gen 3.2",
         "pair_coeff * * potentials/Si.tersoff.potc-param Si"]}

default_header = ['units metal', 'atom_style atomic', 'atom_modify map array sort 0 0'] 

lammps = {}
for name, cmd in cmds.items(): # collect calculators for all the different code variants
    header = default_header
    lammps[name] = ase_lmp.LAMMPSlib(
        lmpcmds=cmd, log_file='test.log', lammps_name='serial', keep_alive=True, lammps_header=header)

de = []
df = []
def printenergy(a):
    """Function to print the potential, kinetic and total energy"""
    eref = lammps['ref'].get_potential_energy(a)
    eour = lammps['our'].get_potential_energy(a)
    delta = np.max(np.abs(lammps['ref'].get_forces(a) - lammps['our'].get_forces(a)))
    de.append(np.abs(eref-eour))
    df.append(delta)
    #print('de=%.3e df=%.3e' % (np.abs(eref - eour), delta))

traj = Trajectory('example.traj')

# Now run the dynamics
for atoms in traj:
    printenergy(atoms)

print('de=%.3e df=%.3e' % (np.max(de), np.max(df)))
