import sys
import os
import numpy as np
import ase
import ase.calculators.lammpslib as ase_lmp


def generate_system_positions(dim, num, min_dist, species):
    pos = np.random.random((num, 3)) * dim
    exclude_pos = set()
    atoms = ase.Atoms(
        np.random.choice(species, pos.shape[0]),
        positions=pos,
        cell=[dim, dim, dim],
        pbc=True)
    dists = atoms.get_all_distances(mic=True)
    for i in range(num):
        if i in exclude_pos:
            continue
        for j in range(num):
            if j <= i:
                continue
            if j in exclude_pos:
                continue
            if dists[i, j] < min_dist:
              exclude_pos.add(j)
    return atoms[list(set(range(num)) - exclude_pos)]

SYSTEM_DIM = 8
SYSTEM_MIN = 1.0
SYSTEM_COUNT = 200
REPEAT = 1000
MAX_FAULTS = 10

lmp_dir = os.path.abspath('lammps-30Mar18')

sys.path.append(os.path.join(lmp_dir, 'python'))
print('PID', os.getpid()) # in case of debugging
if len(sys.argv) > 2:
    input()

if sys.argv[1] == 'tersoff':
    cmds = {'ref':
            ["pair_style tersoff",
             "pair_coeff * * %s/potentials/Si.tersoff Si" % lmp_dir],
            'our':
            ["pair_style pot/gen 3.2",
             "pair_coeff * * potentials/Si.tersoff.potc-param Si"]}
    SPECIES = ['Si']
elif sys.argv[1] == 'rebo':
    cmds = {'ref':
            ["pair_style rebo",
             "pair_coeff * * %s/potentials/CH.airebo C H" % lmp_dir],
            'our':
            ["pair_style rebo/gen 6.0",
             "pair_coeff * * potentials/CH.rebo.potc-param C H"]}
    SPECIES = ['C', 'H']
    

default_header = ['units metal', 'atom_style atomic', 'atom_modify map array sort 0 0'] 

print('Initializing...')

lammps = {}
for name, cmd in cmds.items(): # collect calculators for all the different code variants
    header = default_header
    lammps[name] = ase_lmp.LAMMPSlib(
        lmpcmds=cmd, log_file='test.log', lammps_name='serial', keep_alive=True, lammps_header=header)

print('Running...')

def check_config(atoms):
    epr = lammps['ref'].get_potential_energy(atoms)
    epo = lammps['our'].get_potential_energy(atoms)
    fpr = lammps['ref'].get_forces(atoms)
    fpo = lammps['our'].get_forces(atoms)
    de = np.abs(epr - epo) / np.spacing(np.maximum(epr, epo)) * np.spacing(1)
    df = np.abs(np.max(fpr - fpo))
    df = np.max(np.abs(fpr - fpo) / np.spacing(np.maximum(fpr, fpo))) * np.spacing(1)
    if de > 1e-10 or df > 1e-9:
        return True, "{:4} n={:3} de={:5.2e} df={:5.2e} epr={:5.2e} epo={:5.2e} mfpr={:5.2e} mfpo={:5.2e}".format(r, len(atoms), de, df, epr, epo, np.max(np.abs(fpr)), np.max(np.abs(fpo)))
    else:
        return False, "{:4} {:3} de={:5.2e} df={:5.2e}".format(r, len(atoms), de, df)

def reduce_config(config):
    recurse = True
    while recurse:
        recurse = False
        i = 0
        while i < len(config) and len(config) > 1:
            #next_config = config.copy()
            next_symbols = [a.symbol for a in config]
            del next_symbols[i]
            next_positions = [a.position for a in config]
            del next_positions[i]
            next_config = ase.Atoms(
                next_symbols,
                positions=next_positions,
                cell=config.cell,
                pbc=config.pbc)
            #del next_config[i]
            err, out = check_config(next_config)
            if err:
                recurse = True
                config = next_config
            else:
                i += 1
        err, out = check_config(config)
        print(out)
    return config

def write_input(config):
    pass

def detail_config(atoms):
    print(check_config(atoms))
    epr = lammps['ref'].get_potential_energy(atoms)
    epo = lammps['our'].get_potential_energy(atoms)
    fpr = lammps['ref'].get_forces(atoms)
    fpo = lammps['our'].get_forces(atoms)
    print(atoms)
    print(atoms.positions)
    print(atoms.get_all_distances(mic=True))
    print('epr', epr, 'epo', epo)
    print(fpr)
    print(np.sum(fpr, axis=0))
    print('Our')
    print(fpo)
    print(np.sum(fpo, axis=0))

faults = 0
min_fault = None

for r in range(REPEAT):
    atoms = generate_system_positions(SYSTEM_DIM, SYSTEM_COUNT, SYSTEM_MIN, SPECIES)
    err, out = check_config(atoms)
    print(out)
    if err:
        atoms = reduce_config(atoms)
        if not min_fault or len(atoms) < len(min_fault):
            min_fault = atoms
        faults += 1
        if faults > MAX_FAULTS:
            break
if faults > 0:
    detail_config(min_fault)
