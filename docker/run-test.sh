sudo stdbuf -i0 -e0 docker run -v /home/markus/Documents/potc:/potc --workdir /potc/docker --rm plain-lammps /potc/docker/test.sh | tee -a log
