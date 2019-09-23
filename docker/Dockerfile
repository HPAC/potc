FROM ubuntu:latest

RUN apt-get clean
RUN apt-get update
RUN apt-get install -qy build-essential
COPY lammps-stable.tar.gz /root/
RUN tar xfz /root/lammps-stable.tar.gz -C /root 
RUN mv /root/lammps-*/ /lammps/
RUN (cd /lammps/src; make yes-MANYBODY)
RUN (cd /lammps/src; make serial)
RUN apt-get -qy install git
RUN apt-get -qy install gdb
