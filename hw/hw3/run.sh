#!/usr/bin/env bash
rm -rf a.m b.m
python random_float_matrix.py 32 32 >> a.m
python random_float_matrix.py 32 32 >> b.m

make mpi
make sequential

mpiexec -n 2 ./bin/mpi a.m b.m
./bin/seq a.m b.m
