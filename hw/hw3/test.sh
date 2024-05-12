#!/usr/bin/env bash
make mpi
make sequential
rm -rf *.log
for ((matrix_size=64; matrix_size<=4096; matrix_size*=2)); do
    echo "Matrix size: $matrix_size x $matrix_size"
    rm -rf a.m b.m
    python random_float_matrix.py $matrix_size $matrix_size >> a.m
    python random_float_matrix.py $matrix_size $matrix_size >> b.m
    
    for ((num_processes=2; num_processes<=matrix_size && num_processes<=128; num_processes*=2)); do
        echo "Number of MPI processes: $num_processes"
        
        mpiexec -n $num_processes ./bin/mpi a.m b.m 2>&1 >> mpi.log
    done

    ./bin/seq a.m b.m 2>&1 >> seq.log

    # Clean up matrices for next iteration
    rm -rf a.m b.m
done