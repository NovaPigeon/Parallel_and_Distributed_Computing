#!/usr/bin/env bash
mkdir results
for i in {0..7};do
    omniperf analyze -p workloads/reduce_v$i/MI100 > results/v$i.log
    echo "v$i generated"
done