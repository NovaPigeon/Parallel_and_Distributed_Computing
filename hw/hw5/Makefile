APP=reduction
HIP_CPU=../HIP-CPU

cpu:
	g++ -o ${APP} ${APP}.cpp -std=c++17 -I${HIP_CPU}/include -ltbb -pthread

gpu:
	hipcc -o ${APP} ${APP}.cpp -std=c++17

format:
	clang-format -i *.cpp

perf:
	for i in `seq 0 7`; do \
		omniperf profile -n reduce_v$$i -- ./reduction $$i; \
		omniperf analyze -p workloads/reduce_v$$i/MI100 -b 2 ; \
		done

clean:
	rm -f ${APP}
	rm -rf workloads/