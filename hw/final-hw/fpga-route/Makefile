BUILD_DIR=./build
RELEASE_DIR=./release
DEBUG_DIR=./debug
CONTEST_DIR=./contest
TASKFLOW_DIR=./extern/taskflow
DEVICE=xcvu3p.device
BENCHMARKS_URL=https://github.com/Xilinx/fpga24_routing_contest/releases/latest/download/benchmarks.tar.gz
BENCHMARKS_DIR=./benchmarks

RESULT_DIR ?= ./results
CONFIG ?= config.cfg

# List of all benchmarks (default to all)
BENCHMARKS ?= boom_med_pb \
			  boom_soc_v2 \
			  corescore_1700 \
			  corescore_500 \
			  mlcad_d181_lefttwo3rds \
			  mlcad_d181 \
			  vtr_mcml

# Choice of router (default to aceroute)
ROUTER ?= aceroute

# Make /usr/bin/time only print out wall-clock time in seconds
export TIME=Wall-clock time (sec): %e

# Existence of the VERBOSE environment variable indicates whether router/
# checker outputs will be displayed on screen
VERBOSE ?= 0
ifneq ($(VERBOSE), 0)
	log_and_or_display = 2>&1 | tee $(1)
else
	log_and_or_display = > $(1) 2>&1
endif

.PHONY: build release debug device taskflow run-bench download-bench

all: build

build: device taskflow
	mkdir -p ${BUILD_DIR} && cd ${BUILD_DIR} && \
		cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -G Ninja .. && ninja

release: device taskflow
	mkdir -p ${RELEASE_DIR} && cd ${RELEASE_DIR} && \
		cmake -DCMAKE_BUILD_TYPE=Release -G Ninja .. && ninja

debug: device taskflow
	mkdir -p ${DEBUG_DIR} && cd ${DEBUG_DIR} && \
		cmake -DCMAKE_BUILD_TYPE=Debug -G Ninja .. && ninja

submit: taskflow
	mkdir -p ${RELEASE_DIR} && cd ${RELEASE_DIR} && \
		cmake -DCMAKE_BUILD_TYPE=Release -G Ninja .. && ninja aceroute_par.exe && \
		cp aceroute_par.exe ../aceroute_submit.exe

device: ${DEVICE}

taskflow: ${TASKFLOW_DIR}/taskflow/taskflow.hpp

${TASKFLOW_DIR}/taskflow/taskflow.hpp:
	git submodule update --init --recursive ${TASKFLOW_DIR}

run-bench: ${BENCHMARKS:%=${RESULT_DIR}/%_${ROUTER}.phys}

${DEVICE}:
	git submodule update --init --recursive ${CONTEST_DIR}
	make -C ${CONTEST_DIR} $@ && \
		ln -s ${CONTEST_DIR}/$@

download-bench: ${BENCHMARKS_DIR}

${BENCHMARKS_DIR}:
	mkdir $@ && cd $@ && wget ${BENCHMARKS_URL} && \
		tar -xzf benchmarks.tar.gz

style:
	clang-format -i src/*.h src/*.cc

clean:
	rm -rf ${BUILD_DIR} ${RELEASE_DIR} ${DEBUG_DIR} *.exe *.phys *.log

cleanall: clean
	rm -rf ${RESULT_DIR} *.device *.device.ungzip

${RESULT_DIR}:
	mkdir -p $@

${BENCHMARKS_DIR}/%_unrouted.phys: download-bench

#### BEGIN ROUTER RECIPES

## ACEROUTE
${RESULT_DIR}/%_aceroute.phys: ${BENCHMARKS_DIR}/%_unrouted.phys release ${RESULT_DIR}
	-(./release/aceroute.exe -i $< -o $@ -c ${CONFIG}) $(call log_and_or_display,$@.log)
