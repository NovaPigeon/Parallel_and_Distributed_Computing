# AceRoute

## How to run
Environment Setup
```bash
# Environment setup
apt -y update
apt -y install build-essential cmake zlib1g-dev curl wget
apt -y install openjdk-17-jdk

apt-get -y update
apt-get -y install git python3 pip pkg-config time libtinfo5
apt-get -y install liblog4cplus-dev ninja-build

pip3 install pycapnp 
pip3 install networkx

# Install capnproto
curl -O https://capnproto.org/capnproto-c++-1.0.1.tar.gz && \
    tar zxf capnproto-c++-1.0.1.tar.gz && \
    cd capnproto-c++-1.0.1 && \
    ./configure && \
    make -j6 check && \
    make install && \
    cd .. && \
    rm -rf capnproto-c++-1.0.1.tar.gz capnproto-c++-1.0.1

# download submodules
cd fpga-route
git submodule add https://github.com/Xilinx/fpga24_routing_contest contest
git submodule add https://github.com/taskflow/taskflow extern/taskflow
```


Build via CMake and Ninja. Device file `xcvu3p.device` is automatically generated and symlinked.
```bash
cd fpga-route/
make         # or "make build"
             # build the debug version
make release   # build the release version
```
To run the router, you need to provide the paths to input netlists, output netlist, and the config file (optional).
```bash
./<build or release>/aceroute.exe \
        -i <input_netlist.phys> \
        -o <output_netlist.phys> \
       [-c <config.cfg>] 

```
The config can specify all the entries in `config.h`. If not provided, or some entries are not specified, the default values will be used. 


To run AceRoute on the contest benchmarks, run:
```bash
make run-bench [BENCHMARKS="<design1> <design2> ..."] \  # default: all benchmarks
               [CONFIG="path/to/config.cfg"]          \
               [RESULT_DIR="path/to/result_dir/"]      \
               [-j <num_parallel_jobs>]                  # default: 1

# For example 
make run-bench BENCHMARKS=boom_med_pb
```

## How to evaluate your routing
```bash
cd ./data
python3 wl.py ../results
```
The critical path wirelength of your routing will be generated in the result directory.