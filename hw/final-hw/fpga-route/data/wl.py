import sys
import os
import fnmatch
import random
from multiprocessing import Pool

sys.path.append(os.path.join(os.path.dirname(__file__), 'wirelength_analyzer'))
saved_stdout = sys.stdout
from wa import WirelengthAnalyzer

def eval_wl(netlist, result_file):
    with open(result_file, 'w') as sys.stdout:
        wa = WirelengthAnalyzer(netlist)
        wa.find_critical_wirelength()

def find_filename(dir, pattern):
    for root, dirs, files in os.walk(dir):
        for name in files:
            if fnmatch.fnmatch(name, pattern):
                filename = os.path.join(root, name)
                yield filename

def run(netlist):
    result_file = os.path.join(netlist + '.wl')
    if os.path.exists(result_file):
        with open(result_file, 'r') as res:
            line = res.readline()
            if line.startswith("Critical Path Wirelength:"):
                return
    sys.stdout = saved_stdout
    print('Evaluating wirelength for', netlist)
    eval_wl(netlist, result_file)


if __name__ == '__main__':
    if sys.argv[1:]:
        results_dir = sys.argv[1]
    else:
        results_dir = os.path.join(os.path.dirname(__file__), '..', 'results')
        
    netlists = [netlist for netlist in find_filename(results_dir, '*.phys')]
    random.shuffle(netlists)
    
    pool = Pool(processes=8)
    pool.map(run, netlists)
    pool.close()
    pool.join()
