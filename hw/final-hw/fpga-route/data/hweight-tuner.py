import os
import subprocess
import sys

BENCHMARKS = ['boom_med_pb',
              'vtr_mcml',
              'rosetta_fd',
              'corundum_25g',
              'vtr_lu64peeng',
              'corescore_500',
              'corescore_500_pb',
              'mlcad_d181_lefttwo3rds',
              'koios_dla_like_large',
              'boom_soc',
              'ispd16_example2']

# BENCHMARKS = ['boom_soc',
#               'ispd16_example2']

# BENCHMARKS = ['vtr_mcml']

H_WEIGHTS = [1.0, 0.8, 0.6, 0.4]

REP = 1

if __name__ == '__main__':
    res = {bench: [] for bench in BENCHMARKS}
    for bench in BENCHMARKS:
        for hw in H_WEIGHTS:
            time_sum = 0
            wl_sum = 0
            wl_error = False
            for i in range(REP):
                with open('config.conf', 'w') as fout:
                    fout.write(f'h_weight={hw}')
                    
                os.system(f'make run-bench BENCHMARKS={bench} > make.log')
                # with open(f'make.log', 'w') as fout:
                #     p = subprocess.Popen(
                #         ['make', 'run-bench', f'BENCHMARKS={bench}'], stdout=fout, stderr=fout)
                #     p.wait()
                
                os.system(f'python contest/wirelength_analyzer/wa.py results/{bench}_aceroute.phys > results/{bench}_aceroute.wl')
                # with open(f'results/{bench}_aceroute.wl', 'w') as fout:
                #     p = subprocess.Popen(
                #         ['python', 'contest/wirelength_analyzer/wa.py', f'results/{bench}_aceroute.phys'], stdout=fout, stderr=fout)
                #     p.wait()
                
                # os.system(f'make run-bench BENCHMARKS={bench}')
                # os.system(
                #     f'python contest/wirelength_analyzer/wa.py results/{bench}_aceroute.phys > results/{bench}_aceroute.wl')
                with open(f'results/{bench}_aceroute.phys.log') as fin:
                    time = float(fin.readlines()[-1][23:])
                    time_sum += time
                if not wl_error:
                    with open(f'results/{bench}_aceroute.wl') as fin:
                        try:
                            wl = int(fin.readlines()[8][11:])
                            wl_sum += wl
                        except:
                            print('wl error')
                            wl_error = True
                            wl = -1
                with open(f'tuner_{sys.argv[1]}.log', 'a') as fout:
                    fout.write(
                        f'bench={bench},hw={hw},i={i},time={time},wl={wl}\n')
            res[bench].append(
                {'h_weight': hw, 'time': time_sum / REP, 'wl': wl_sum / REP})
    print(res)
    with open(f'tuner_{sys.argv[1]}.res', 'w') as fout:
        fout.write(str(res))