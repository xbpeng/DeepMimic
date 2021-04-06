import sys
import subprocess
from util.arg_parser import ArgParser
from util.logger import Logger
import DeepMimic_Optimizer

def main():
    # Command line arguments
    args = sys.argv[1:]
    arg_parser = ArgParser()
    arg_parser.load_args(args)

    num_workers = arg_parser.parse_int('num_workers', 1)
    assert(num_workers > 0)
	
    if (num_workers > 1):
        Logger.print('Running with {:d} workers'.format(num_workers))
        cmd = 'mpiexec -n {:d} python3 DeepMimic_Optimizer.py '.format(num_workers)
        cmd += ' '.join(args)
        Logger.print('cmd: ' + cmd)
        subprocess.call(cmd, shell=True)
    else:
        DeepMimic_Optimizer.main()
    return

if __name__ == '__main__':
    main()
