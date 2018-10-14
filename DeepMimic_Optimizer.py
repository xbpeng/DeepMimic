import numpy as np
import sys
from env.deepmimic_env import DeepMimicEnv
from learning.rl_world import RLWorld
from util.logger import Logger
from DeepMimic import update_world, update_timestep, build_world
import util.mpi_util as MPIUtil

args = []
world = None

def run():
    global update_timestep
    global world

    done = False
    while not (done):
        update_world(world, update_timestep)

    return

def shutdown():
    global world

    Logger.print('Shutting down...')
    world.shutdown()
    return

def main():
    global args
    global world

    # Command line arguments
    args = sys.argv[1:]

    world = build_world(args, enable_draw=False)

    run()
    shutdown()

    return

if __name__ == '__main__':
    main()