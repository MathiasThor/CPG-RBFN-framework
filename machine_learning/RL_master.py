#!/usr/bin/env python

import atexit
import json
import math
import os
import os.path
import csv
import shutil
import sys, getopt
import time
import RL_plotter
import RL_PIBB
import RL_workerhandle
import RL_encoding
import numpy as np

def main(argv):
    global parameter_arr, sensor_parameter_arr, noise_arr, noise_arr_sensor
    start = time.time()

    ############################
    #  FILESYSTEM PARAMETERS   #
    ############################
    file_dir        = './../data/jobs'
    file_name       = file_dir+'/RL_job'
    file_answer_dir = file_dir+'/answers'
    file_answer_name= file_answer_dir+'/answer'

    ############################
    #   LEARNING PARAMETERS    #
    ############################
    rollouts        = 8      # Runs pr. iteration
    variance        = 0.08   # Variance on the parameter set
    init_var_boost  = 2      # This num is multiplied with the var in iteration 1
    iteration       = 0      # Number of iterations
    sim_length      = 6      # Length of one roll out in s.
    workers         = 4      # Workers available
    h               = 10     # Exploration constant
    decay           = 0.995  # Exploration decay constant
    best_max_fitness= -1000  # Maximum fitness for the run
    best_avg_fitness= -1000  # Maximum fitness for the run
    best_min_fitness= -1000  # Maximum fitness for the run
    max_iterations  = 100    # Infinity = -1
    rbfneurons      = 20     # default 20
    policy_selector = 2      # 1="only parameters", 2="only feedback", 3="both"
    behaviour_selector = "obstacle"  # "walk", "tilt", "direction", "obstacle", "roll", "all"

    if behaviour_selector == "walk":
        policy_selector = 1
        sim_length = 6
        variance = 0.02
    elif behaviour_selector == "obstacle":
        policy_selector = 2
        sim_length = 14
        variance = 0.02
    elif behaviour_selector == "obstacle_direct":
        policy_selector = 2
        sim_length = 12
        variance = 0.10
    elif behaviour_selector == "tilt":
        policy_selector = 2
        sim_length = 6
        variance = 0.10
    elif behaviour_selector == "direction":
        policy_selector = 2
        sim_length = 10
        variance = 0.02
    elif behaviour_selector == "high":
        policy_selector = 2
        sim_length = 6
        variance = 0.06
    elif behaviour_selector == "low":
        policy_selector = 2
        sim_length = 6
        variance = 0.06
    elif behaviour_selector == "narrow":
        policy_selector = 2
        sim_length = 6
        variance = 0.06
    elif behaviour_selector == "flip":
        policy_selector = 2
        sim_length = 12
        variance = 3.0
        decay = 0.970
    elif behaviour_selector == "pipe":
        policy_selector = 2
        sim_length = 14
        variance = 0.06
    elif behaviour_selector == "wall":
        policy_selector = 2
        sim_length = 12
        variance = 0.06
    elif behaviour_selector == "walknomed":
        policy_selector = 2
        sim_length = 6
        variance = 0.15

    robot    = "MORF"       # Only tested with MORF
    encoding = "indirect"   # Only tested with indirect

    myopts, args = getopt.getopt(sys.argv[1:], "e:r:i:t:")
    for o, a in myopts:
        if o == '-e':
            encoding = a
        elif o == '-r':
            rollouts = int(a)
        elif o == '-i':
            max_iterations = int(a)
        elif o == '-t':
            robot = a
        else:
            print("Usage: %s -i input -o output" % sys.argv[0])

    print("Encoding:  \t" + str(encoding))
    print("rollouts:  \t" + str(rollouts))
    print("iterations:\t" + str(max_iterations))
    print("robot:     \t" + str(robot))

    plotter         = RL_plotter.PLOTTER()
    workerhandle    = RL_workerhandle.WORKERHANLDE(workers, rollouts, sim_length, file_answer_dir, robot, policy_selector, behaviour_selector)
    pibb            = RL_PIBB.PIBB(rollouts, h, 1)

    atexit.register(cleanup, workerhandle)

    # PARAMETER SETUP FOR RBFN (no prior)
    encoder         = RL_encoding.encoder()
    init_parameter_set, init_sensor_parameter_set = encoder.get_init_parameter_set(robot, encoding, rbfneurons)

    # Initialize answer directory
    if os.path.exists(file_answer_dir):
        shutil.rmtree(file_answer_dir)

    os.mkdir(file_answer_dir)

    # Init log file as it does not exist
    log_progress(True, file_dir, 0, 0, 0, 0, 0, 0, 0)

    # Create first job for workers
    noise_arr = [[0]*len(init_parameter_set)] * rollouts
    noise_arr_sensor = [[0]*len(init_sensor_parameter_set)] * rollouts

    with open(file_name + '.json', 'w') as json_file:
        data = {}
        data['iteration']   = iteration
        data['rollout']     = 0
        data['rollouts']    = rollouts

        if policy_selector == 1 or policy_selector == 3:
            for k in range(rollouts):
                noise = np.random.normal(0, 0, len(init_parameter_set))
                data['noise_'+str(k)] = noise.tolist()
                noise_arr[k] = noise

        if policy_selector == 2 or policy_selector == 3:
            for k in range(rollouts):
                noise = np.random.normal(0, 0, len(init_sensor_parameter_set))
                data['noise_sensor_'+str(k)] = noise.tolist()
                noise_arr_sensor[k] = noise

        data['ParameterSet'] = init_parameter_set
        data['SensorParameterSet'] = init_sensor_parameter_set
        data['checked'] = encoding

        json.dump(data, json_file, indent=4, sort_keys=True)
        json_file.write("\n\n\n\n\n\n\n\n")

    parameter_arr = init_parameter_set
    sensor_parameter_arr = init_sensor_parameter_set

#####
    while True:
        # Reset arrays
        fitness_arr         = [-1.0]*rollouts
        fitness_arr_stab    = [-1.0]*rollouts
        fitness_arr_coll    = [-1.0]*rollouts
        fitness_arr_powr    = [-1.0]*rollouts
        fitness_arr_dist    = [-1.0]*rollouts
        fitness_arr_slip    = [-1.0]*rollouts
        distance_arr        = [-1.0]*rollouts
        energy_arr          = [-1.0]*rollouts

        print("\nIter.: " + str(iteration).zfill(3) + " → " + "Rollout: ", end="", flush=True)

        # Start Working
        dt_start = time.time()
        workerhandle.work()

        # Collect answers
        for k in range(rollouts):
            with open(file_answer_name + '_' + str(k) + '.json') as json_file:
                time.sleep(0.25)
                data = json.load(json_file)
                fitness_arr[k]      = data['FitnessValue']
                fitness_arr_stab[k] = data['Fitness_Stab']
                fitness_arr_coll[k] = data['Fitness_Coll']
                fitness_arr_powr[k] = data['Fitness_Powr']
                fitness_arr_dist[k] = data['Fitness_Dist']
                fitness_arr_slip[k] = data['Fitness_Slip']
                distance_arr[k]     = data['Distance']
                energy_arr[k]       = data['Energy']

        # Removes all files in 'file_answer_dir'
        shutil.rmtree(file_answer_dir)
        os.mkdir(file_answer_dir)

        # Run PIBB algorithm
        if policy_selector == 1 or policy_selector == 3:
            parameter_arr = pibb.step(fitness_arr, parameter_arr, noise_arr)
        if policy_selector == 2 or policy_selector == 3:
            sensor_parameter_arr = pibb.step(fitness_arr, sensor_parameter_arr, noise_arr_sensor)

        print(" (var: " + str(round(variance,4)) + ", dt: " + str(round(time.time()-dt_start,2)) + ", t: " + str(round(time.time()-start,2)) + ")", end="", flush=True)

        # Update exploration variance
        variance = decay * variance

        # Plot and save fitness info
        log_progress(False, file_dir, iteration, fitness_arr, fitness_arr_stab, fitness_arr_coll, fitness_arr_powr, fitness_arr_dist, fitness_arr_slip)
        plotter.plot(fitness_arr_stab, fitness_arr_coll, fitness_arr, fitness_arr_dist)

        # Backup and cleanup
        os.rename(file_name + '.json', file_name + "_" + str(iteration) + '.json')
        if max(fitness_arr) > best_max_fitness:
            best_max_fitness = max(fitness_arr)
            shutil.copyfile(file_name + "_" + str(iteration) + '.json', file_name + "_" + "best_max" + '.json')

        if sum(fitness_arr)/len(fitness_arr) > best_avg_fitness:
            best_avg_fitness = sum(fitness_arr)/len(fitness_arr)
            shutil.copyfile(file_name + "_" + str(iteration) + '.json', file_name + "_" + "best_avg" + '.json')

        if min(fitness_arr) > best_min_fitness:
            best_min_fitness = min(fitness_arr)
            shutil.copyfile(file_name + "_" + str(iteration) + '.json', file_name + "_" + "best_min" + '.json')

        if iteration >= max_iterations != -1:
            break
        else:
            iteration += 1

        # Generate new json for next iteration
        with open(file_name + '.json', 'w+') as json_file:
            data = {}
            data['iteration'] = iteration
            data['rollouts']  = rollouts

            if policy_selector == 1 or policy_selector == 3:
                for k in range(rollouts):
                    if iteration == 1:
                        noise = np.random.normal(0, variance*init_var_boost, len(init_parameter_set))
                    else:
                        noise = np.random.normal(0, variance, len(init_parameter_set))
                    data['noise_'+str(k)] = noise.tolist()
                    noise_arr[k] = noise


            if policy_selector == 2 or policy_selector == 3:
                for k in range(rollouts):
                    if iteration == 1:
                        noise = np.random.normal(0, variance*init_var_boost, len(init_sensor_parameter_set))
                    else:
                        noise = np.random.normal(0, variance, len(init_sensor_parameter_set))
                    data['noise_sensor_'+str(k)] = noise.tolist()
                    noise_arr_sensor[k] = noise
                #data['noise_sensor'] = data['noise_sensor_0']

            data['ParameterSet']        = parameter_arr
            data['SensorParameterSet']  = sensor_parameter_arr
            data['checked']             = encoding

            json.dump(data, json_file, indent=4, sort_keys=True)
            json_file.write("\n\n\n\n\n\n\n\n")


def log_progress(init, file_dir, _iteration, fitness_arr, fitness_arr_stab, fitness_arr_coll, fitness_arr_powr, fitness_arr_dist, fitness_arr_slip):
    if init:
        RL_log = open(file_dir + "/RL_log.txt", "w")
        RL_log.write("iteration\tfitness_arr_avg\tfitness_arr_max\tfitness_arr_min\tfitness_arr_stab_avg\tfitness_arr_stab_max\tfitness_arr_stab_min\tfitness_arr_coll_avg\tfitness_arr_coll_max\tfitness_arr_coll_min\tfitness_arr_powr_avg\tfitness_arr_powr_max\tfitness_arr_powr_min\tfitness_arr_dist_avg\tfitness_arr_dist_max\tfitness_arr_dist_min\tfitness_arr_slip_avg\tfitness_arr_slip_max\tfitness_arr_slip_min")
        RL_log.close()
    else:
        max_FT          = max(fitness_arr)
        max_FT_stab     = max(fitness_arr_stab)
        max_FT_coll     = max(fitness_arr_coll)
        max_FT_powr     = max(fitness_arr_powr)
        max_FT_dist     = max(fitness_arr_dist)
        max_FT_slip     = max(fitness_arr_slip)

        avg_FT          = sum(fitness_arr)/len(fitness_arr)
        avg_FT_stab     = sum(fitness_arr_stab)/len(fitness_arr_stab)
        avg_FT_coll     = sum(fitness_arr_coll)/len(fitness_arr_coll)
        avg_FT_powr     = sum(fitness_arr_powr)/len(fitness_arr_powr)
        avg_FT_dist     = sum(fitness_arr_dist)/len(fitness_arr_dist)
        avg_FT_slip     = sum(fitness_arr_slip)/len(fitness_arr_slip)

        min_FT         = min(fitness_arr)
        min_FT_stab    = min(fitness_arr_stab)
        min_FT_coll    = min(fitness_arr_coll)
        min_FT_powr    = min(fitness_arr_powr)
        min_FT_dist    = min(fitness_arr_dist)
        min_FT_slip    = min(fitness_arr_slip)

        RL_log = open('./../data/jobs' + "/RL_log.txt", "a")
        RL_log.write("\n%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f" % (_iteration, avg_FT, max_FT, min_FT, avg_FT_stab, max_FT_stab, min_FT_stab, avg_FT_coll, max_FT_coll, min_FT_coll, avg_FT_powr, max_FT_powr, min_FT_powr, avg_FT_dist, max_FT_dist, min_FT_dist, avg_FT_slip, max_FT_slip, min_FT_slip))
        RL_log.close()


def switch_encoding_light(old_encoding, new_encoding, _current_parameterset, _rollouts):
    if old_encoding == 'indirect' and new_encoding == 'sindirect':
        bc_enc = _current_parameterset[0:20]
        cf_enc = _current_parameterset[20:40]
        ft_enc = _current_parameterset[40:60]
        _param_set_new_enc = bc_enc + bc_enc + bc_enc + cf_enc + cf_enc + cf_enc + ft_enc + ft_enc + ft_enc

    elif old_encoding == 'indirect' and new_encoding == 'direct':
        _param_set_new_enc = [0]*(18*20)
        for x in range(3):
            start = x*6
            for y in range(6):
                _param_set_new_enc[start+y] = _current_parameterset[x]
    else:
        print('[ error] unknown encoding (1)')

    new_noise_arr = [[0]*len(_param_set_new_enc)] * _rollouts
    return _param_set_new_enc, new_noise_arr


def switch_encoding(old_encoding, new_encoding, _file_name, _dist_file_name):
    with open(_file_name) as json_file:
        data = json.load(json_file)
        _param_set_old_enc = data['ParameterSet']
        _rollouts          = data['rollouts']

    if old_encoding == 'indirect' and new_encoding == 'sindirect':
        bc_enc = _param_set_old_enc[0:20]
        cf_enc = _param_set_old_enc[20:40]
        ft_enc = _param_set_old_enc[40:60]
        _param_set_new_enc = bc_enc + bc_enc + bc_enc + cf_enc + cf_enc + cf_enc + ft_enc + ft_enc + ft_enc

    elif old_encoding == 'indirect' and new_encoding == 'direct':
        _param_set_new_enc = [0]*(18*20)
        for x in range(3):
            start = x*6
            for y in range(6):
                _param_set_new_enc[start+y] = _param_set_old_enc[x]
    else:
        print('[ error] unknown encoding (2)')

    new_noise_arr = [[0]*len(_param_set_new_enc)] * _rollouts

    return _param_set_new_enc, new_noise_arr


def cleanup(workerhand):
    workerhand.process_cleaner_all()
    print('\n')
    print('cleaned up!')


if __name__ == '__main__':
    main(sys.argv[1:])
