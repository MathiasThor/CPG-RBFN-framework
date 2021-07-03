#!/usr/bin/env python

import subprocess
import time
import os
import os.path


class WORKERHANLDE(object):
    def __init__(self, _workers, _rollouts, _sim_length, _file_answer_dir, _robot, _optimize_selector, _behaviour_selector):
        tmp = subprocess.Popen('ls', stdout=subprocess.DEVNULL, shell=False)
        self.worker_pool        = [tmp] * _workers
        self.workers            = _workers
        self.rollouts           = _rollouts
        self.sim_length         = _sim_length
        self.file_answer_dir    = _file_answer_dir
        self.iteration_counter  = 0
        if _robot == "MORF":
            self.job_task = "./../interfaces/morf/sim/build_dir/bin/morf_controller"
        elif _robot == "ALPHA":
            self.job_task = "./../interfaces/alpha/sim/build_dir/bin/alpha_controller"
        elif _robot == "LAIKAGO":
            self.job_task = "./../interfaces/laikago/sim/build_dir/bin/laikago_controller"
        self.optimize_selector = _optimize_selector
        self.behaviour_selector = _behaviour_selector

    def process_cleaner(self, process):
        process.terminate()
        time.sleep(0.29)
        process.kill()

    def process_cleaner_all(self):
        for p in self.worker_pool:
            p.terminate()
        time.sleep(0.29)
        for p in self.worker_pool:
            p.kill()

    def work(self):
        if self.iteration_counter % 5 != 0 and self.iteration_counter != 0:
            blackout = "1" # True
        else:
            blackout = "0" # False

        # Run roll outs on the available workers
        rollout = 0
        working = True
        workerJobList = [-1] * self.workers
        self.iteration_counter = self.iteration_counter+1
        while working:
            for worker in range(self.workers):
                if not working:
                    break
                time.sleep(0.1)
                poll = self.worker_pool[worker].poll()
                if poll is not None:
                    workerJobList[worker] = rollout
                    job_description = [self.job_task, str(worker+1), str(rollout), str(self.sim_length), str(blackout), str(self.optimize_selector), str(self.behaviour_selector)]
                    devnull = open(os.devnull, 'w')
                    self.worker_pool[worker] = subprocess.Popen(job_description, bufsize=0)#, stdout=devnull)
                    rollout = rollout+1
                    print(str(rollout) + " ", end="", flush=True)
                    if rollout >= self.rollouts:
                        working = False
                        break

        # Wait for any remaining workers to finish
        timeout_answer = 0
        timeout_worker = 0
        answer_files = len([name for name in os.listdir(self.file_answer_dir) if os.path.isfile(os.path.join(self.file_answer_dir, name))])
        while answer_files < self.rollouts:
            time.sleep(0.1)
            answer_files = len([name for name in os.listdir(self.file_answer_dir) if os.path.isfile(os.path.join(self.file_answer_dir, name))])

            # Detect missing answers
            if timeout_answer > 1200:
                existing_answer_files = [f for f in os.listdir(self.file_answer_dir) if os.path.isfile(os.path.join(self.file_answer_dir, f))]
                expected_answers = ["where_is_my_cake"] * self.rollouts
                for t in range(self.rollouts):
                    expected_answers[t] = "answer_" + str(t) + ".json"

                missing = [i for i in expected_answers if i not in existing_answer_files]
                print("[ERROR] Missing answer: ", missing)

                #Generate missing file
                existing_file = self.file_answer_dir + '/' + existing_answer_files[0]
                for file in missing:
                    missing_file = self.file_answer_dir + '/' + file
                    os.popen('cp ' + existing_file + ' ' + missing_file)
                timeout_answer = 0
            else:
                timeout_answer = timeout_answer + 1

            # Detect broken workers
            if timeout_worker > 1200:
                # Detect specific broken worker
                for worker in range(self.workers):
                    poll = self.worker_pool[worker].poll()
                    if poll is None:
                        print("[ERROR] Stopping broken worker")
                        print("[ERROR] Missing answer is: ", str(workerJobList[worker]))
                        # Stop broken worker
                        self.process_cleaner(self.worker_pool[worker])
                        time.sleep(1)
                        # Start broken worker
                        job_description = [self.job_task, str(worker+1), str(rollout), str(self.sim_length), str(blackout), str(self.optimize_selector), str(self.behaviour_selector)]
                        devnull = open(os.devnull, 'w')
                        self.worker_pool[worker] = subprocess.Popen(job_description, bufsize=0, stdout=devnull)
                        # Restart detector
                        timeout_worker = 0
            else:
                timeout_worker = timeout_worker + 1

        # Kill zombie workers "Zombies eat brains, you are safe"
        self.process_cleaner_all()
