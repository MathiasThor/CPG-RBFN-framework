#!/usr/bin/env python

import rospy
import time
import sys
import signal
from std_msgs.msg import Bool, Int32, Float32, Float32MultiArray


class REPEATER(object):

    def __init__(self, sim_num, roll_num):

        nameAddOn = str(sim_num)

        if sim_num == -1:
            nameAddOn = ''

        self.blackout_pub       = rospy.Publisher('/sim_control'+nameAddOn+'/blackout',             Bool, queue_size=1)
        self.terminate_pub      = rospy.Publisher('/sim_control'+nameAddOn+'/terminateController',  Bool, queue_size=1)
        self.start_pub          = rospy.Publisher('/sim_control'+nameAddOn+'/startSimulation',      Int32, queue_size=1)
        self.stop_pub           = rospy.Publisher('/sim_control'+nameAddOn+'/stopSimulation',       Bool, queue_size=1)
        self.restart_pub        = rospy.Publisher('/sim_control'+nameAddOn+'/pauseSimulation',      Int32, queue_size=1)
        self.state_sub          = rospy.Subscriber('/sim_control'+nameAddOn+'/simulationState',     Int32, self.state_cb)
        self.simTime_sub        = rospy.Subscriber('/sim_control'+nameAddOn+'/simulationTime',      Float32, self.simTime_cb)
        self.testParameters_sub = rospy.Subscriber('/sim_control'+nameAddOn+'/testParameters',      Float32MultiArray, self.testParameters_cb)

        self.stop_msg       = Bool()
        self.stop_msg.data  = True
        self.start_msg      = Int32()
        self.start_msg      = roll_num
        self.BO_msg         = Bool()
        self.BO_msg.data    = False
        self.state          = 0
        self.simTime        = 0.0
        self.circleBreak    = 0

        # Wait for subscriber to connect
        while self.stop_pub.get_num_connections() == 0 or self.start_pub.get_num_connections() == 0:
            time.sleep(0.1)

    def state_cb(self, msg):
        self.state = msg.data

    def simTime_cb(self, msg):
        self.simTime = msg.data

    def testParameters_cb(self, msg):
        self.circleBreak = msg.data[12]

    def blackout_sim(self):
        self.blackout_pub.publish(self.BO_msg)

    def restart_sim(self):
        self.restart_pub.publish(self.start_msg)

    def stop_sim(self):
        self.stop_pub.publish(self.stop_msg)

    def stop_sim_n_exit(self):
        self.stop_pub.publish(self.stop_msg)
        time.sleep(0.1)
        print("STOP n EXIT")
        sys.exit()

    def terminate_sim(self):
        self.terminate_pub.publish(self.stop_msg)

    def start_sim(self):
        self.start_pub.publish(self.start_msg)

    def wait_simtime(self, time_to_wait):
        time_start = 0
        while self.simTime < time_to_wait:
            time.sleep(0.1)
            if self.state != 1:
                self.start_sim()
            else:
                pass

        self.stop_sim()
        while self.state != 0:
            time.sleep(0.1)
            self.stop_sim()
            pass

    def reset_sim(self):
        if self.state != 0:
            self.stop_sim()
            self.state = 1
            while self.state != 0:
                pass

        time.sleep(0.75)

        self.start_sim()
        self.state = 0
        while self.state != 1:
            pass

        self.simTime = 0


def cleanup(*args):
    node.stop_sim()
    sys.exit()


def main(argv):
    global node

    if len(sys.argv) < 4+1:
        print("Please provide following arguments: \nTrial length in seconds (int/arg1) \nNumber of trials (int/arg2) \nSimulation number (int/arg3) \nRollout number (int/arg4)")
        exit()

    if sys.argv[3] == "-1":
        nameAddOn = ""
    else:
        nameAddOn = sys.argv[3]

    rospy.init_node('repeater' + nameAddOn)
    node = REPEATER(int(sys.argv[3]), int(sys.argv[4]))

    #signal.signal(signal.SIGINT, node.stop_sim_n_exit())
    #signal.signal(signal.SIGTERM, node.stop_sim_n_exit())

    sleep_time = int(sys.argv[1])
    trials = int(sys.argv[2])
    trial_count = 1

    start = time.time()

    #while not rospy.is_shutdown() and trial_count <= trials:
    # print "  Trial ", trial_count, " of ", trials
    startTrail = time.time()
    node.restart_sim()
    # print "  Time spend reset:", round(time.time()-startTrail,2), "seconds"
    startTrail = time.time()

    if str(sys.argv[5]) == "True":
        node.blackout_sim()

    node.wait_simtime(sleep_time)
    # print "  Time spend simul:", round(time.time()-startTrail,2), "seconds"
    trial_count = trial_count+1

    sys.exit()

    # end = time.time()
    # real_time_spend = trials*sleep_time
    # time_spend = end-start
    # if real_time_spend > time_spend:
    #     procentage = ((real_time_spend/time_spend)*100)-100
    # else:
    #     procentage = ((time_spend/real_time_spend)*100)*-1
    # print " "
    # print "Real time: ", round(real_time_spend,2), "seconds"
    # print "Time spend:", round(time_spend,2), "seconds"
    # print "Difference:", round(procentage,2), "%"
    # print " "


if __name__ == '__main__':
    main(sys.argv[1:])
