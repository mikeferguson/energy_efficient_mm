#!/usr/bin/env python

# Copyright (c) 2014, Michael E. Ferguson
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE 
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import rosbag
from energy_efficient_mm.power_data import *

base_joint_names = ["base_l_wheel_joint", "base_r_wheel_joint"]

if __name__ == "__main__":
    joints = dict()

    joints["shoulder_pan_joint"] = JointPowerData("shoulder_pan_joint")

    main_breaker = BreakerPowerData("Main")
    comp_breaker = BreakerPowerData("Computer")

    # Timing
    last_timestamp = None
    first_timestamp = None

    # Load bag and analyze
    bag = rosbag.Bag(sys.argv[1])
    for topic, msg, t in bag.read_messages():

        if topic == "/robot_debug" or topic == "robot_debug":
            if last_timestamp == None:
                last_timestamp = msg.info.header.stamp
                first_timestamp = msg.info.header.stamp

            dt = (msg.info.header.stamp - last_timestamp).to_sec()

            for joint in msg.joints:
                try:
                    joints[joint.name].add_observation(joint.actual_velocity, joint.actual_effort, dt)
                except:
                    pass # not a joint we care about

            main_breaker.add_observation(msg.info.supply_voltage, msg.charger.system_current, dt)
            comp_breaker.add_observation(msg.info.supply_voltage, msg.breakers[1].state.current, dt)                           
            
            last_timestamp = msg.info.header.stamp

    power = main_breaker.get_power_data()
    time = range(len(power))
    time = [t * 1/250. for t in time]
    pylab.plot(time, power, "-b", label="Main Breaker")
    power = comp_breaker.get_power_data()
    time = range(len(power))
    time = [t * 1/250. for t in time]
    pylab.plot(time, power, "-g", label="Computer Breaker")
    power = joints["shoulder_pan_joint"].get_power_data()
    time = range(len(power))
    time = [t * 1/250. for t in time]
    pylab.plot(time, power, "-r", label="Shoulder Pan Joint")
    pylab.xlabel("Time (s)")
    pylab.ylabel("Power (W)")
    pylab.title("Power Usage")
    pylab.show()

