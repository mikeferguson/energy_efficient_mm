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

import pylab

def subsample(data, n):
    new_data = list()
    i = 0
    for d in data:
        if i % 2 == 0:
            new_data.append(d)
        else:
            new_data[-1] = (new_data[-1]+d)/2.
        i += 1
    if len(new_data) < n:
        return new_data
    return subsample(new_data, n)

class JointPowerData:

    def __init__(self, name):
        self.name = name
        self.effort = list()
        self.power = list()  # instantaneous power usage
        self.energy = list() # energy used during timestep 

    def add_observation(self, velocity, effort, dt):
        power = abs(velocity * effort)
        self.effort.append(abs(effort))
        self.power.append(power)
        self.energy.append(power * dt)

    def get_total_energy(self):
        return sum(self.energy)

    def get_average_power(self):
        return sum(self.power)/len(self.power)

    def plot_power(self, samples = 1000, timestep = 1/250.):
        power = subsample(self.power, samples)
        time = range(len(power))
        time = [t * timestep for t in time]
        pylab.plot(time, power, "-")
        pylab.xlabel("Time (s)")
        pylab.ylabel("Power (W)")
        pylab.title("Joint '%s' Power Usage" % self.name)
        pylab.show()

    def get_power_data(self, samples = 1000):
        return subsample(self.power, samples)

class JointPowerGroup:

    def __init__(self, joints):
        self.joints = joints

    def plot_power(self):
        pass

class BreakerPowerData:

    def __init__(self, name):
        self.name = name
        self.current = list()
        self.power = list()
        self.energy = list()

    def add_observation(self, voltage, current, dt):
        self.current.append(current)
        self.power.append(voltage * current)
        self.energy.append(voltage * current * dt)

    def get_total_energy(self):
        return sum(self.energy)

    def get_average_power(self):
        return sum(self.power)/len(self.power)

    def plot_power(self, samples = 1000, timestep = 1/250.):
        power = subsample(self.power, samples)
        time = range(len(power))
        time = [t * timestep for t in time]
        pylab.plot(time, power, "-")
        pylab.xlabel("Time (s)")
        pylab.ylabel("Power (W)")
        pylab.title("%s Breaker Power Usage" % self.name)
        pylab.show()

    def get_power_data(self, samples = 1000):
        return subsample(self.power, samples)
        
