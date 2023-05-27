#!/usr/bin/env python3

from lava.proc.lif.process import LIF
from lava.lib.dnf.kernels.kernels import SelectiveKernel
from lava.lib.dnf.connect.connect import connect
from lava.lib.dnf.operations.operations import Convolution
from lava.lib.dnf.kernels.kernels import MultiPeakKernel
#import lava plot
#import lava.lib.dnf.utils.plotting.py

import numpy as np
import matplotlib.pyplot as plt
from lava.proc.io.sink import RingBuffer
from lava.magma.core.run_configs import Loihi1SimCfg,Loihi2HwCfg
from lava.magma.core.run_conditions import RunSteps
from lava.proc.lif.process import LIF
from lava.proc.dense.process import Dense
from lava.proc.monitor.process import Monitor
from lava.lib.dnf.connect.connect import connect
from lava.lib.dnf.inputs.gauss_pattern.process import GaussPattern
from lava.lib.dnf.inputs.rate_code_spike_gen.process import \
    RateCodeSpikeGen
from lava.lib.dnf.operations.operations import (
    Weights,
    ExpandDims,
    ReorderDims,
    ReduceAlongDiagonal,
    ExpandAlongDiagonal,
    Flip)

from utils import plot_1d, animated_1d_plot

shape = (17,)
shape_target = (8,)
shape_reference = (17,)
shape_relational = tuple(np.array(shape_target) + np.array(shape_reference) - 1)
num_time_steps = 100

# Set up Processes that provide Input to the Relational Network
gauss_target = GaussPattern(shape=shape_target, amplitude=5000, mean=5, stddev=0.1)
sg_target = RateCodeSpikeGen(shape=shape_target)

gauss_reference = GaussPattern(shape=shape_reference, amplitude=5000, mean=3, stddev=0.1)
sg_reference = RateCodeSpikeGen(shape=shape_reference)

input_dense_target = Dense(weights=np.eye(shape_target[0]) * 25)
input_dense_reference = Dense(weights=np.eye(shape_reference[0]) * 25)

# Set up Processes that constitute the Relational Network
target = LIF(shape=shape_target, du=409, dv=2045, vth=200)
reference = LIF(shape=shape_reference, du=409, dv=2045, vth=200)
transformation = LIF(shape=shape_target + shape_reference, du=409, dv=2045, vth=200)

# Set up RingBuffer Processes to store Variable Evolutions/Spike Activity (later used for plotting)
py_rec_target = RingBuffer(shape=shape_target, buffer=num_time_steps)
py_rec_reference = RingBuffer(shape=shape_reference, buffer=num_time_steps)
py_rec_transformation = RingBuffer(shape=shape_target+shape_reference, buffer=num_time_steps)

# Make Connections
# Connect Gauss Pattern to Spike Generators
gauss_target.a_out.connect(sg_target.a_in)
gauss_reference.a_out.connect(sg_reference.a_in)


# In Simulation we can directly connect the generated spike input to the corresponding dense processes.
sg_target.s_out.connect(input_dense_target.s_in)
sg_reference.s_out.connect(input_dense_reference.s_in)

input_dense_target.a_out.connect(target.a_in)
input_dense_reference.a_out.connect(reference.a_in)

# Make Relational Network Connections
connect(target.s_out, transformation.a_in,
        ops=[ExpandDims(new_dims_shape=shape_reference[0]),
             Weights(10)])
connect(reference.s_out, transformation.a_in,
        ops=[Flip(),
             ExpandDims(new_dims_shape=shape_target[0]),
             ReorderDims(order=(1, 0)),
             Weights(10)])

# For all LIF populations we want to read out spike rates. Except for the Transformation field,
# for which we later plot the voltage.
target.s_out.connect(py_rec_target.a_in)
reference.s_out.connect(py_rec_reference.a_in)
monitor_transformation = Monitor()
monitor_transformation.probe(transformation.v, num_time_steps)

# Execution
run_cfg=Loihi1SimCfg(select_tag="fixed_pt")
try:
    target.run(condition=RunSteps(num_steps=num_time_steps),
               run_cfg=run_cfg)

    # Extract data from RingBuffers
    data_target = py_rec_target.data.get().transpose()
    data_reference = py_rec_reference.data.get().transpose()
    #data_relational = py_rec_relational.data.get().transpose()
    
    # Variable probing 
    # The monitor process stores the data in a dictionary, where the key is the name of the process (CP)
    data_transformation = monitor_transformation.get_data() \
    [transformation.name][transformation.v.name].astype(float)

finally:
    target.stop()

# Plot data ------------------------------------------------------------------------------------------------------
_num_neurons_target = shape_target[0]
_num_neurons_reference = shape_reference[0]
_num_neurons_relational = shape_relational[0]

plt.figure(figsize=(10, 10))
ax0 = plt.subplot(4, 1, 1)
ax0.plot(np.mean(data_target[50:70, :], axis=0))
ax0.set_xlabel('Target neuron idx')
ax0.set_ylabel('Spike rate')
ax0.set_xticks(range(_num_neurons_target))
ax0.set_title('Target')

ax1 = plt.subplot(4, 1, 2)
ax1.plot(np.mean(data_reference[50:70, :], axis=0))
ax1.set_xlabel('Reference neuron idx')
ax1.set_ylabel('Spike rate')
ax1.set_xticks(range(_num_neurons_reference))
ax1.set_title('Reference')

ax2 = plt.subplot(4, 1, 3)
ax2.imshow(np.mean(data_transformation[50:70, :],
                   axis=0).reshape(shape_target + shape_reference))
ax2.set_title('Transformation (Voltage)')
ax2.set_xlabel('Reference position')
ax2.set_ylabel('Target position')
ax2.set_xticks(range(0, _num_neurons_reference, 5))
ax2.set_yticks(range(0, _num_neurons_target, 5))

# ax3 = plt.subplot(4, 1, 4)
# ax3.plot(np.mean(data_relational[50:70], axis=0))
# ax3.set_xlabel('Relative position')
# ax3.set_ylabel('Spike rate')
# ax3.set_title('Relational')
# plt.xticks(ticks=range(_num_neurons_relational),
#            labels=range(-int(_num_neurons_relational/2),
#                         int(_num_neurons_relational/2) + 1))

# ax3 = plt.subplot(4, 1, 4)
# ax3.plot1d()

plt.tight_layout()
plt.show()

# Generate a raster plot from the probed data
# plot_1d(data_transformation,
#         data_target,
#         data_reference)