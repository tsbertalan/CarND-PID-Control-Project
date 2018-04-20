import matplotlib.pyplot as plt
import numpy as np
from sys import argv

def line(l):
    if '|' in l:
        l = l.split('|')
        assert len(l) == 2
        return long(l[0]), l[1].strip()
    else:
        return None, l

nparam = 3

if len(argv) == 1:
    fname = 'twiddle.out'
else:
    fname = argv[1]

with open(fname, 'r') as f:
    lines = f.readlines()

parameter_history = []
parameter_history_times = []

diff_parameter_history = []
diff_parameter_history_times = []

std_history = []
std_history_times = []

mae_history = []
mae_history_times = []

obj_history = []
obj_history_times = []

cycle_start_times = []

for l in lines:
    t_clock, l = line(l)

    if t_clock is None:
        continue

    elif 'dp =' in l:
        exec(l)
        if len(dp) == nparam:
            diff_parameter_history.append(dp)
            diff_parameter_history_times.append(t_clock)

    elif 'p =' in l:
        exec(l)
        if len(p) == nparam:
            parameter_history.append(p)
            parameter_history_times.append(t_clock)

    elif '>Mean absolute error' in l:
        mae_history.append(
            float(l.split('=')[-1])
        )
        mae_history_times.append(t_clock)

    elif '>Stdd error' in l:
        std_history.append(
            float(l.split('=')[-1])
        )
        std_history_times.append(t_clock)

    elif 'objective = ' in l:
        obj_history.append(
            float(l.split('=')[-1])
        )
        obj_history_times.append(t_clock)

    elif 'Twiddle iteration' in l:
        cycle_start_times.append(t_clock)

    elif 'i=' in l:
        i_attempt = 0
        i_param = int(l.replace('-', '').replace('i=', ''))

diff_parameter_history = np.array(diff_parameter_history)
parameter_history = np.array(parameter_history)

ar = lambda v: np.array(v).astype(float)

parameter_history_times = ar(parameter_history_times)
diff_parameter_history_times = ar(diff_parameter_history_times)
obj_history_times = ar(obj_history_times)
mae_history_times = ar(mae_history_times)
std_history_times = ar(std_history_times)
cycle_start_times = ar(cycle_start_times)

Ts = (
    parameter_history_times, diff_parameter_history_times, 
    obj_history_times, mae_history_times, std_history_times, cycle_start_times
)
t0 = min([a.min() for a in Ts])
for a in Ts:
    a[:] -= t0
    # convert ms to min
    a[:] /= 1000 * 60

fig, ax = plt.subplots()
ax.plot(mae_history_times, mae_history, color='black', label='MAE$(t)$')
ax.plot(std_history_times, std_history, color='magenta', label='STD$(t)$')
ax.plot(obj_history_times, obj_history, 
    color='blue', linewidth=2, linestyle='-', label='objective$(t)$')
mobj = min(obj_history)
ax.axhline(mobj, label='min obj $=%.5f$' % mobj, 
    linewidth=3, linestyle='--', color='green')

ax2 = ax.twinx()
ax2.plot(
    diff_parameter_history_times,
    np.sum(diff_parameter_history, axis=1), color='red'
)
ax2.set_ylabel('$\sum_i \Delta p_i(t)$', color='red')
ax2.set_yscale('log')

label = 'Twiddle loop restarts'
from math import ceil
for t in cycle_start_times:
    ax.axvline(t, color='gold', lw=2, label=label)
    label = None

ax.grid(False)
ax2.grid(False)

ax.set_xlabel('$t$ [min]')
ax.set_ylabel('objective components')
ax.legend(fontsize=12, loc='lower left')

fig.tight_layout()

fig.savefig('error_hist_2.png')


# fig, (ax1, ax2) = plt.subplots(ncols=2)
# ax1.scatter(parameter_history[:, 0], parameter_history[:, 1], c=range(len(parameter_history)))
# ax1.set_xlabel('$p_0(t)$')
# ax1.set_ylabel('$p_1(t)$')

# ax2.scatter(parameter_history[:, 0], parameter_history[:, 2], c=range(len(parameter_history)))
# ax2.set_xlabel('$p_0(t)$')
# ax2.set_ylabel('$p_2(t)$')

# fig.tight_layout()

# fig.savefig('space_trajectory.png')

plt.show()