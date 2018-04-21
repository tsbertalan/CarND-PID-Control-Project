import matplotlib.pyplot as plt
import numpy as np
from sys import argv

plot_fname_segments = []
def add_fname_segment(name, value, format='%s'):
    plot_fname_segments.append((name + format) % value)
def get_fname_segments():
    return '_'.join(plot_fname_segments)

nparam = 3
add_fname_segment('nparam', nparam)


if len(argv) == 1:
    fname = 'twiddle.out'
else:
    fname = argv[1]

# Read twiddle log.
with open(fname, 'r') as f:
    lines = f.readlines()

# Read CTE log.
cte_history = []
cte_history_times = []
steer_history = []
throttle_history = []
t0_cte = None
# Discard the first fraction of a minute of telemetry.
cte_discard = 0.08 * 1000 * 60
with open('../build/cte.csv', 'r') as ctefile:
    for line in ctefile.readlines():
        t, cte, speed, angle, steer, throttle = line.split(',')
        t = int(t)
        if t0_cte is None:
            t0_cte = t
        if t - t0_cte > cte_discard:
            cte_history.append(float(cte))
            cte_history_times.append(t)
            steer_history.append(float(steer))
            throttle_history.append(float(throttle))

parameter_history = []
parameter_history_times = []

accepted_parameter_history = []
accepted_parameter_history_times = []

diff_parameter_history = []
diff_parameter_history_times = []

std_history = []
std_history_times = []

mae_history = []
mae_history_times = []

obj_history = []
obj_history_times = []

cycle_start_times = []
param_modification_times = []
success_times = []


def parse_time_from_line(l):
    if '|' in l:
        l = l.split('|')
        assert len(l) == 2
        return long(l[0]), l[1].strip()
    else:
        return None, l


for l in lines:
    t_clock, l = parse_time_from_line(l)

    if t_clock is None:
        continue

    if 'succeed' in l:
        accepted_parameter_history.append(parameter_history[-1])
        accepted_parameter_history_times.append(parameter_history_times[-1])
        success_times.append(t_clock)

    elif 'dp =' in l:
        exec(l)
        if len(dp) == nparam:
            diff_parameter_history.append(dp)
            diff_parameter_history_times.append(t_clock)

    elif 'p =' in l:
        param_modification_times.append(t_clock)
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

ar = lambda v: np.array(v).astype(float)

parameter_history = ar(parameter_history)
accepted_parameter_history = ar(accepted_parameter_history)
diff_parameter_history = ar(diff_parameter_history)
# obj_history = ar(obj_history)
# mae_history = ar(mae_history)
# std_history = ar(std_history)
# cte_history = ar(cte_history)
# steer_history = ar(steer_history)

parameter_history_times = ar(parameter_history_times)
accepted_parameter_history_times = ar(accepted_parameter_history)
diff_parameter_history_times = ar(diff_parameter_history_times)
obj_history_times = ar(obj_history_times)
mae_history_times = ar(mae_history_times)
std_history_times = ar(std_history_times)
cte_history_times = ar(cte_history_times)

cycle_start_times = ar(cycle_start_times)
param_modification_times = ar(param_modification_times)
success_times = ar(success_times)


# Find the modal number-of-samples per parameter change.
try:
    param_modification_steps = []
    i = 0
    cte_times_shrinking = np.copy(cte_history_times)
    for t in param_modification_times:
        if len(cte_times_shrinking) == 0:
            break
        i_local = np.argmin(np.abs(t - cte_times_shrinking))
        i += i_local
        param_modification_steps.append(int(i))
        cte_times_shrinking = cte_times_shrinking[i_local:]
    diffs = np.diff(param_modification_steps)
    diffs = diffs[diffs!=0]
    (_, idx, counts) = np.unique(diffs, return_index=True, return_counts=True)
    index = idx[np.argmax(counts)]
    nsamps = diffs[index]
except ValueError:
    nsamps = None
add_fname_segment('nsamps', nsamps)

Ts = (
    parameter_history_times, diff_parameter_history_times, 
    obj_history_times, mae_history_times, std_history_times,
    cte_history_times,
    cycle_start_times, param_modification_times, success_times,
)
t0 = min([a.min() for a in Ts if len(a) > 0])
for a in Ts:
    a[:] -= t0
    # convert ms to min
    a[:] /= 1000 * 60

fig, ax = plt.subplots(figsize=(24,12))
ax2 = ax.twinx()
ax.axhline(0, color='black', alpha=.25, linestyle=':')

ax2.plot(mae_history_times, mae_history, 
    color='blue', label='MAE$(t)$', linewidth=1, linestyle='--')
ax2.plot(std_history_times, std_history, 
    color='blue', label='STD$(t)$', linewidth=1, linestyle='-.')
ax2.plot(obj_history_times, obj_history, 
    color='blue', linewidth=1, linestyle='-', label='objective$(t)$')

ax.plot(cte_history_times, cte_history,
        color='black', alpha=.25, linestyle='-', label='raw cte$(t)$')
ax.plot(cte_history_times, steer_history,
        color='red', alpha=.25, linestyle='-', label='steer$(t)$')
ax.plot(cte_history_times, throttle_history,
        color='green', alpha=.25, linestyle='-', label='throttle$(t)$')

ax.grid(False)
ax2.grid(False)

if len(obj_history) > 0:
    mobj = min(obj_history)
    ax2.axhline(mobj, label='best obj so far $=%.5f$' % mobj,
        linewidth=1, linestyle=':', color='blue')

if len(diff_parameter_history) > 0:
    ax2.plot(
        diff_parameter_history_times,
        np.sum(diff_parameter_history, axis=1), color='purple',
        label=r'update norm $\left(\sum_i \Delta p_i\right)(t)$',
    )
    ax2.set_ylabel('logarithmic values')
    ax2.set_yscale('log')
    

label = 'Twiddle loop restarts'
for t in cycle_start_times:
    ax2.axvline(t, color='gold', lw=4, label=label, alpha=.5)
    label = None

label = 'Parameter modifications'
for t in param_modification_times:
    ax2.axvline(t, color='pink', lw=1, label=label, alpha=.5)
    label = None

label = 'Prev. param mod. successfully decreased obj.'
for t in success_times:
    ax2.axvline(t, color='salmon', lw=4, label=label, alpha=.5, linestyle='--')
    label = None

ax.set_xlabel('$t$ [min]')
ax.set_ylabel('linear values')
ax.legend(fontsize=12, loc='lower left')
ax2.legend(fontsize=12, loc='lower right')

fig.tight_layout()

for fname in ['error_hist-%s.png' % get_fname_segments(), 'error_hist.png']:
    fig.savefig(fname)


fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(16,9))
try:
    ax1.scatter(parameter_history[:, 0], parameter_history[:, 1], c=range(len(parameter_history)))
    ax1.plot(accepted_parameter_history[:, 0], accepted_parameter_history[:, 1],
        linewidth=2, color='red')
    ax1.set_xlabel('$p_0(t)$')
    ax1.set_ylabel('$p_1(t)$')

    sc = ax2.scatter(parameter_history[:, 0], parameter_history[:, 2], c=range(len(parameter_history)))
    ax2.plot(accepted_parameter_history[:, 0], accepted_parameter_history[:, 2],
        linewidth=2, color='red')
    fig.colorbar(sc, ax=ax2, label='iteration')
    ax2.set_xlabel('$p_0(t)$')
    ax2.set_ylabel('$p_2(t)$')

    fig.tight_layout()

    for fname in ['space_trajectory-%s.png' % get_fname_segments(), 'space_trajectory.png']:
        fig.savefig(fname)
except IndexError:
    plt.close(fig)

plt.show()