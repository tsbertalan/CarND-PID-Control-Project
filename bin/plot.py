import matplotlib.pyplot as plt
import numpy as np
from sys import argv

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

t = 0
i_param = 0
t_call = lambda : t * nparam + i_param
for l in lines:
    if l.startswith('p = '):
        exec(l)
        if len(p) == nparam:
            parameter_history.append(p)
            parameter_history_times.append(t_call())
    elif l.startswith('dp = '):
        exec(l)
        if len(dp) == nparam:
            diff_parameter_history.append(dp)
            diff_parameter_history_times.append(t_call())
    elif 'Mean absolute error' in l:
        mae_history.append(
            float(l.split('=')[1])
        )
        mae_history_times.append(t_call())
    elif 'Stdd error' in l:
        std_history.append(
            float(l.split('=')[1])
        )
        std_history_times.append(t_call())

    elif 'Twiddle iteration' in l:
        t = int(l.replace('Twiddle iteration', '').replace('=', '').replace(':', ''))
    elif 'i=' in l:
        i_param = int(l.replace('-', '').replace('i=', ''))

diff_parameter_history = np.array(diff_parameter_history)
parameter_history = np.array(parameter_history)

fig, ax = plt.subplots()
ax.plot(mae_history_times, mae_history, color='black', label='MAE$(t)$')
ax.plot(std_history_times, std_history, color='blue', label='STD$(t)$')

ax2 = ax.twinx()
ax2.plot(
    diff_parameter_history_times,
    np.sum(diff_parameter_history, axis=1), color='red'
)
ax2.set_ylabel('$\sum_i \Delta p_i(t)$', color='red')
ax2.set_yscale('log')

ax.grid(False)
ax2.grid(False)

ax.set_xlabel('Twiddle call count $t_c=t\cdot%d+i_{param}$' % nparam)
ax.set_ylabel('objective components')
ax.legend()

fig.tight_layout()


fig, (ax1, ax2) = plt.subplots(ncols=2)
ax1.scatter(parameter_history[:, 0], parameter_history[:, 1], c=range(len(parameter_history)))
ax1.set_xlabel('$p_0(t)$')
ax1.set_ylabel('$p_1(t)$')

ax2.scatter(parameter_history[:, 0], parameter_history[:, 2], c=range(len(parameter_history)))
ax2.set_xlabel('$p_0(t)$')
ax2.set_ylabel('$p_2(t)$')

fig.tight_layout()

plt.show()