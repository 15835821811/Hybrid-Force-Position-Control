"""Retain every ledger attempt; unavailable evaluations are never false outcomes."""
from paper_plot_style import *

attempts = read(ROOT / 'run_ledger.json')['attempts']
data, labels = [], []
for attempt in attempts:
    run = ROOT / 'runs' / attempt['name']
    metrics = read(run / 'metrics.json') if (run / 'metrics.json').exists() else {}
    complete = metrics.get('continuous_task_completed')
    safety = (bool(metrics['actual_safety_violations'])
              if 'actual_safety_violations' in metrics and (run / 'trace.npz').exists()
              else None)
    detumbling = (metrics.get('postgrasp_detumbling')
                  if metrics.get('full_window_evaluated') else None)
    data.append([.5 if value is None else int(value)
                 for value in (complete, safety, detumbling)])
    stop = metrics.get('end_time_s', attempt.get('end_time_s'))
    stop_text = 'NE' if stop is None else f'{stop:.3f} s'
    status = metrics.get('status', attempt.get('status', 'UNAVAILABLE'))
    labels.append(f"{attempt['name']} ({attempt['category']}, {attempt['version']})\n"
                  f"{status}; stop: {stop_text}")

fig, ax = plt.subplots(figsize=(8.2, max(2.4, .62 * len(attempts) + .65)),
                       constrained_layout=True)
ax.pcolormesh(data, cmap='Greys', vmin=0, vmax=1, shading='flat', rasterized=False)
ax.set_xlim(0, 3)
ax.set_ylim(len(labels), 0)
ax.set_xticks(np.arange(3) + .5, ['Task complete', 'Observed safety\nviolation', 'Full-window\ndetumbling'])
ax.set_yticks(np.arange(len(labels)) + .5, labels)
ax.tick_params(axis='y', labelsize=8)
for i, row in enumerate(data):
    for j, value in enumerate(row):
        ax.text(j + .5, i + .5, 'NE' if value == .5 else ('Yes' if value else 'No'),
                ha='center', va='center', color='white' if value == 1 else 'black')
savefig(fig, 'f4_all_attempt_outcomes')
