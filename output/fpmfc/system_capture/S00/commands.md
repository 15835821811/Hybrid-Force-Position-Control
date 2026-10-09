# S00 commands

Run in the S00 branch with the existing Python 3.10 / MuJoCo 3.3.2 environment.

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
$env:PYTHONIOENCODING='utf-8'
python -m v6_mujoco.system_capture --phase S00 --mode prepare
python -m v6_mujoco.system_capture --phase S00 --mode test
python -m v6_mujoco.system_capture --phase S00 --mode replay
python -m v6_mujoco.system_capture --phase S00 --mode report
```

`run` invokes only the registered S00 replay plan and report, after prepare/tests. `--resume` requires exact source/config/environment/input identities and reuses only completed passing replays; incomplete or failed invocations are not restarted. Inspect the saved PID/process and logs before any intervention. No `--all` or implicit stage exists. Other phases return NOT_IMPLEMENTED with exit code 2 and create no outputs. Reserved paper-model factories likewise refuse execution.

The four-replay limit counts two t0 actuator/event streams and two packet decision streams. A direct archived-controller call is composed inside each adapter stream; its raw output and the independent original recording are both checked, without extra baseline passes. The temporary runtime view contains exact archived source bytes and only verified required assets plus the C1 pairing manifest. It is not a new maintained algorithm copy. All outputs go to S00; legacy runner entry points are never called.

After a successful local commit, record its SHA outside tracked content:

```powershell
python -c "from v6_mujoco.system_capture.report import record_local_commit; record_local_commit()"
```
