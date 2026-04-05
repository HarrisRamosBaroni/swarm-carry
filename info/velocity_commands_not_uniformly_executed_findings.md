I ran some tests:
CMD_VX = 10.0, CMD_VY = 0.0:
(mj-venv) harris@harris-IdeaPad-Pro-5-14AKP10:~/Documents/y3/dot/swarm-carry$ python experiments/mrcap_fg/diagnose_formation.py --vis

Trial 1: surround formation (robots at 0°, 90°, 180°, 270°)
  [Surround formation] Viewer open — press Enter to start...
  saved /home/harris/Documents/y3/dot/swarm-carry/experiments/mrcap_fg/figures/diagnose_surround.png

Trial 2: aligned formation (all robots yaw=0°)
  [Aligned formation] Viewer open — press Enter to start...
  saved /home/harris/Documents/y3/dot/swarm-carry/experiments/mrcap_fg/figures/diagnose_aligned.png

Spread in final Δx (should be ~0 if conversion is correct):
  surround: Δx = [12.54   2.43  12.59   2.443]  (range 10.1598 m)
  aligned: Δx = [12.518 12.518 12.495 12.549]  (range 0.0539 m)

Spread in final Δy (should be ~0):
  surround: Δy = [ 0.014 -0.004 -0.022  0.012]
  aligned:  Δy = [-0.193  0.014  0.107  0.228]

CMD_VX = 5.0, CMD_VY = 0.0:
(mj-venv) harris@harris-IdeaPad-Pro-5-14AKP10:~/Documents/y3/dot/swarm-carry$ python experiments/mrcap_fg/diagnose_formation.py --vis
...
Spread in final Δx (should be ~0 if conversion is correct):
  surround: Δx = [12.025  2.43  12.057  2.443]  (range 9.6266 m)
  aligned: Δx = [12.043 12.027 12.    12.028]  (range 0.0430 m)

Spread in final Δy (should be ~0):
  surround: Δy = [ 0.012 -0.004  0.03   0.012]
  aligned:  Δy = [-0.141 -0.056  0.109  0.204]

CMD_VX = 1.0, CMD_VY = 0.0:
(mj-venv) harris@harris-IdeaPad-Pro-5-14AKP10:~/Documents/y3/dot/swarm-carry$ python experiments/mrcap_fg/diagnose_formation.py --vis
...
Spread in final Δx (should be ~0 if conversion is correct):
  surround: Δx = [3.807 2.001 3.808 1.986]  (range 1.8225 m)
  aligned: Δx = [3.808 3.809 3.807 3.809]  (range 0.0023 m)

Spread in final Δy (should be ~0):
  surround: Δy = [ 0.001 -0.001 -0.003 -0.011]
  aligned:  Δy = [-0.002 -0.003  0.    -0.001]


CMD_VX = 0.1, CMD_VY = 0.0:
(mj-venv) harris@harris-IdeaPad-Pro-5-14AKP10:~/Documents/y3/dot/swarm-carry$ python experiments/mrcap_fg/diagnose_formation.py --vis
...
Spread in final Δx (should be ~0 if conversion is correct):
  surround: Δx = [0.409 0.226 0.409 0.226]  (range 0.1832 m)
  aligned: Δx = [0.409 0.409 0.409 0.409]  (range 0.0000 m)

Spread in final Δy (should be ~0):
  surround: Δy = [-0. -0.  0. -0.]
  aligned:  Δy = [-0.  0.  0. -0.]

CMD_VX = 0.01, CMD_VY = 0.0:
(mj-venv) harris@harris-IdeaPad-Pro-5-14AKP10:~/Documents/y3/dot/swarm-carry$ python experiments/mrcap_fg/diagnose_formation.py
...
Spread in final Δx (should be ~0 if conversion is correct):
  surround: Δx = [0.032 0.018 0.032 0.018]  (range 0.0143 m)
  aligned: Δx = [0.032 0.032 0.032 0.032]  (range 0.0000 m)

Spread in final Δy (should be ~0):
  surround: Δy = [ 0. -0.  0.  0.]
  aligned:  Δy = [ 0. -0. -0. -0.]

to me this says that first, the speed limit is at least hit with command speed 5ms, since final distance travelled by front facing robots is 12m in CMD_VX = 5.0 and CMD_VX = 10.0 . I don't know if speed limit implies torque limit hit. Second, even without bieng anywhere near speed limit, CMD_VX = 0.01 only travelling 0.032 m for front facing robots, the strafing robots still travelled a little over half that velocity. The ratios 2.001/3.807, 0.018/0.032, 0.226/0.409 are not quite the same:
(mj-venv) harris@harris-IdeaPad-Pro-5-14AKP10:~/Documents/y3/dot/swarm-carry$ python
>>> 2.001/3.807
0.5256107171000788
>>> 0.018/0.032
0.5625
>>> 0.226/0.409
0.5525672371638143
so I suppose it is not a simple deterministic ratio based on, say, the robot-world interactions and geometries quirks. Or maybe it is and perhaps it is easily or not easily calculable.

I ran these tests because I was noticing that the velocity limits wasn't keeping formation for n=3 robots (features only one front-facing robot) (or any formations without homogeneous robot orientations). This happened regardless of payload interactions (though payload interactions did exacerbate drift from formation), and the following two experiments remove this variable by enforcing payload_size=(0.01, 0.01, 0.01) as in the diagnosis script:

(mj-venv) harris@harris-IdeaPad-Pro-5-14AKP10:~/Documents/y3/dot/swarm-carry$ python experiments/mrcap_fg/run_experiment.py --n-values 3 --vis --sim-speed 1.0 --v-max 0.2 --payload-mass 0.01

============================================================
MR.CAP Factor-Graph Scaling Experiment
  robots:       [3]
  distance:     5.0 m
  horizon:      15
  v_max:        0.2 m/s
  payload_mass: 0.01 kg
============================================================

Running n=3 ...
  Viewer open — adjust camera, then press Enter to start...
  [TIMEOUT]  payload=0.0 kg  final_error=5.000 m  deviation=0.000 m  solve=10.8 ± 1.4 ms  sat=45.8%  peak=316.9 Nm  steps=1199

===========================================================================
   n    status  final_err(m)  deviation(m)  solve_mean(ms)    sat%   peak(Nm)
----  --------  ------------  ------------  --------------  ------  ---------
   3   TIMEOUT         5.000         0.000           10.75    45.8     316.89
============================================================

Results saved to /home/harris/Documents/y3/dot/swarm-carry/experiments/mrcap_fg/results.json
(mj-venv) harris@harris-IdeaPad-Pro-5-14AKP10:~/Documents/y3/dot/swarm-carry$ python experiments/mrcap_fg/run_experiment.py --n-values 3 --vis --sim-speed 1.0 --v-max 0.02 --payload-mass 0.01

============================================================
MR.CAP Factor-Graph Scaling Experiment
  robots:       [3]
  distance:     5.0 m
  horizon:      15
  v_max:        0.02 m/s
  payload_mass: 0.01 kg
============================================================

Running n=3 ...
  Viewer open — adjust camera, then press Enter to start...
  [TIMEOUT]  payload=0.0 kg  final_error=5.000 m  deviation=0.000 m  solve=10.2 ± 1.2 ms  sat=48.5%  peak=54.8 Nm  steps=1199

===========================================================================
   n    status  final_err(m)  deviation(m)  solve_mean(ms)    sat%   peak(Nm)
----  --------  ------------  ------------  --------------  ------  ---------
   3   TIMEOUT         5.000         0.000           10.22    48.5      54.82
============================================================

Results saved to /home/harris/Documents/y3/dot/swarm-carry/experiments/mrcap_fg/results.json


I don't understand how to interpret the sat% unless it is the case that the torque limits are being hit at any commanded speed as long as there is some strafing?

Anyway, what can we gather or hypothesise?