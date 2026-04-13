"""
ForceCentralisedController: centralized factor-graph controller for multi-robot payload transport.
Note: This is a centralised version of the factor graph, not the decentralised 
version we're developping.
Other note: this is very similar to the centralised DRCAP


Implements the core idea from:
  Jaafar et al., "Distributed Velocity-based Global Formation Control and Planning for Object Transport",
  not yet published at time of writing.

Factor graph structure (receding horizon of N steps):
  Variables:
    C_j in R^3  -- centroid pose [x, y, theta]   for j = k .. k+N
    U_j in R^3  -- centroid control [vx, vy, omega]  for j = k .. k+N-1 

    R_{i_j} in R^3 -- i-th robot pose [x, y, theta]   for j = k .. k+N
    U_{i_j} in R^3 -- i-th robot control [vx, vy, omega]   for j = k .. k+N
    m in R         -- mass of centroid (time invariant)
    
  Factors:
    current-state anchor  -- tight prior on C_k (current centroid pose)
    current-state anchor  -- tight prior on R_{i_k} (current robot pose)
    reference priors      -- PriorFactor on C_j toward linear-interp reference
    robot reference priors -- PriorFactor on R_{i_k} toward linear-interp reference (taking offset between robot and centroid into account)
    control regulariser centroid  -- PriorFactor on U_j toward zero
    control regulariser robot  -- PriorFactor on U_j toward zero
    motion model for robots -- R_{i_{j+1}} = R_{i_j} + dt * U_{i_j}   (linear; exact Jacobians) 
    motion model for centroid --  C_{j+1} = C_j + dt * U_j + 0.5 * F / m   , with F cumulative forces and m mass

    terminal anchor       -- tight prior on C_{k+N} at goal

    robot to robot (R2R)  -- prior ? keep constant distances between each robot (keep current formation)
    centroid pull-in      -- prior ? pull centroid position towards each robot (so effectively pull it towards formation center)


Note: The obstacle avoidance factor from the original DRCap was removed for simplification

Per-robot velocity (holonomic rigid body): 
  defined by their respective control inputs

#TODO add a high-cost factor in case forces are moer than 4-5kg to avoid breaking loadcells
#TODO add a way to estimate forces while doing the MPC thing #currently just assuming cst
#TODO implement mass as a 'global' node (currently more 'local')
#TODO remember entire factor graph as opposed to creating a new one every time step
#TODO make it decentralised
#TODO Remove real centroid position being fed in and use factor graph node instead (note: get it to work with real measurement first)
#TODO ROS


"""

from __future__ import annotations

import time
from functools import partial
from typing import Dict, Any, List, Optional

import numpy as np

try:
    import gtsam
except ImportError:
    raise ImportError("gtsam required: pip install gtsam")

from .base_controller import BaseController


# ---------------------------------------------------------------------------
# Reusable factor error functions
# ---------------------------------------------------------------------------

def _prior_error(
    measurement: np.ndarray,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
) -> np.ndarray:
    """Prior factor: error = variable - measurement."""
    v = values.atVector(this.keys()[0])
    error = v - measurement
    if jacobians is not None:
        jacobians[0] = np.eye(len(measurement))

    return error

def _r2r_distance_error(
    measurement: np.ndarray,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
)-> np.ndarray:
    """R2R dstance factor: error = abs(initial_distance - current_distance)"""
    robot1 = values.atVector(this.keys()[0])
    robot2 = values.atVector(this.keys()[1])

    current_distance = float(np.linalg.norm(robot1[0:2] - robot2[0:2]))
    error = np.array([current_distance - measurement[0]]) 


    if jacobians is not None:
        diff = robot1[0:2] - robot2[0:2]
        dist = np.linalg.norm(diff)

        if dist < 1e-9:
            J = np.zeros((1, 3))
        else:
            grad = diff / dist  # shape (2,)
            J = np.zeros((1, 3))
            J[0, 0:2] = grad

        jacobians[0] = J
        jacobians[1] = -J


    return error



def _pull_in_error(
    measurement: np.ndarray,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
) -> np.ndarray:
    """Pull-in factor: error = robot - centroid."""

    #from passed keys access values (no measurements)
    robot  = values.atVector(this.keys()[0])
    centroid  = values.atVector(this.keys()[1])

    error = robot[0:2] - centroid[0:2] #don't care about angle, only care about x,y

    if jacobians is not None:
        J_robot = np.zeros((2, 3))
        J_centroid = np.zeros((2, 3))

        # derivative wrt robot
        J_robot[0:2, 0:2] = np.eye(2)

        # derivative wrt centroid
        J_centroid[0:2, 0:2] = -np.eye(2)

        jacobians[0] = J_robot
        jacobians[1] = J_centroid

    return error


def _motion_model_error( 
    dt: float,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
) -> np.ndarray:
    """
    Euler motion model: C_{j+1} = C_j + dt * U_j
    Keys: [C_j, U_j, C_{j+1}]
    error = C_{j+1} - (C_j + dt * U_j)
    """
    Cj  = values.atVector(this.keys()[0])
    Uj  = values.atVector(this.keys()[1])
    Cj1 = values.atVector(this.keys()[2])

    error = Cj1 - (Cj + dt * Uj)

    if jacobians is not None:
        I3 = np.eye(3)
        jacobians[0] = -I3
        jacobians[1] = -dt * I3
        jacobians[2] =  I3

    return error

def _motion_force_model_error(
    dt: float,
    forces: np.ndarray,
    this: gtsam.CustomFactor,
    values: gtsam.Values,
    jacobians: Optional[List[np.ndarray]],
) -> np.ndarray:
    """
    Euler motion model: C_{j+1} = C_j + dt * U_j + 0.5 * dt**2 / M * [F_x, F_y, 0] #we set torque to 0 (for now at least)
    Keys: [C_j, U_j, C_{j+1}, M]
    error = C_{j+1} - (C_j + dt * U_j + 0.5 * dt**2 / M * [F_x, F_y, 0])
    """
    Cj  = values.atVector(this.keys()[0])
    Uj  = values.atVector(this.keys()[1])
    Cj1 = values.atVector(this.keys()[2])
    M = values.atVector(this.keys()[3])

    F = np.array([forces[0], forces[1], 0.0]) #F_x, F_y, F_theta = 0 bc ignoring torque right now

    error = Cj1 - (Cj + dt * Uj + (0.5 * (dt**2) / M) * F)

    if jacobians is not None:
        I3 = np.eye(3)
        jacobians[0] = -I3
        jacobians[1] = -dt * I3
        jacobians[2] =  I3
        # jacobians[3] = np.column_stack([((dt**2) * F[0]) / (2*(M**2)), ((dt**2) * F[1]) / (2*(M**2)), 0]) #last one set to 0 bc, once again, ignoring torque
        
        jacobians[3] = np.array([[((dt**2) * F[0]) / (2*(M[0]**2))], [((dt**2) * F[1]) / (2*(M[0]**2))], [0]]) #last one set to 0 bc, once again, ignoring torque
        
    return error


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class ForceCentralisedController(BaseController):
    """
    Centralized factor-graph controller (MR.CAP, holonomic adaptation).

    Parameters
    ----------
    num_robots : int
    formation  : list of (x_off, y_off, yaw) per robot relative to payload centre,
                 same format as MecanumTransportEnv's `formation` parameter.
                 If None, robots are placed on a ring of radius r_formation.
    config     : optional dict:
        horizon      int    receding horizon N (default 15)
        sigma_x      float  std-dev for reference trajectory priors (default 0.5)
        sigma_u      float  std-dev for control regularisation (default 0.3)
        sigma_anchor float  std-dev for current-state / terminal anchors (default 0.01)
        sigma_mm     float  std-dev for motion model factor (default 1e-4)
        v_max        float  per-robot speed clamp m/s (default 1.0)
        omega_max    float  centroid angular velocity clamp rad/s (default 1.5)
        r_formation  float  ring radius used when formation=None (default 0.6)

        sigma_r2r    float  std-dev for robot to robot factor (default 0.05) #note: probably need to tune these quite a bit
        sigma_pull_in float  std-dev for pull-in factor (default 0.3) (high bc incorrect by definition, we know the centroid isn't on the robot)
    """

    def __init__(
        self,
        num_robots: int,
        formation: Optional[List[tuple]] = None,
        config: Optional[Dict[str, Any]] = None,
    ):
        super().__init__(num_robots, config)

        cfg = self.config
        self._N          = int(cfg.get("horizon",       15))
        self._sigma_x    = float(cfg.get("sigma_x",     0.5)) #0.5
        self._sigma_u    = float(cfg.get("sigma_u",     0.3)) #0.3
        self._sigma_anc  = float(cfg.get("sigma_anchor", 0.01))
        self._sigma_mm   = float(cfg.get("sigma_mm",    1e-4))
        self._v_max      = float(cfg.get("v_max",       1.0))
        self._omega_max  = float(cfg.get("omega_max",   1.5))

        self._sigma_r2r  = float(cfg.get("sigma_r2r",   0.05))
        self._sigma_pull_in  = float(cfg.get("sigma_pull_in",   0.3))
        self._sigma_anc_robots = float(cfg.get("sigma_anc_robots",   2)) #TESTING


        # Formation offsets r_i = [dx, dy] from centroid to robot i (world frame).
        # Rigid body: these stay constant throughout the run.
        if formation is not None:
            self._r = np.array([[f[0], f[1]] for f in formation], dtype=float)
        else:
            radius = float(cfg.get("r_formation", 0.6))
            angles = np.linspace(0, 2 * np.pi, num_robots, endpoint=False)
            self._r = np.column_stack([radius * np.cos(angles),
                                       radius * np.sin(angles)])

        # Noise models built once and reused
        self._noise_x   = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_x))
        self._noise_u   = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_u))
        self._noise_anc = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_anc))
        self._noise_mm  = gtsam.noiseModel.Diagonal.Sigmas(np.full(3, self._sigma_mm))

        self._noise_r2r = gtsam.noiseModel.Diagonal.Sigmas(np.full(1, self._sigma_r2r)) #only 1D num bc distance
        self._noise_pull_in = gtsam.noiseModel.Diagonal.Sigmas(np.array([self._sigma_pull_in, self._sigma_pull_in])) #back to 2D #made 3D for simplification, but we don't care about angle (since very high uncertainty. only care about x and y)
        self._noise_anc_robots = gtsam.noiseModel.Diagonal.Sigmas(np.array([self._sigma_anc_robots, self._sigma_anc_robots, 9999])) 


        self._robots_init_pos = self._r.copy() #intial position of robots 

    # ------------------------------------------------------------------
    # BaseController interface
    # ------------------------------------------------------------------

    def reset(self):
        self._last_solve_time = 0.0
        self._total_solves = 0

    def compute_control(
        self,
        payload_state: np.ndarray,
        robot_states: np.ndarray, # [x, y, theta, vx, vy] for each robot
        goal_state: np.ndarray,
        dt: float,
        forces: np.ndarray = None,
        mass_estimate: float = None,
    ) -> np.ndarray:
        centroid = payload_state[:3].copy()   # [x, y, theta]
        robots = robot_states[:,:3].copy() # [x, y, theta] for each robot
        print('payload_state:', payload_state)
        print('robot_states:', robot_states) 

        goal     = np.asarray(goal_state, dtype=float)[:3]

        t0 = time.perf_counter()
        # U_c, mass_estimate = self._solve_fg(centroid, goal, dt, forces, mass_estimate)
        #TODO will only use robot pos from MoCap eventually
        U_c_all, mass_estimate = self._solve_fg(centroid, robots, goal, dt, forces, mass_estimate)

        self._set_solve_time(time.perf_counter() - t0)

        print("Forces:", forces)
        print("mass estimate:",mass_estimate)

        return self._robot_velocities2(U_c_all)*10, mass_estimate #TODO robot velocities x10 bc they are way too slow for some reason...

    # ------------------------------------------------------------------
    # Factor graph solve
    # ------------------------------------------------------------------

    def _solve_fg(
        self, centroid: np.ndarray, robots: np.ndarray, goal: np.ndarray, dt: float, 
        forces: np.ndarray, mass_estimate: float
    ) -> np.ndarray:
        """Solve FG and return optimal centroid control [vx, vy, omega]."""
        N = self._N

        # Linear reference trajectory from current centroid to goal #5th factor hidden here
        ref = np.array([centroid + (j / N) * (goal - centroid) for j in range(N + 1)])

        graph = gtsam.NonlinearFactorGraph()
        init  = gtsam.Values()

        def Ck(j): return gtsam.symbol('X', j) #using X as character or else overwritten by robot control node
        def Uk(j): return gtsam.symbol('U', j)
        def M(): return gtsam.symbol('Z', 0) #mass is time-invariant

        #make as many robot nodes as needed
        robot_nodes_fg = []
        for i in range(self.num_robots):
            robot_nodes_fg.append(lambda j: gtsam.symbol(chr(97 + i), j)) #lowercase letters representing robots

        robot_control_nodes_fg = []
        for i in range(self.num_robots):
            robot_control_nodes_fg.append(lambda j: gtsam.symbol(chr(65 + i), j)) #uppercase letters representing robot controls (1 per robot)

        for j in range(N):
            kC  = Ck(j)
            kU  = Uk(j)
            kC1 = Ck(j + 1)
            Ma = M()

            #multiple robots
            current_robot_nodes_fg = []
            current_robot_control_nodes_fg = []
            for i in range(self.num_robots):
                current_robot_nodes_fg.append(robot_nodes_fg[i](j)) 
                current_robot_control_nodes_fg.append(robot_control_nodes_fg[i](j)) 

            #time step k+1
            next_ts_robot_nodes_fg = []
            for i in range(self.num_robots):
                next_ts_robot_nodes_fg.append(robot_nodes_fg[i](j+1)) 

            # State prior
            if j == 0:
                noise_c = self._noise_anc

                #initialising M here bc time-invariant so only needs 1 init. #TODO check if it initialised once at beginning only or once every fg solve (=> yeah it defo resets after each MPC window). Maybe collect value from previous guess?
                if mass_estimate is None:
                    init.insert(Ma, np.array([10])) #initialised as little weight (can't do 0 bc div by 0.)
                else:
                    init.insert(Ma, np.array([mass_estimate]))

                #hard pin robot pos to MoCap-measured pos
                for i in range(self.num_robots):                  
                    graph.add(gtsam.CustomFactor(
                        noise_c, [current_robot_nodes_fg[i]], partial(_prior_error,  np.array([float(robots[i,0]), float(robots[i,1]), float(robots[i,2])])))) 



            else:
                noise_c = self._noise_x

            #current state anchor factor
            graph.add(gtsam.CustomFactor(
                noise_c, [kC], partial(_prior_error, ref[j].copy()))) 
            
 
            init.insert(kC, ref[j].copy())

            # control regulariser factor (bring control to 0)
            # Control regularisation toward zero; warm-start with ref velocity
            u_warm = (ref[j + 1] - ref[j]) / dt if dt > 1e-9 else np.zeros(3)
            # graph.add(gtsam.CustomFactor(
            #     self._noise_u, [kU], partial(_prior_error, np.zeros(3))))
     
            init.insert(kU, u_warm)

            #same for robot control nodes
            for i in range(self.num_robots):
                graph.add(gtsam.CustomFactor(
                self._noise_u, [current_robot_control_nodes_fg[i]], partial(_prior_error, np.zeros(3))))

                init.insert(current_robot_control_nodes_fg[i], u_warm)
                
            #get cumulative forces:
            cum_forces = np.sum(forces, axis=0)

            #motion model factor
            # Motion model: C_{j+1} = C_j + dt * U_j
            graph.add(gtsam.CustomFactor(
                self._noise_mm, [kC, kU, kC1, Ma],
                partial(_motion_force_model_error, dt, cum_forces)))
            
            #motion model for robots factor (for all robots)
            for i in range(self.num_robots):
                graph.add(gtsam.CustomFactor(
                    self._noise_mm, [current_robot_nodes_fg[i], current_robot_control_nodes_fg[i], next_ts_robot_nodes_fg[i]], 
                    partial(_motion_model_error, dt)))
            

            #robot to robot factor
            for i in range(self.num_robots):
                graph.add(gtsam.CustomFactor(
                self._noise_r2r, [current_robot_nodes_fg[i], current_robot_nodes_fg[(i+1)%self.num_robots]], partial(_r2r_distance_error, 
                    np.array([
                        np.linalg.norm(self._robots_init_pos[i] - self._robots_init_pos[(i+1)%self.num_robots])
                        ])
                        ))) 
            
            
            #initialise all robot nodes
            for i in range(self.num_robots):
                # init.insert(current_robot_nodes_fg[i], np.array([float(self._robots_init_pos[i,0]), float(self._robots_init_pos[i,1]), 0.0])) #angle assumption here is wrong but we do nothing with angles so doesn't matter really
                print('robots:',robots)
                traj_offset = ref[j].copy() - ref[0].copy() #follow (roughly) trajectory of centroid
                init.insert(current_robot_nodes_fg[i], np.array([float(robots[i,0]) + traj_offset[0], float(robots[i,1]) + traj_offset[1], float(robots[i,2]) + traj_offset[2]])) #use positions measured from Sim (simulating MoCap) for robot positions at time t


            #pull in factor (all robot nodes)
            for i in range(self.num_robots):
                graph.add(gtsam.CustomFactor(self._noise_pull_in, [current_robot_nodes_fg[i], kC], partial(_pull_in_error, 0.0)))



        # Terminal anchor on C_N
        kCN = Ck(N)
        graph.add(gtsam.CustomFactor(
            self._noise_anc, [kCN], partial(_prior_error, goal.copy())))
        init.insert(kCN, ref[N].copy())

        #terminal anchor on last state of each robot
        #I don't really understand why, but apparently this is necessary for the code to run, so I just put a huge noise value so the factor graph doesn't pay much attention to it
        for i in range(self.num_robots):
            #kRN = next_ts_robot_nodes_fg[-1](N)
            graph.add(gtsam.CustomFactor(
                self._noise_anc_robots, [robot_nodes_fg[i](N)], partial(_prior_error, goal.copy())))
            init.insert(robot_nodes_fg[i](N), ref[N].copy())

        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosity("SILENT")
        result = gtsam.LevenbergMarquardtOptimizer(graph, init, params).optimize()

        # U_opt = result.atVector(Uk(0))
        all_U_opt = [result.atVector(robot_control_nodes_fg[i](0)) for i in range(self.num_robots)]
        M_opt = result.atVector(M())

        lo = np.array([-self._v_max, -self._v_max, -self._omega_max])
        hi = np.array([ self._v_max,  self._v_max,  self._omega_max])

        # U_opt = np.clip(U_opt, lo, hi)
        print('all_U_opt before clip',all_U_opt)
        all_U_opt = np.clip(all_U_opt, lo, hi)
        print('all_U_opt after clip',all_U_opt)
        M_opt = np.clip(M_opt, 0.1, 10000) #clip mass to avoid numerical instability (especially negative or =0)
        return all_U_opt, M_opt

    # ------------------------------------------------------------------
    # Rigid-body velocity distribution
    # ------------------------------------------------------------------

    def _robot_velocities(self, U_c: np.ndarray) -> np.ndarray: 
        """
        Derive per-robot [vx, vy] from centroid control U_c = [vx_c, vy_c, omega_c].

        Holonomic rigid-body kinematics:
          v_ix = vx_c - omega_c * r_iy
          v_iy = vy_c + omega_c * r_ix
        Then clamp individual robot speeds to v_max.
        """
        vx_c, vy_c, omega_c = U_c

        r = self._r                            # (n, 2)
        vx = vx_c - omega_c * r[:, 1]
        vy = vy_c + omega_c * r[:, 0]

        speeds = np.hypot(vx, vy)
        scale  = np.where(speeds > self._v_max, self._v_max / speeds, 1.0)
        return np.column_stack([vx * scale, vy * scale])
    
    def _robot_velocities2(self, U_c_all: np.ndarray) -> np.ndarray: 
        """
        Derive per-robot [vx, vy] from centroid control U_c = [vx_c, vy_c, omega_c].

        Holonomic rigid-body kinematics:
          v_ix = vx_c - omega_c * r_iy
          v_iy = vy_c + omega_c * r_ix
        Then clamp individual robot speeds to v_max.
        """
        print('U_c_all', U_c_all)
        
        vx = U_c_all[:,0]
        vy =  U_c_all[:,1]
        omega = U_c_all[:,2]
        # vx, vy, omega = U_c_all

        r = self._r                            # (n, 2)
        vx = vx - omega * r[:, 1]
        vy = vy + omega * r[:, 0]

        speeds = np.hypot(vx, vy)
        scale  = np.where(speeds > self._v_max, self._v_max / speeds, 1.0)
        print('return robot vel', np.column_stack([vx * scale, vy * scale]))
        return np.column_stack([vx * scale, vy * scale])
    

