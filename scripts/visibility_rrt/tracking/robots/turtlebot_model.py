import numpy as np
from visibility_rrt.tracking.robot import BaseRobot

class TurtleBotModel(BaseRobot):
    def __init__(self, X0, dt, ax):
        # X = [x, y, θ]ᵀ  (we won’t model velocity explicitly)
        super().__init__(X0.reshape(-1,1), dt, ax, 'TurtleBot')
        # set your actual turtlebot limits:
        self.v_max = 0.22       # m/s
        self.w_max = 2.84       # rad/s

    def f(self):
        # drift term for  ẋ = v cosθ, ẏ = v sinθ, θ̇ = ω
        # since we treat v,ω as inputs, f(x)=0
        return np.zeros((3,1))

    def g(self):
        # columns map [v; ω] → ẋ
        θ = float(self.X[2])
        return np.array([
            [np.cos(θ), 0.0],
            [np.sin(θ), 0.0],
            [0.0,       1.0]
        ])

    def nominal_input(self, goal_xy):
        # simple proportional controller toward the waypoint
        dx, dy = goal_xy - self.X[:2].flatten()
        desired_theta = np.arctan2(dy, dx)
        θ_err = np.arctan2(np.sin(desired_theta - self.X[2,0]),
                           np.cos(desired_theta - self.X[2,0]))
        v = self.v_max * np.exp(-abs(θ_err))           # slow down when turning
        ω = np.clip(2.0*θ_err, -self.w_max, self.w_max)
        return np.array([[v],[ω]])
    
    def agent_barrier(self, obs):
        # reuse whatever BaseRobot’s barrier or override to include your robot radius
        return super().agent_barrier(obs)
