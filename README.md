# Setup and Launch for RRT* and RRT* with CBF

Before proceeding with the following launch, ensure the following.

Install libraries: numpy, matplotlib, and scipy.

For running CBF-QP, it requires cvxpy, gurobipy, and shapely.

```bash
pip install numpy matplotlib scipy cvxpy shapely
```

Once that is all done, follow these steps to set up and launch the project in Gazebo:  

## 1. Clone the Repository

First, clone the repository to your local machine.

You can do this from visual studio by clicking the source control buttom, then selecting the "Clone repository" option and then pasting the link to this Github. After that, you select your local folder and you select the top right icon that says "Select as Repository Destination."


## 2. Run following commands in the termial
The run the following command in the terminal

```bash
colcon build
```
```bash
export TURTLEBOT3_MODEL=waffle
```
```bash
source install/setup.bash
```
```bash
ros2 launch proj5_amogha_sagar_shreya map_world.launch.py
```

## 3. Run following commands in a new termial
In new terminal, do either of the following nav_rrtStar.py or nav_rrtStarCBF.py to test the gazebo simulation of rrt* or rrt* with CBF

```bash
python3 scripts/nav_rrtStar.py
```

```bash
python3 scripts/nav_rrtStarCBF.py
```

Changes to make in order to test new start and end goal points.

![image](https://github.com/user-attachments/assets/688f3247-cfd8-49bc-913c-9635f15b2e38)
https://github.com/amoghatsunil/ENPM661_Final_Project/blob/c819a4118ba3818f20f3e0956ce633b8fe226789/scripts/nav_rrtStar.py#L137
https://github.com/amoghatsunil/ENPM661_Final_Project/blob/c819a4118ba3818f20f3e0956ce633b8fe226789/scripts/nav_rrtStar.py#L31
 
 For our project,our 2D map and Gazebo map are not aligned similarly so we have to make the following transformation to ensure the robot and the gazebo world obstalces spawn in the positions similar to that of the 2D Map.

# Setup and Launch for Visibility RRT* and Visibility RRT* with CBF.
For this, you just either of the bottom 2 commands depending on what you want to test. This is only a 2D simulation.

```bash
python3 scripts/visibility_rrtStar.py
```
* To test new start and end goal points,Edit lines [309](https://github.com/amoghatsunil/ENPM661_Final_Project/blob/main/scripts/visibility_rrtStar.py#L309)
and [310](https://github.com/amoghatsunil/ENPM661_Final_Project/blob/main/scripts/visibility_rrtStar.py#L310)  in `scripts/visibility_rrtStar.py`.

* To tune the planner’s behavior, edit the arguments in the `VisibilityRRTStar(...)` call in `scripts/visibility_rrtStar.py` (around lines [312–323](https://github.com/amoghatsunil/ENPM661_Final_Project/blob/main/scripts/visibility_rrtStar.py#L312-L323)), for example:

    ```python
    lqr_rrt_star = VisibilityRRTStar(
        x_start=x_start, x_goal=x_goal,
        max_sampled_node_dist=0.5,      # how far each random sample can be
        max_rewiring_node_dist=1.0,     # radius for rewiring nearby nodes
        goal_sample_rate=0.1,           # probability of sampling the goal directly
        rewiring_radius=0.5,            # neighborhood radius when rewiring
        iter_max=1000,                  # max number of iterations
        solve_QP=True,                  # enforce CBF via quadratic program
        visibility=True,                # use visibility‐guided sampling
        collision_cbf=False,            # apply collision constraints via CBF
        show_animation=SHOW_ANIMATION   # toggle on‐screen plotting
    )
    ```

```bash
python3 scripts/visibility_rrtStar_cbf.py
```
* To test new start and end goal points,Edit lines [306](https://github.com/amoghatsunil/ENPM661_Final_Project/blob/main/scripts/visibility_rrtStar_cbf.py#L306)
and [307](https://github.com/amoghatsunil/ENPM661_Final_Project/blob/main/scripts/visibility_rrtStar_cbf.py#L307)  in `scripts/visibility_rrtStar_cbf.py`.

If you were to notice an error message "FileNotFoundError: [Errno 2] No such file or directory: 'ffmpeg'" when running either of the above simulations, run the following command in the terminal.


```bash
sudo apt-get install ffmpeg
```

and then restart the setup and simulation process.
