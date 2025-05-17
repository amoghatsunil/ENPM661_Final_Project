



```bash
colcon build
```

```bash
export TURTLEBOT3_MODEL=waffle
```
```bash
ros2 launch proj5_amogha_sagar_shreya map_world.launch.py
```

```bash
cd src/proj5_amogha_sagar_shreya/scripts/
```

```bash
python3 nav_rrtStar.py
```

```bash
python3 nav_rrtStarCBF.py
```

Changes to make in order to test new start and end goal points.

![image](https://github.com/user-attachments/assets/688f3247-cfd8-49bc-913c-9635f15b2e38)
https://github.com/amoghatsunil/ENPM661_Final_Project/blob/c819a4118ba3818f20f3e0956ce633b8fe226789/scripts/nav_rrtStar.py#L137
https://github.com/amoghatsunil/ENPM661_Final_Project/blob/c819a4118ba3818f20f3e0956ce633b8fe226789/scripts/nav_rrtStar.py#L31
 (todo) write about conversion from algo to gazebo coordinate systemn. (x,y  --> (y, 5-x)

todo
Time and space complexity analysis

```bash
python3 visibility_rrtStar.py
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
python3 visibility_rrtStar_cbf.py
```
* To test new start and end goal points,Edit lines [306](https://github.com/amoghatsunil/ENPM661_Final_Project/blob/main/scripts/visibility_rrtStar_cbf.py#L306)
and [307](https://github.com/amoghatsunil/ENPM661_Final_Project/blob/main/scripts/visibility_rrtStar_cbf.py#L307)  in `scripts/visibility_rrtStar_cbf.py`.
