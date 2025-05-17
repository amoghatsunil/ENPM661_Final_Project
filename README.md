



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
* To test new start and end goal points, edit lines [31](https://github.com/amoghatsunil/ENPM661_Final_Project/blob/main/scripts/visibility_rrtStar.py#L309) and [137](https://github.com/amoghatsunil/ENPM661_Final_Project/blob/main/scripts/visibility_rrtStar.py#L310) in `scripts/visibility_rrtStar.py`.
