With `catkin_make`:
```bash
catkin_make tests
source devel/setup.bash
```

With `catkin tools`:
```bash
catkin test
source devel/setup.bash
```

Launch the visual tests:
```bash
rostest ram_path_planning services_clients.launch use_gui:=true
```

Press `Enter` to skip to the next test
