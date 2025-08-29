# RMF Troubleshooting 

## Cleaning up rmf left over process

- Check if there any process alive.
```
ros2 node list 
ps -ef | egrep 'rmf_|rmf-|rmf/' | grep -v grep
```
```bash
echo "=== RMF-related processes ==="
ps -eo pid,ppid,sid,stat,cmd | egrep 'rmf_|rmf-|rmf/|gz sim|gazebo|rviz2|launch_ros|ros2 launch' | grep -v egrep
echo "=== Process tree (grep) ==="
pstree -pal | egrep 'rmf_|gz|gazebo|rviz2|launch_ros|python.*launch.py' || true

```

2) Kill launch parents by process group (this reaps <defunct> zombies)
```bash
# TERM first
for p in $(ps -eo pid,cmd | egrep '/opt/ros/.*/bin/ros2 launch ' | awk '{print $1}'); do
  sid=$(ps -o sid= -p "$p" | tr -d ' ')
  echo "TERM PGID $sid (parent PID $p)"
  kill -TERM -- "-$sid" 2>/dev/null
done
sleep 2
# KILL stragglers
for p in $(ps -eo pid,cmd | egrep '/opt/ros/.*/bin/ros2 launch ' | awk '{print $1}'); do
  sid=$(ps -o sid= -p "$p" | tr -d ' ')
  echo "KILL PGID $sid (parent PID $p)"
  kill -KILL -- "-$sid" 2>/dev/null
done
```


## HTOP
```
F5 -> Tree, change to tree view 
F4 -> Filter, filter for rmf
F9 -> Kill, Ensure all rmf_related process are killed
```