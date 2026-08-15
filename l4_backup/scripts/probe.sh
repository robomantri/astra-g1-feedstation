U=/root/h12/unitree_rl_gym/resources/robots/h1_2/h1_2.urdf
echo "=== LINKS ==="
grep -oP '(?<=<link name=")[^"]+' $U | tr '\n' ' '; echo
echo "=== REVOLUTE JOINTS ==="
python3 - <<'PY'
import xml.etree.ElementTree as ET
r = ET.parse('/root/h12/unitree_rl_gym/resources/robots/h1_2/h1_2.urdf').getroot()
js = [j.get('name') for j in r.findall('joint') if j.get('type') in ('revolute','continuous')]
print(len(js), "actuated joints:")
for j in js: print("   ", j)
PY
M=/root/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/mdp
echo "=== velocity mdp dir ==="; ls $M
echo "=== velocity rewards.py funcs ==="; grep -oP '^def \w+' $M/rewards.py
