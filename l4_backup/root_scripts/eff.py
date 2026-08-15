import xml.etree.ElementTree as ET
r = ET.parse('/root/h12/unitree_rl_gym/resources/robots/h1_2/h1_2.urdf').getroot()
BODY = ('hip','knee','ankle','torso','shoulder','elbow','wrist')
for j in r.findall('joint'):
    n = j.get('name')
    if j.get('type') not in ('revolute','continuous') or not any(b in n for b in BODY):
        continue
    lim = j.find('limit')
    print(f"{n:32s} effort={lim.get('effort'):>7s} vel={lim.get('velocity'):>7s}")
