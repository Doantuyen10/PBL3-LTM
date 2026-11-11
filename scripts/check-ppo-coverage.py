"""Geometric coverage diagnostic; no Webots process or real UDP is involved."""
from pathlib import Path
import sys,json,re
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
FOLDER=ROOT/'drone-rl/webots_world/controllers/dieukhien'
sys.path.insert(0,str(FOLDER))
from ppo_bridge import PPOBridge, USER_DEFS
from dieukhien import COMM_RANGE_DRONE
world=(ROOT/'drone-rl/webots_world/worlds/VD2_vscode.wbt').read_text()
def position(block):
    return np.array([float(x) for x in re.search(r'\btranslation\s+([^\n]+)',block).group(1).split()])
users=np.array([position(world.split('DEF '+name+' E-puck {',1)[1].split('}',1)[0]) for name in USER_DEFS])
initial=position(world.split('\nMavic2Pro {',1)[1].split('}',1)[0])
results=[]
for altitude in [30.]:
    bridge=PPOBridge(FOLDER,altitude=altitude)
    try:
        pos=initial.copy();pos[2]=altitude;bridge.start(pos,users)
        start_dist=np.linalg.norm(users[:3]-pos,axis=1)
        coverage=[];first_drop=None;first_all_out=None
        while not bridge.done:
            target,_,_=bridge.next_target(pos,users)
            # Match the controller's 125 interpolated states per 1 s action.
            for tick in range(1,126):
                point=pos+(target-pos)*(tick/125)
                ok=np.linalg.norm(users[:3]-point,axis=1)<=COMM_RANGE_DRONE
                t=(bridge.env.t-1)+tick*.008
                coverage.append(ok)
                if first_drop is None and not ok.all():first_drop=t
                if first_all_out is None and not ok.any():first_all_out=t
            pos=target
        c=np.array(coverage)
        results.append(dict(altitude_m=altitude,initial_distances_m=start_dist.tolist(),
            initial_in_range=(start_dist<=COMM_RANGE_DRONE).tolist(),
            duration_s=bridge.env.t,per_client_in_range_pct=(100*c.mean(axis=0)).tolist(),
            all_three_in_range_pct=float(100*c.all(axis=1).mean()),
            all_three_out_of_range_pct=float(100*(~c.any(axis=1)).mean()),
            first_any_out_of_range_s=first_drop,first_all_out_of_range_s=first_all_out))
    finally:bridge.close()
print(json.dumps({'scope':'deterministic PPO rollout; geometric coverage only, not measured UDP connectivity',
                  'radius_m':COMM_RANGE_DRONE,'results':results},ensure_ascii=False,indent=2))
