from pathlib import Path
import sys,importlib.util,ast,time,json,csv,platform,hashlib,zipfile
import numpy as np
import torch,gymnasium,stable_baselines3
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv,VecNormalize
O=Path(__file__).resolve().parents[1];ROOT=O.parent;E=O/'minh_chung'
spec=importlib.util.spec_from_file_location('ref',E/'uav_bs_ppo_tham_chieu.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
torch.set_num_threads(1)
folder=ROOT/'drone-rl/webots_world/controllers/dieukhien'
vn=VecNormalize.load(str(folder/'vecnorm.pkl'),DummyVecEnv([m.make_env]));vn.training=False;vn.norm_reward=False
model=PPO.load(folder/'ppo_uav_bs.zip',device='cpu')
rows=[];traces={};runtime=[]
for seed in range(30):
 for policy in ['PPO','Hover']:
  env=m.UAVBSEnv();obs,_=env.reset(seed=seed);path=[env.pos.tolist()];rlist=[];ee=[];ret=0.;elapsed=[];hits=0
  while True:
   if policy=='PPO':
    start=time.perf_counter_ns();a=model.predict(vn.normalize_obs(obs[None,:]),deterministic=True)[0][0];elapsed.append((time.perf_counter_ns()-start)/1e6)
   else:a=np.zeros(2,dtype=np.float32)
   obs,r,term,trunc,info=env.step(a);ret+=r
   path.append(env.pos.tolist());rlist.append((env._rates()*env.B/env.K).tolist());ee.append(env.energy_used)
   hits+=int(np.any((env.pos[:2]==0)|(env.pos[:2]==env.area)))
   if term or trunc:break
  path=np.array(path);rates=np.array(rlist)
  rows.append(dict(seed=seed,policy=policy,steps=env.t,time_s=env.t*env.dt,bits_Mbit=info['bits']/1e6,energy_kJ=info['energy']/1000,EE_bit_J=info['EE'],rate_Mbps=info['bits']/(env.t*env.dt)/1e6,min_user_rate_Mbps=float(rates.min(axis=1).mean()/1e6),path_m=float(np.linalg.norm(np.diff(path[:,:2],axis=0),axis=1).sum()),reward=ret,boundary_steps=hits))
  if seed==0:traces[policy]={'path':path.tolist(),'rates':rates.tolist(),'energy':ee,'users':env.users.tolist()}
  runtime.extend(elapsed)
with (E/'ket_qua_30_seed.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
summary={}
metrics=['bits_Mbit','energy_kJ','EE_bit_J','rate_Mbps','min_user_rate_Mbps','path_m','reward','steps']
for policy in ['PPO','Hover']:
 summary[policy]={k:{'mean':float(np.mean([r[k] for r in rows if r['policy']==policy])),'sd':float(np.std([r[k] for r in rows if r['policy']==policy],ddof=1))} for k in metrics}
summary['paired']={}
for k in metrics:
 a=np.array([r[k] for r in rows if r['policy']=='PPO']);b=np.array([r[k] for r in rows if r['policy']=='Hover']);d=a-b
 summary['paired'][k]={'mean_difference':float(d.mean()),'sd_difference':float(d.std(ddof=1)),'positive_seeds':int((d>0).sum()),'relative_mean_pct':float((a.mean()/b.mean()-1)*100) if b.mean()!=0 else None}
# Dedicated steady-state component timing; no Webots/UDP.
obs,_=m.UAVBSEnv().reset(seed=0)
for _ in range(50):model.predict(vn.normalize_obs(obs[None,:]),deterministic=True)
times=[]
for _ in range(1000):
 t=time.perf_counter_ns();model.predict(vn.normalize_obs(obs[None,:]),deterministic=True);times.append((time.perf_counter_ns()-t)/1e6)
summary['inference_ms']={'n':1000,'warmup':50,'mean':float(np.mean(times)),'median':float(np.median(times)),'p95':float(np.percentile(times,95)),'max':float(np.max(times))}
with zipfile.ZipFile(folder/'ppo_uav_bs.zip') as z:
 d=json.loads(z.read('data'));summary['checkpoint']={k:d.get(k) for k in ['num_timesteps','n_envs','n_steps','batch_size','gamma','gae_lambda','n_epochs','learning_rate','policy_kwargs','seed']}
summary['versions']={'python':platform.python_version(),'numpy':np.__version__,'torch':torch.__version__,'gymnasium':gymnasium.__version__,'sb3':stable_baselines3.__version__,'platform':platform.platform(),'torch_threads':torch.get_num_threads()}
summary['cpu']=next((s.split(':',1)[1].strip() for s in Path('/proc/cpuinfo').read_text().splitlines() if s.startswith('model name')),'unknown')
summary['parameter_count']=sum(p.numel() for p in model.policy.parameters())
summary['syntax']=[]
for path in (ROOT/'drone-rl/webots_world/controllers').rglob('*.py'):
 ast.parse(path.read_text());summary['syntax'].append(str(path.relative_to(ROOT)))
summary['files']={}
for path in [folder/'ppo_uav_bs.zip',folder/'vecnorm.pkl',E/'uav_bs_ppo_tham_chieu.py',folder/'dieukhien.py']:
 summary['files'][str(path.relative_to(ROOT))]=hashlib.sha256(path.read_bytes()).hexdigest()
# Directly extract the actual observation helper from controller.
tree=ast.parse((folder/'dieukhien.py').read_text());fun=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='build_observation');scope={'np':np};exec(compile(ast.Module(body=[fun],type_ignores=[]),'controller-helper','exec'),scope)
samples=[]
for pos in [[0,0,0],[0,0,5],[10,-10,5]]:
 raw=scope['build_observation'](pos);act=model.predict(vn.normalize_obs(raw),deterministic=True)[0];assert np.isfinite(act).all() and (np.abs(act)<=1).all();samples.append({'position':pos,'action':act.tolist()})
summary['webots_helper_samples']=samples
# Boundary and mapping tests: geometry only.
true=np.array([[-100,-50,0],[50,-50,0],[100,100,0]],float);fallback=np.array([[50,-50,0],[-100,-50,0],[100,100,0]],float)
summary['mapping_case']={'uav':[100,-50,5],'true_distances':np.linalg.norm(true-[100,-50,5],axis=1).tolist(),'fallback_distances':np.linalg.norm(fallback-[100,-50,5],axis=1).tolist()}
summary['radius_tests']=[{'distance_m':d,'forward':bool(d<=150)} for d in [149.999,150,150.001]]
(E/'tong_hop.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2));(E/'quy_dao_seed_0.json').write_text(json.dumps(traces));(E/'thoi_gian_suy_luan_ms.json').write_text(json.dumps(times))
print(json.dumps(summary,ensure_ascii=False,indent=2))
