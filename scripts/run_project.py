"""Load project .env and launch Webots or its external PPO controller on Linux."""
from pathlib import Path
import argparse,os,shlex,sys
ROOT=Path(__file__).resolve().parents[1]

def environment():
    env=os.environ.copy()
    path=ROOT/'.env'
    if path.exists():
        for line_no,line in enumerate(path.read_text().splitlines(),1):
            line=line.strip()
            if not line or line.startswith('#'):continue
            if '=' not in line:raise ValueError(f'.env line {line_no}: expected KEY=value')
            key,value=line.split('=',1);key=key.strip()
            if key not in {'WEBOTS_HOME','PYTHON_EXECUTABLE','WEBOTS_WORLD','DISPLAY'}:
                raise ValueError(f'.env line {line_no}: unsupported key {key}')
            tokens=shlex.split(value,comments=True)
            if len(tokens)!=1:raise ValueError(f'.env line {line_no}: quote paths containing spaces')
            env.setdefault(key,tokens[0])
    return env

def resolve(value):
    p=Path(value).expanduser()
    return p if p.is_absolute() else ROOT/p

def command(mode,env):
    home=resolve(env.get('WEBOTS_HOME','/usr/local/webots'))
    python=resolve(env.get('PYTHON_EXECUTABLE','.venv/bin/python'))
    world=resolve(env.get('WEBOTS_WORLD','drone-rl/webots_world/worlds/VD2_vscode.wbt'))
    if mode=='webots':
        if not world.is_file():raise FileNotFoundError(f'World not found: {world}')
        args=[str(home/'webots'),'--mode=pause',str(world)]
    else:
        api=home/'lib/controller/python'
        if not api.is_dir():raise FileNotFoundError(f'Webots controller API not found: {api}')
        env['WEBOTS_HOME']=str(home)
        for key,path in [('PYTHONPATH',api),('LD_LIBRARY_PATH',home/'lib/controller')]:
            env[key]=str(path)+(os.pathsep+env[key] if env.get(key) else '')
        args=[str(python),'-u',str(ROOT/'drone-rl/webots_world/controllers/dieukhien/dieukhien.py')]
    if not Path(args[0]).is_file():raise FileNotFoundError(f'Executable not found: {args[0]}')
    return args

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['webots','controller'])
    parser.add_argument('--dry-run',action='store_true',help='Show command without starting it')
    args=parser.parse_args()
    try:
        env=environment();cmd=command(args.mode,env)
        if args.dry_run:print(shlex.join(cmd))
        else:
            os.chdir(ROOT)
            os.execve(cmd[0],cmd,env)
    except (ValueError,OSError) as exc:parser.exit(1,f'{exc}\n')
