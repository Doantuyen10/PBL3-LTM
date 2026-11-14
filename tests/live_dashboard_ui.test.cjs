// DOM/canvas smoke test without browser, network or additional dependencies.
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');
const assert = require('node:assert/strict');
const context2d = new Proxy({}, {get: (target,key)=>target[key]||(()=>{}),set:(target,key,value)=>(target[key]=value,true)});
function element(){return {textContent:'',className:'',checked:true,children:[],events:{},append(...items){this.children.push(...items);},replaceChildren(...items){this.children=items;},get lastChild(){return this.children.at(-1);},addEventListener(name,fn){this.events[name]=fn;},getBoundingClientRect(){return {width:800,height:440};},getContext(){return context2d;}};}
const elements=new Map();
const get=id=>{if(!elements.has(id))elements.set(id,element());return elements.get(id);};
const pending=[];
let response={state:null,age_s:null},error=false;
const sandbox={document:{getElementById:get,createElement:element},window:{devicePixelRatio:1,addEventListener(){}},fetch:async()=>{if(error)throw Error('offline');return {ok:true,json:async()=>response};},setTimeout:fn=>pending.push(fn),AbortSignal,console,Blob,URL};
vm.createContext(sandbox);
const source=fs.readFileSync(path.join(__dirname,'../drone-rl/webots_world/controllers/dieukhien/live/app.js'),'utf8');
const tick=()=>new Promise(resolve=>setImmediate(resolve));
(async()=>{
 vm.runInContext(source,sandbox);await tick();assert.match(get('connection').textContent,/chờ GPS/);
 response={age_s:0,state:{session:'one',sim_time:1,position:[0,0,30],target:[2,3,30],speed:1,altitude:30,battery:39000,policy_step:1,phase:'PPO điều hướng',ns3_enabled:true,feedback:{assist_weight:.2},packets:{received:1,forwarded:1,dropped:0,verdict:{pdr:1,e2e_delay_ms:5,throughput_bps:3000}},users:[{name:'E_PUCK_CLIENT_1',position:[-100,-50,0],distance:115,snr_db:12,link_ok:true}],events:[{sender:'E_PUCK_CLIENT_1',seq:1,sim_time:1,forwarded:true}]}};
 await pending.shift()();assert.match(get('connection').textContent,/LIVE/);assert.equal(get('altitude').textContent,'30.0 m');assert.equal(get('pdr').textContent,'100.0 %');assert.equal(get('links').children.length,1);assert.equal(get('events').children.length,1);
 response.age_s=4;await pending.shift()();assert.match(get('connection').textContent,/Dữ liệu cũ/);
 error=true;await pending.shift()();assert.match(get('connection').textContent,/Mất kết nối/);
 error=false;response.age_s=0;response.state.session='two';response.state.sim_time=0;await pending.shift()();assert.equal(vm.runInContext('history.length',sandbox),1);
 get('clear').events.click();assert.equal(vm.runInContext('history.length',sandbox),0);
 console.log('PASS: waiting, live rendering, stale data, disconnect, session reset, clear trail');
})().catch(err=>{console.error(err);process.exitCode=1;});
