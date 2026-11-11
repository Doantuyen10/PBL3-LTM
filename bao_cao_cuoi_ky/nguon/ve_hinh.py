from pathlib import Path
import json,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
O=Path(__file__).resolve().parents[1];H=O/'hinh';R=O/'minh_chung/relay_3epuck'
D=json.loads((R/'evaluation_rollouts.json').read_text());plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
def save(fig,n):fig.tight_layout();fig.savefig(H/n,dpi=180,bbox_inches='tight');plt.close(fig)
a=np.array([r['coverage_all3'] for r in D['heldout']])*100;b=np.array([r['coverage_all3'] for r in D['hover']])*100
fig,ax=plt.subplots(figsize=(8,4));ax.bar(['PPO relay mới','Đứng yên'],[a.mean(),b.mean()],yerr=[a.std(ddof=1),b.std(ddof=1)],capsize=6,color=['#157e82','#8392a4']);ax.set(ylabel='Thời gian phủ đồng thời ba client (%)',ylim=(0,110),title='64 seed giữ riêng • trung bình ± SD');save(fig,'ket_qua_tong_hop.png')
fig,ax=plt.subplots(figsize=(10,4));ax.plot(range(1000,1064),a,'o-',label='PPO relay',color='#157e82',markersize=3);ax.plot(range(1000,1064),b,'x',label='Đứng yên',color='#bd664b');ax.set(xlabel='Seed',ylabel='Phủ đồng thời ba client (%)',ylim=(-5,105));ax.legend();ax.grid(alpha=.2);save(fig,'so_sanh_ghep_cap.png')
rows=[json.loads(l) for l in (R/'progress.jsonl').read_text().splitlines()]
fig,ax=plt.subplots(figsize=(9,4));ax.plot([r['steps'] for r in rows],[r['exact_coverage']*100 for r in rows],label='World danh định');ax.plot([r['steps'] for r in rows],[r['validation_coverage']*100 for r in rows],label='Validation seed 100–107');ax.axvline(262144,color='gray',ls='--',label='Checkpoint được chọn');ax.set(xlabel='Bước huấn luyện',ylabel='Thời gian phủ đủ client (%)',ylim=(99.5,100.1));ax.legend();ax.grid(alpha=.2);save(fig,'tien_trinh.png')
u=np.array(D['exact']['users'])-[500,500,0];fig,ax=plt.subplots(figsize=(7,5));colors=['#246d9e','#bb6b33','#378245']
for i in range(3):
 ax.add_patch(plt.Circle(u[i,:2],np.sqrt(150**2-30**2),color=colors[i],fill=False,alpha=.6));ax.scatter(*u[i,:2],c=colors[i],label=f'Client {i+1}')
ax.scatter(*u[3,:2],marker='s',c='black',label='Server');ax.scatter(0,0,marker='^',c='red',label='UAV ban đầu');ax.set(xlim=(-255,255),ylim=(-210,255),xlabel='x Webots (m)',ylabel='y Webots (m)',title='Biên phủ sóng chiếu xuống mặt phẳng ngang • H = 30 m');ax.set_aspect('equal');ax.legend(fontsize=8);save(fig,'vung_phu.png')
def diagram(name,boxes,arrows,foot):
 fig,ax=plt.subplots(figsize=(11,5));ax.set(xlim=(0,11),ylim=(0,5));ax.axis('off')
 for x,y,w,h,text in boxes:
  ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=.05',fc='#17394f',ec='none'));ax.text(x+w/2,y+h/2,text,ha='center',va='center',color='white',fontsize=10)
 for x,y,xx,yy in arrows:ax.annotate('',xy=(xx,yy),xytext=(x,y),arrowprops=dict(arrowstyle='->',lw=1.6,color='#15858a'))
 ax.text(5.5,.15,foot,ha='center',fontsize=9);save(fig,name)
diagram('kien_truc.png',[(.1,2,2.2,1.2,'3 client e-puck\nChu kỳ 2 s mô phỏng'),(3.6,2,3,1.2,'UAV relay :8081\nKhoảng cách ≤150 m'),(8,3.1,2.7,1,'Server :9000'),(8,1.1,2.7,1,'C++ log :8080'),(3.6,3.8,3,.9,'PPO relay + VecNormalize')],[(2.4,2.6,3.5,2.6),(6.7,2.9,7.9,3.6),(6.7,2.3,7.9,1.6),(5.1,3.7,5.1,3.3)],'Hai nhánh UDP độc lập; chưa có phản hồi kênh từ ns-3. Server là nút thứ tư trong quan sát PPO.')
diagram('luu_do.png',[(.1,3.5,3,1,'Nạp PPO + chuẩn hóa\nKiểm tra hash, DEF'),(4,3.5,2.7,1,'Cất cánh đến 30 m\n5 m/s'),(7.7,3.5,3,1,'Quan sát 18 chiều\nPPO xuất gia tốc'),(7.7,1.6,3,1,'Bước mô hình 1 s\nNội suy 125 × 8 ms'),(3.9,1.6,2.9,1,'Cập nhật Supervisor\nNhận và relay UDP'),(.1,1.6,2.8,1,'Hết 200 s hoặc pin\nGiữ vị trí, tiếp tục relay')],[(3.2,4,3.9,4),(6.8,4,7.6,4),(9.2,3.4,9.2,2.7),(7.6,2.1,6.9,2.1),(3.8,2.1,3,2.1)],'Nhận gói và kiểm tra khoảng cách diễn ra ở mọi bước, cả khi cất cánh. Chưa hết episode: tiếp tục vòng PPO.')
q=np.array(D['exact']['path'])-[500,500,0]
fig,axs=plt.subplots(1,2,figsize=(11,4.5))
axs[0].plot(q[:,0],q[:,1],color='#157e82',label='PPO relay')
for i in range(3):axs[0].scatter(*u[i,:2],color=colors[i],label=f'Client {i+1}')
axs[0].scatter(*u[3,:2],marker='s',c='black',label='Server');axs[0].scatter(*q[0,:2],marker='^',c='red',label='Khởi đầu');axs[0].set(xlabel='x (m)',ylabel='y (m)',title='Quỹ đạo tại world hiện tại');axs[0].legend(fontsize=8);axs[0].set_aspect('equal',adjustable='datalim')
d=np.linalg.norm(q[:,None,:]-u[None,:3,:],axis=2)
for i in range(3):axs[1].plot(d[:,i],color=colors[i],label=f'Client {i+1}')
axs[1].axhline(150,c='red',ls='--',label='Ngưỡng 150 m');axs[1].set(xlabel='Thời gian mô hình (s)',ylabel='Khoảng cách 3D (m)',title='Khoảng cách đến từng client');axs[1].legend(fontsize=8)
fig.suptitle('Độ cao 30 m • kết quả động học, chưa phải đo gói UDP',fontsize=11);save(fig,'quy_dao_va_khoang_cach.png')
