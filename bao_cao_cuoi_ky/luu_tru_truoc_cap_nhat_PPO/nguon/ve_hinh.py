from pathlib import Path
import json,csv,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
O=Path(__file__).resolve().parents[1];H=O/'hinh';S=json.loads((O/'minh_chung/tong_hop.json').read_text());T=json.loads((O/'minh_chung/quy_dao_seed_0.json').read_text())
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'axes.titleweight':'bold','figure.facecolor':'white','savefig.facecolor':'white'})
C=['#197c83','#8696a8']
fig,axs=plt.subplots(1,3,figsize=(11,3.8))
for ax,k,title in zip(axs,['rate_Mbps','energy_kJ','EE_bit_J'],['Tốc độ tổng (Mb/s)','Năng lượng (kJ)','Hiệu quả năng lượng (kbit/J)']):
 scale=1000 if k=='EE_bit_J' else 1
 ax.bar(['PPO','Đứng yên'],[S[p][k]['mean']/scale for p in ['PPO','Hover']],yerr=[S[p][k]['sd']/scale for p in ['PPO','Hover']],color=C,capsize=5,width=.55)
 ax.set_title(title,fontsize=11);ax.set_ylim(bottom=0);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
fig.suptitle('30 seed ghép cặp • trung bình ± độ lệch chuẩn',fontsize=12);fig.tight_layout();fig.savefig(H/'ket_qua_tong_hop.png',dpi=200);plt.close(fig)
rows=list(csv.DictReader((O/'minh_chung/ket_qua_30_seed.csv').open()));p=np.array([float(r['rate_Mbps']) for r in rows if r['policy']=='PPO']);b=np.array([float(r['rate_Mbps']) for r in rows if r['policy']=='Hover'])
fig,axs=plt.subplots(1,2,figsize=(10,4));axs[0].scatter(b,p,c=C[0],s=30);v=max(p.max(),b.max())+.2;axs[0].plot([0,v],[0,v],'--',color=C[1]);axs[0].set(xlabel='Đứng yên (Mb/s)',ylabel='PPO (Mb/s)',title='Mỗi điểm là một seed',xlim=(0,v),ylim=(0,v));axs[1].bar(np.arange(30),p-b,color=[C[0] if x>=0 else '#bd5b44' for x in p-b]);axs[1].axhline(0,color='#444444',lw=.8);axs[1].set(xlabel='Seed',ylabel='PPO − đứng yên (Mb/s)',title='Chênh lệch theo kịch bản');fig.tight_layout();fig.savefig(H/'so_sanh_ghep_cap.png',dpi=200);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(10,4.5))
for policy,col in zip(['PPO','Hover'],C):
 tr=np.array(T[policy]['path']);rates=np.array(T[policy]['rates'])/1e6
 axs[0].plot(tr[:,0],tr[:,1],color=col,label='PPO' if policy=='PPO' else 'Đứng yên',lw=2)
 axs[1].plot(np.arange(1,len(rates)+1),rates.sum(axis=1),color=col,label=policy)
u=np.array(T['PPO']['users']);start=np.array(T['PPO']['path'])[0];axs[0].scatter(u[:,0],u[:,1],marker='s',c='#bd5b44',label='Người dùng');axs[0].scatter(start[0],start[1],marker='^',c='#333333',label='Khởi đầu');axs[0].set(xlim=(0,1000),ylim=(0,1000),xlabel='x (m)',ylabel='y (m)',title='Quỹ đạo tại seed 0');axs[0].legend(fontsize=8);axs[1].set(xlabel='Thời gian mô hình (s)',ylabel='Tốc độ tổng (Mb/s)',title='Diễn biến tốc độ tại seed 0');axs[1].legend();fig.tight_layout();fig.savefig(H/'quy_dao_va_toc_do.png',dpi=200);plt.close(fig)
def new(name,w=11,h=5):
 fig,ax=plt.subplots(figsize=(w,h));ax.axis('off');ax.set_xlim(0,11);ax.set_ylim(0,h);return fig,ax

def box(ax,x,y,w,h,t,color='#15324f'):
 ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.08',facecolor=color,edgecolor='none'));ax.text(x+w/2,y+h/2,t,ha='center',va='center',color='white',fontsize=10)
def arrow(ax,a,b):ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='->',lw=1.8,color='#197c83'))
fig,ax=new('architecture',h=5)
box(ax,.1,2.4,2.4,1.3,'3 client e-puck\nJSON / UDP');box(ax,3.4,2.4,3,1.3,'UAV relay :8081\nPPO + ngưỡng 150 m','#197c83');box(ax,8,3.2,2.8,1.1,'Server :9000');box(ax,8,1.4,2.8,1.1,'C++ log :8080','#60758b');arrow(ax,(2.6,3),(3.3,3));arrow(ax,(6.5,3.2),(7.9,3.7));arrow(ax,(6.5,2.7),(7.9,1.9));ax.text(5.5,.5,'Hai nhánh UDP độc lập • chưa có phản hồi chất lượng kênh vào PPO',ha='center',fontsize=11);fig.tight_layout();fig.savefig(H/'kien_truc.png',dpi=200);plt.close(fig)
fig,ax=new('flow',h=6)
box(ax,.1,4.65,3,1,'Khởi tạo Supervisor\nPPO và VecNormalize');box(ax,4,4.65,3,1,'Nhận gói (nếu có)\nTìm vị trí nguồn');box(ax,8.1,4.65,2.6,1,'Tính khoảng cách\nSo với 150 m');arrow(ax,(3.2,5.1),(3.9,5.1));arrow(ax,(7.1,5.1),(8,5.1))
box(ax,6.4,2.75,4.2,1,'Trong vùng: gửi server và log\nNgoài vùng: bỏ gói','#197c83');arrow(ax,(9.4,4.55),(9.4,3.85));box(ax,.2,2.75,4.3,1,'Chưa đạt ngưỡng cao: tăng z\nĐã đạt: PPO dự đoán vx, vy');arrow(ax,(6.3,3.25),(4.6,3.25));box(ax,.2,.7,4.3,1,'Ghi vị trí • resetPhysics\nChuyển bước mô phỏng');arrow(ax,(2.3,2.65),(2.3,1.8));ax.text(7.9,1.25,'Lặp đến khi Webots dừng\nΔt = 8 ms trong world',ha='center',fontsize=11);fig.tight_layout();fig.savefig(H/'luu_do.png',dpi=200);plt.close(fig)
fig,ax=new('rl',h=4)
box(ax,.1,1.4,2.5,1.3,'Môi trường UAV-BS\nKênh + năng lượng');box(ax,3.5,1.4,3.1,1.3,'Quan sát 18 chiều\nVecNormalize','#197c83');box(ax,7.6,1.4,3,1.3,'Chính sách PPO\nGia tốc 2 chiều');arrow(ax,(2.7,2),(3.4,2));arrow(ax,(6.7,2),(7.5,2));ax.annotate('',xy=(1.35,1.3),xytext=(9.1,1.3),arrowprops=dict(arrowstyle='->',connectionstyle='arc3,rad=-.20',color='#197c83',lw=1.8));ax.text(5.4,.05,'Hành động → động học → kênh & năng lượng → phần thưởng',ha='center',fontsize=10);fig.tight_layout();fig.savefig(H/'vong_lap_rl.png',dpi=200);plt.close(fig)
print('6 figures created')
