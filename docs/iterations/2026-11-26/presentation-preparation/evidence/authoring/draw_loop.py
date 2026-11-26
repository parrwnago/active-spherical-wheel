from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch,FancyArrowPatch
ROOT=Path.cwd();OUT=ROOT/'results/presentation_preparation_20261126/figures'
fig,ax=plt.subplots(figsize=(9.4,4.8));ax.set_xlim(-.92,3.5);ax.set_ylim(-.65,2.67);ax.axis('off')
xs=[-.2,1.0,2.2];ys=[2.,.95,-.08]
labels=[['Current state q, v\nPrevious applied input','Momentum observer\nFiltered force + moment','Support locator\nPatch, point, shares'],['Prepare incoming drive\nHeading helper off / load fade','Scheduled LQR\n+ holding feedforward','Ground/load filters\nReferences + rolling FF'],['Hard input clipping\n3 torques + 2 belt speeds','Gazebo / DART step\nHold input for 5 × 1 ms','Contact truth logger\nEvaluation only']]
boxes={}
for r,y in enumerate(ys):
 for c,x in enumerate(xs):
  color='#E8F4EF' if (r,c)==(2,2) else '#EDF3F8'
  ax.add_patch(FancyBboxPatch((x-.48,y-.21),.96,.42,boxstyle='round,pad=.025',edgecolor='#267A63' if (r,c)==(2,2) else '#386785',facecolor=color,lw=1.3))
  ax.text(x,y,labels[r][c],ha='center',va='center',fontsize=9.2,color='#17334D');boxes[r,c]=(x,y)
def arrow(a,b,rad=0,color='#386785',style='-'):
 x,y=boxes[a];xx,yy=boxes[b]
 if abs(x-xx)>.1:
  start=(x+(.51 if xx>x else -.51),y);end=(xx-(.51 if xx>x else -.51),yy)
 else:start=(x,y+(.25 if yy>y else -.25));end=(xx,yy-(.25 if yy>y else -.25))
 ax.add_patch(FancyArrowPatch(start,end,arrowstyle='-|>',mutation_scale=11,linewidth=1.4,color=color,connectionstyle=f'arc3,rad={rad}',linestyle=style))
for a,b in [((0,0),(0,1)),((0,1),(0,2)),((0,2),(1,2)),((1,2),(1,1)),((1,1),(1,0)),((1,0),(2,0)),((2,0),(2,1))]:arrow(a,b)
arrow((2,1),(2,2),color='#267A63',style='--')
ax.plot([1.,1.,-.82,-.82,-.2],[-.32,-.57,-.57,2.5,2.5],color='#386785',lw=1.25)
ax.annotate('',xy=(-.2,2.25),xytext=(-.2,2.5),arrowprops=dict(arrowstyle='-|>',lw=1.25,color='#386785'))
ax.text(.76,2.51,'State feedback after each held interval',ha='center',fontsize=9,color='#526170')
ax.annotate('Timed path\nx*, v*, a*',xy=(2.72,.98),xytext=(3.08,1.10),ha='center',va='center',fontsize=8.8,color='#885D24',arrowprops=dict(arrowstyle='->',color='#885D24',lw=1.2))
ax.text(2.2,-.43,'Truth never feeds the observer-based command',ha='center',fontsize=8.4,color='#267A63')
fig.subplots_adjust(left=.02,right=.99,bottom=.05,top=.97)
fig.savefig(OUT/'current_loop.png',dpi=220);fig.savefig(OUT/'current_loop.pdf');plt.close(fig)
