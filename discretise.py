import numpy as np
import math

def smallest_dist_to_segment(x,y,range_l, X_l=None, Y_l=None ):
    if Y_l is not None:
        w=range_l[1]-range_l[0]
        t= float(x-range_l[0])/w
        if t>=0 and t<=1: d=abs(y-Y_l)
        elif t<0: d=math.sqrt((x-range_l[0])**2+(y-Y_l)**2)

        elif t>1: d=math.sqrt((x-range_l[1])**2+(y-Y_l)**2)
    elif X_l is not None:
        h=range_l[1]-range_l[0]
        t= float(y-range_l[0])/h
        if t>=0 and t<=1: d=abs(x-X_l)
        elif t<0: d=math.sqrt((x-X_l)**2+(y-range_l[0])**2)
        elif t>1: d=math.sqrt((x-X_l)**2+(y-range_l[1])**2)

    return d




#define the qualitiative spans
spans=np.array([5, 15])   # [0-5],]5-15], ]15,]


#read the environment file
O=[]
with open('environment.dat','r') as f:
    f.readline( ) #comment
    while True:
        obs=f.readline()

        if len(obs)==0: break
        O.append([int(z) for z in obs.split()])

#read the files from paths foldes
Dir="paths/"


Paths_js=[] #joints
Paths_cs=[] #cartesian

import os
for pf in range(110):
    P1=[]
    P2=[]
    with open(Dir+"path_"+str(pf)+".dat", 'r') as f:
        f.readline()    #comment
        while True:
            pose=f.readline()

            if len(pose)==0: break
            num_of_joints=len(pose.split())/3       #len(pose)=2*joint_num(arms)+joint_num(joints)

            #import pdb; pdb.set_trace()
            P1.append([float(p) for p in pose.split()[-num_of_joints:]])
            P2.append([float(p) for p in pose.split()[0:2*num_of_joints:]])
    Paths_js.append(P1)
    Paths_cs.append(P2)
    #break

QPaths=[]
#use the cartesian coordinates of the gripper to discretise into qualitative spans
for h,path in enumerate(Paths_cs):
    qpath=[]
    for p in path:
        x=p[-2];y=p[-1]     #gripper
        q=[]        #qualitative descriptor
        for o in O:
            x_o=o[0]; y_o=o[1]; w_o=o[2]; h_o=o[3]

            #find distance to obstacle
            d=min(smallest_dist_to_segment(x,y,[x_o,x_o+w_o], None, y_o ),\
            smallest_dist_to_segment(x,y,[x_o,x_o+w_o], None, y_o+h_o ),\
            smallest_dist_to_segment(x,y,[y_o,y_o+h_o], x_o ),\
            smallest_dist_to_segment(x,y,[y_o,y_o+h_o], x_o+w_o ))



            z=sum(np.where(d>=spans,1,0))   #qualitative descriptor for o
            q.append(z)
        if len(qpath)==0 or qpath[-1]!=q: qpath.append(q)


    with open('qpaths/qpath%d_%d.dat'%(len(spans)+1,h),'w') as f:
        f.write('#'+' '.join([str(s) for s in spans])+'\n')
        f.write('\n'.join(["%d %d %d"%(g[0],g[1],g[2]) for g in qpath]))

    QPaths.append(qpath)
    #break
