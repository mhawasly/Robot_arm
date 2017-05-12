import numpy as np
import math

import cPickle as pickle

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
for pf in range(200):
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
QPaths2=[]
#use the cartesian coordinates of the gripper to discretise into qualitative spans
for h,path in enumerate(Paths_cs):
    qpath=[]
    qpath_times_starts=[]
    qpath_times_ends=[]

    for t,p in enumerate(path):
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
        if len(qpath)==0 or qpath[-1]!=q:
            qpath.append(q)
            if len(qpath)!=1: qpath_times_ends.append(t-1)
            qpath_times_starts.append(t)


    qpath_times_ends.append(len(path)-1)
    #import pdb; pdb.set_trace()

    with open('qpaths/qpath%d_%d.dat'%(len(spans)+1,h),'w') as f:
        f.write('# (_'+'_'.join([str(s) for s in spans])+'_) start end\n')
        f.write('\n'.join(["%d %d %d %d %d"%(g[0][0],g[0][1],g[0][2],g[1],g[2]) for g in zip(qpath,qpath_times_starts,qpath_times_ends)]))

    # with open('qpaths/qpath%d_%d_times.dat'%(len(spans)+1,h),'w') as f:
    #     f.write('#\n')
    #     f.write('\n'.join([""%(qpath_times_starts[g],qpath_times_ends[g]) for g in range(len(qpath))]))

    QPaths.append(zip(qpath,qpath_times_starts,qpath_times_ends))
    QPaths2.append(zip(qpath[0:-1],qpath[1:],qpath_times_starts[0:-1],qpath_times_ends[0:-1]))
    #break




#analyse QPaths
#find all kinds of transitions

L=[[i]*len(qp) for i,qp in enumerate(QPaths)]

All_qpaths=zip([item for sublist in L for item in sublist],[item for sublist in QPaths for item in sublist])

L2=[[i]*len(qp) for i,qp in enumerate(QPaths2)]

All_qpaths2=zip([item for sublist in L2 for item in sublist],[item for sublist in QPaths2 for item in sublist])



#sort according to
All_qpaths2.sort(key=lambda x: (x[1][0],x[1][1]))


Behaviours={}
Ps=[]
k=-1
key=-1#All_qpaths2[0][1][0:2]
while True:
    if All_qpaths2[k][1][0:2]==key:
        #from the same group
        #get the behaviours on the boundary of the two qualitative states
        pid=All_qpaths2[k][0]
        end=All_qpaths2[k][1][3]

        P=Paths_js[pid][max(0,end-2):min(end+3,len(Paths_js[pid]))] #local behaviours that cross the boundary
        Ps.append(P)
        k+=1
        if k==len(All_qpaths2):
            break

    else:
        if len(Ps)!=0:
            #learn and save the group behaviour for key
            Behaviours[tuple([x for y in key for x in y])]=Ps
        #reset the repository
        Ps=[]
        k+=1
        if k<len(All_qpaths2):
            key=All_qpaths2[k][1][0:2]
        else:
            break


pickle.dump(Behaviours, open('behaviours.p','w'))
