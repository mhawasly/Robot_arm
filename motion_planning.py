import numpy as np
import robot2d
from discretise import  distance_to_obstacle
import math
import random


encoding=[5,15]


################################################################################


''' for a motion planning task, find a qualitative path
'''
class qualitative_motion_planning(object):
    def __init__(self,objects,encoding,manoeuvres,points, robot):
        self.Objects=objects
        self.encoding=encoding
        self.Manoeuvres=manoeuvres
        self.Points=points
        self.Robot=robot


#################


    def encode(self,pos):
        Z=[]
        for o in self.Objects:
            d=distance_to_obstacle(pos[-1],o)
            z=sum(np.where(d>=np.array(encoding),1,0))
            Z.append(z)
        return Z



#################



    def plan(self,start,end):

        #####################################################
        # all_points=[]
        # for key in self.Points:
        #     all_points+=self.Points[key]
        # import pdb; pdb.set_trace()
        # self.PRM(all_points, start , end)
        #
        # return
        ######################################################
        
        #find the region of start, end for Environment
        #compute coordinates
        if(self.Robot.set_joints(end)):
            pos=self.Robot.compute_pos()

            Z2=self.encode(pos)

            print 'end ', pos, Z2
            self.Robot.ax.plot(pos[-1][0],pos[-1][1], 'kd', ms=10)


            # Z2=[]
            # #find qualitative pos
            # for o in self.Objects:
            #     d=distance_to_obstacle(pos[-1],o)
            #     z=sum(np.where(d>=np.array(encoding),1,0))
            #     Z2.append(z)

        else:
            print "End position infeasible"
            return



        #import pdb; pdb.set_trace()

        #plan a route from Z1 to Z2
        # implement a more robust method here

        conf=start

        while True:
            replan=False
            if(self.Robot.set_joints(conf)):
                pos=self.Robot.compute_pos()

                Z1=self.encode(pos)
                print 'start ', pos, Z1
                self.Robot.ax.plot(pos[-1][0],pos[-1][1], 'kd', ms=10)

                # Z1=[]
                # #find qualitative pos
                # for o in self.Objects:
                #     d=distance_to_obstacle(pos,o)
                #     z=sum(np.where(d>=encoding,1,0))
                #     Z1.append(z)
            else:
                print "Start position infeasible"
                return
            plan=[]
            moves=[z2-z1 for (z1,z2) in zip(Z1,Z2)]

            plan.append(Z1)

            while plan[-1]!=Z2:
                r=random.randint(0,len(Z1)-1)
                if moves[r]!=0:

                    z_new=[0]*len(Z1)
                    z_new[r]+=moves[r]/abs(moves[r])
                    moves[r]-=moves[r]/abs(moves[r])
                    plan.append([z1+z2 for [z1,z2] in zip(plan[-1],z_new)])



            #print plan
            #implement the plan

            for t in range(len(plan)-1):
                replan=False
                #pos=self.Robot.compute_pos()
                conf=[j.angle for j in self.Robot.joints]


                if tuple(plan[t]+plan[t+1]) in self.Manoeuvres:
                    B_r12=self.Manoeuvres[tuple(plan[t]+plan[t+1])]

                    # for b in B_r12:
                    #     if random.random()<0.7:continue
                    #     b_p=self.Robot.compute_pos(b[0])
                    #     self.Robot.ax.plot(b_p[-1][0],b_p[-1][1],'rx', alpha=0.6)
                    #
                    # self.Robot.display_robot()


                b=self.find_closest_manoeuvre(conf,plan[t],plan[t+1])

                if not b:
                    print "No known route!"
                    replan=True
                    break

                b_X=[]
                b_Y=[]
                for bn in b:
                    b_p=self.Robot.compute_pos(bn)
                    b_X.append(b_p[-1][0])
                    b_Y.append(b_p[-1][1])

                self.Robot.ax.plot(b_X,b_Y,'r-')
                self.Robot.display_robot()

                #import pdb; pdb.set_trace()

                #conf=start
                #behaviour is a five point sequence to follow

                #simple motion planning to b[0]
                trials=0
                while True:

                    #plot b
                    b0=b[0]


                    print "Moving to start of manoeuvre", plan[t],"->",plan[t+1]

                    #import pdb; pdb.set_trace()
                    #conf=self.go_to_pose(b0)
                    #get points of plan[t]
                    points_t=Points[tuple(plan[t])]
                    points_t=random.sample(points_t, min(300,len(points_t)))
                    path_t=self.PRM(points_t,conf,b0)
                    for p_h in path_t:
                        self.Robot.display_robot_copy(p_h)
                        self.Robot.display_robot()
                        for i,j in reversed(list(enumerate(self.Robot.joints))):
                            if j.angle-p_h[i]==0: continue
                            self.Robot.rotate_joint(i,p_h[i]-j.angle)
                        self.Robot.display_robot()


                    # H=3
                    # step=[float(b0[0]-conf[0])/H,float(b0[1]-conf[1])/H,float(b0[2]-conf[2])/H]
                    # for h in range(H):
                    #     for i,j in enumerate(self.Robot.joints):
                    #         if step[i]!=0: self.Robot.rotate_joint(i,step[i])
                    #     self.Robot.display_robot()
                    #
                    # for i,j in enumerate(self.Robot.joints):
                    #     self.Robot.rotate_joint(i,b0[i]-conf[i]-H*step[i])
                    # self.Robot.display_robot()

                    #conf=[j.angle for j in self.Robot.joints]
                    #import pdb; pdb.set_trace()
                    if abs(sum([c-bb for c,bb in zip(conf,b0)]))<0.1:
                        # once there, move according to b
                        print "manoeuvre following",plan[t],"->",plan[t+1]
                        for n in [1,-1]:#range(1,len(b)):
                            bn=b[n]
                            # self.Robot.display_robot_copy(bn)
                            # step=[float(bn[0]-conf[0]),float(bn[1]-conf[1]),float(bn[2]-conf[2])]
                            # for i,j in enumerate(self.Robot.joints):
                            #     self.Robot.rotate_joint(i,step[i])
                            #     self.Robot.display_robot()
                            conf=self.go_to_pose(bn)
                        #import pdb; pdb.set_trace()
                        break
                    else:
                        print "failed to reach required state"
                        #import pdb; pdb.set_trace()
                        b=self.find_closest_manoeuvre_random(conf,plan[t],plan[t+1])
                        trials+=1
                        if trials==5: #Replan
                            replan=True
                            break


                #check if manoeuvre was successful
                if replan:
                    print "Replan...."
                    break

            if replan:
                conf=[j.angle for j in self.Robot.joints]
                continue




            #move inside the target region to the goal
            print "inside region",plan[-1]
            #import pdb; pdb.set_trace()

            #get all feasible points of region
            points=Points[tuple(plan[-1])]

            #subsample the points for efficiency

            points=random.sample(points, min(300,len(points)))

            A_path=self.PRM(points,conf,end)

            #plot A_path
            # print "following A* path: ", A_path
            #import pdb; pdb.set_trace()

            for p_h in A_path:
                self.Robot.display_robot_copy(p_h,colour=(0,1,0,0.5))
                self.Robot.display_robot()
                for i,j in reversed(list(enumerate(self.Robot.joints))):
                    if j.angle-p_h[i]==0: continue
                    self.Robot.rotate_joint(i,p_h[i]-j.angle)
                self.Robot.display_robot()

            # H=3
            # conf=[j.angle for j in self.Robot.joints]
            # step=[float(end[0]-conf[0])/H,float(end[1]-conf[1])/H,float(end[2]-conf[2])/H]
            # for h in range(H):
            #     for i,j in enumerate(self.Robot.joints):
            #         if step[i]==0: continue
            #         self.Robot.rotate_joint(i,step[i])
            #         self.Robot.display_robot()

            #import pdb; pdb.set_trace()
            conf=[j.angle for j in self.Robot.joints]
            if abs(sum([c-z for c,z in zip(conf,end)]))<0.05:
                break
            else:
                print "Replan..."



#################



    def go_to_pose(self,pose):
        self.Robot.display_robot_copy(pose)
        #import pdb; pdb.set_trace()
        H=3
        conf=[j.angle for j in self.Robot.joints]
        step=[float(pose[0]-conf[0])/H,float(pose[1]-conf[1])/H,float(pose[2]-conf[2])/H]
        for h in range(H):
            for i,j in enumerate(self.Robot.joints):
                if step[i]!=0: self.Robot.rotate_joint(i,step[i])
            self.Robot.display_robot()


        conf=[j.angle for j in self.Robot.joints]
        for i,j in enumerate(self.Robot.joints):
            step_=pose[i]-conf[i]#-H*step[i]
            if step_!=0: self.Robot.rotate_joint(i,step_)
        self.Robot.display_robot()

        conf=[j.angle for j in self.Robot.joints]
        return conf

#################



    '''get the closest manoeuvre that moves the point from region r1 to r2'''
    def find_closest_manoeuvre_random(self, point, r1, r2):
        #import pdb; pdb.set_trace()

        if tuple(r1+r2) in self.Manoeuvres:
            B_r12=self.Manoeuvres[tuple(r1+r2)]
            #d2=[((b[0][0]-point[0])**2+(b[0][1]-point[1])**2+(b[0][2]-point[2])**2) for b in B_r12]
            return B_r12[random.randint(0,len(B_r12)-1)]
        else: return False





#################



    '''get the closest manoeuvre that moves the point from region r1 to r2'''
    def find_closest_manoeuvre(self, point, r1, r2):
        #import pdb; pdb.set_trace()

        if tuple(r1+r2) in self.Manoeuvres:
            B_r12=self.Manoeuvres[tuple(r1+r2)]
            d2=[((b[0][0]-point[0])**2+(b[0][1]-point[1])**2+(b[0][2]-point[2])**2) for b in B_r12]
            return B_r12[d2.index(min(d2))]
        else: return False


#################


    def PRM(self,points, start, end):
        from scipy.spatial import Delaunay
        tri = Delaunay(points)  #Delaunay triangulation

        #cimport pdb; pdb.set_trace()
        # A* search -------------------------------------------------------
        #start pos is conf, find index of (closest) point in graph tri
        e_pos=self.Robot.compute_pos(end)
        [e_x,e_y]=e_pos[-1]


        D2=[((p[0]-end[0])**2+(p[1]-end[1])**2+(p[2]-end[2])**2) for p in tri.points]
        e=D2.index(min(D2))  # e is an index in tri.points
        goal=tri.points[e]  # goal is the closest goal point
        g_pos=self.Robot.compute_pos(goal)
        [g_x,g_y]=g_pos[-1]
        self.Robot.ax.plot(g_x,g_y, 'gx', ms=25)
        #self.Robot.ax.plot(g_x,g_y, 'rx', ms=25)

        s_pos=self.Robot.compute_pos(start)
        [s_x,s_y]=s_pos[-1]
        self.Robot.ax.plot(s_x,s_y, 'g+', ms=25)
        self.Robot.display_robot()

        pos=self.Robot.compute_pos()
        #self.Robot.ax.plot(pos[-1][0],pos[-1][1], 'bd', ms=25)


        cs_points=[self.Robot.compute_pos(p) for p in tri.points]
        cs_points_xy=[p[-1] for p in cs_points]
        #self.Robot.ax.plot([p[0] for p in cs_points_xy],[p[1] for p in cs_points_xy],'k.')
        #self.Robot.display_robot()


        #import pdb; pdb.set_trace()

        D1=[((p[0]-start[0])**2+(p[1]-start[1])**2+(p[2]-start[2])**2) for p in tri.points]
        s=D1.index(min(D1)) # s is an index in tri.points

        O=[s]   #open list of tri.points indices
        P={}    #parents
        P[s]=-1
        f=[0]   #f function
        g=[0]   #f function
        C=[]    #closed list

        #import pdb; pdb.set_trace()
        A_path=[]
        while len(O)!=0:
            #import pdb; pdb.set_trace()
            q=f.index(min(f)) #best point to expand. q is an index into O/f.
            #print "####",q,"####"
            o_q=O[q]        #o_q is the actual index into tri.points
            q_p=tri.points[o_q]     #q_p is the actual point
            q_pos=self.Robot.compute_pos(q_p)
            [q_x,q_y]=q_pos[-1]
            # self.Robot.ax.plot(q_x,q_y, 'bo', alpha=0.6)
            # self.Robot.display_robot(0.01)
            #f_q=g[q]
            g_q=g[q]

            del O[q]
            del f[q]
            del g[q]

            #import pdb; pdb.set_trace()
            #get neighbours of point q
            q_neighbours=tri.vertex_neighbor_vertices[1][tri.vertex_neighbor_vertices[0][o_q]:1+tri.vertex_neighbor_vertices[0][o_q+1]]  #neighbours of vertex v

            # print q_neighbours

            if e in q_neighbours:    #goal found
            #extract path
                # print "Goal found!"

                pp=e
                if s!=e: P[e]=o_q
                while pp!=-1:
                    A_path.insert(0, pp)
                    if len(A_path)>10: import pdb; pdb.set_trace()
                    pp=P[pp]
                break
            for nid,n in enumerate(q_neighbours):      #n is an index into tri.points
                if n in C: continue
                #print nid
                #if len(q_neighbours)>24 and random.random()<0.6: continue
                n_p=tri.points[n]
                n_pos=self.Robot.compute_pos(n_p)

                #[n_x,n_y]=n_pos[-2]
                # [n_x3,n_y3]=n_pos[-1]
                [n_x,n_y]=n_pos[-1]
                #import pdb; pdb.set_trace()
                # self.Robot.ax.plot([n_x,q_x],[n_y,q_y], 'g.-', alpha=0.6)
                # self.Robot.display_robot(0.01)
                base_1=[n_x,n_y]
                l_1=math.sqrt((q_x-n_x)**2+(q_y-n_y)**2)

                if l_1 !=0:
                    angle_1=np.arcsin((q_x-n_x)/l_1)

                    intersect=0
                    #check that the path is clear
                    for [x_ob,y_ob,w_ob,h_ob] in self.Objects:
                        #[x_ob,y_ob,w_ob,h_ob]=ob

                        if (n_x>x_ob and q_x < x_ob) or  (n_x<x_ob and q_x > x_ob):
                            intersect+=self.Robot.line_intersect2(base_1, angle_1, l_1, [y_ob,y_ob+h_ob], x_2=x_ob )
                        if (n_x>x_ob+w_ob and q_x < x_ob+w_ob) or  (n_x<x_ob+w_ob and q_x > x_ob+w_ob):
                            intersect+=self.Robot.line_intersect2(base_1, angle_1, l_1, [y_ob,y_ob+h_ob], x_2=x_ob+w_ob )
                        if (n_y>y_ob and q_y < y_ob) or  (n_y<y_ob and q_y > y_ob):
                            intersect+=self.Robot.line_intersect2(base_1, angle_1, l_1, [x_ob,x_ob+w_ob], y_2=y_ob )
                        if (n_y>y_ob+h_ob and q_y < y_ob+h_ob) or  (n_y<y_ob+h_ob and q_y > y_ob+h_ob):
                            intersect+=self.Robot.line_intersect2(base_1, angle_1, l_1, [x_ob,x_ob+w_ob], y_2=y_ob+h_ob )

                        if intersect>0:
                            break
                    if intersect>0:
                        print "intersection found. Edge removed."
                        self.Robot.ax.plot([n_pos[-1][0],q_pos[-1][0]],[n_pos[-1][1],q_pos[-1][1]],'r-')
                        self.Robot.display_robot()
                        #import pdb; pdb.set_trace()
                        continue




                n_g=g_q+math.sqrt((q_p[0]-n_p[0])**2+(q_p[1]-n_p[1])**2+(q_p[2]-n_p[2])**2)
                n_h=math.sqrt((goal[0]-n_p[0])**2+(goal[1]-n_p[1])**2+(goal[2]-n_p[2])**2)
                n_f=n_g+n_h

                if n in O and n_f>f[O.index(n)]:
                    pass
                elif n in C and n_f>f[C.index(n)]:
                    pass
                else:
                    O.append(n)
                    f.append(n_f)
                    g.append(n_g)
                    P[n]=o_q
            C.append(o_q)
            #self.Robot.ax.plot(q_x,q_y,'ro')
            #self.Robot.display_robot()


            #import pdb; pdb.set_trace()
        path=[]
        for h in A_path:
            p_h=tri.points[h]
            path.append(p_h)

        return path




################################################################################



if __name__=="__main__":

    import cPickle
    Manoeuvres=cPickle.load(open('manoeuvres.p','r'))
    Points=cPickle.load(open('points.p','r'))


    Objects=[]
    with open('environment.dat','r') as f:
        f.readline()
        while True:
            L=f.readline()
            if len(L)==0: break
            [X,Y,W,H]=[int(e) for e in L.split()]
            Objects.append([X,Y,W,H])



    E=robot2d.environment([tuple(o) for o in Objects])    # x,y,w,h
    E.display_environment()

    Robot=robot2d.robot(3,[10,8,6],[math.pi/2,math.pi/1.5,math.pi/2],environment=E, display=True)

    from matplotlib.patches import Rectangle, Wedge

    Cs=['b','y', 'r']
    for i,o in enumerate(Objects):
        Robot.ax.text(o[0], o[1], str(i))
        for j,e in enumerate(encoding):
            r=Rectangle((o[0]-e,o[1]), o[2]+2*e, o[3], color=Cs[i], alpha=0.1, edgecolor='none')
            Robot.ax.add_patch(r)
            r=Rectangle((o[0],o[1]-e), o[2], o[3]+2*e, color=Cs[i], alpha=0.1, edgecolor='none')
            Robot.ax.add_patch(r)
            c=Wedge((o[0],o[1]), e, 180, 270, color=Cs[i], alpha=0.1, edgecolor='none')
            Robot.ax.add_patch(c)
            c=Wedge((o[0]+o[2],o[1]), e, 270, 0, color=Cs[i], alpha=0.1, edgecolor='none')
            Robot.ax.add_patch(c)
            c=Wedge((o[0],o[1]+o[3]), e, 90, 180, color=Cs[i], alpha=0.1, edgecolor='none')
            Robot.ax.add_patch(c)
            c=Wedge((o[0]+o[2],o[1]+o[3]), e, 0, 90, color=Cs[i], alpha=0.1, edgecolor='none')
            Robot.ax.add_patch(c)


    Robot.display_robot()

    #import pdb; pdb.set_trace()
    qmp=qualitative_motion_planning(Objects,encoding,Manoeuvres,Points,Robot)

    qmp.plan([-math.pi/16,math.pi/16,0],[-math.pi/4,-math.pi/6,math.pi/4])
