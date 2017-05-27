import math
import pdb
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib as mpl



################################################################################

class robot(object):

    def __init__(self, num_joints, arm_comp_lengths, joint_ranges, environment, display=False):
        #define parameters here
        self.num_joints=num_joints
        self.num_arm_comps=num_joints
        self.environment=environment
        self.display=display
        self.joints=[]
        self.arm_comps=[]
        #self.pos=[]
        self.comp_plot=[]   #refernce for the plotted arm comps
        self.comp_plot_copy=[]   #refernce for the plotted arm comps
        self.path=[]


        if self.display:
            plt.ion()
            self.ax = self.environment.ax

        for j in range(num_joints):
            x= joint(joint_ranges[j])
            self.joints.append(x)
            y=arm_comp(arm_comp_lengths[j])
            self.arm_comps.append(y)

#################

    def rotate_joint(self,joint_id, theta):
        step_size=math.pi/36
        num_steps=0
        steps=[]

        if theta>=step_size:
            #pdb.set_trace()
            num_steps=abs(int(theta/step_size))
            sign=num_steps/(theta/step_size)
            limit_reached=0

            steps=[sign*step_size]*num_steps
        steps.append(theta-num_steps*step_size)


        for s in steps:
            #check for joint limit
            limit_reached=self.joints[joint_id].rotate(s)

            if limit_reached:
                print "limit reached, joint ", joint_id
                break
            if self.arm_obstructed():
                self.joints[joint_id].rotate(-1*s);
                print "Obstruction"
                break

            self.display_robot()
            self.path.append([ self.compute_pos(),[j.angle for j in self.joints] ] )







        #self.pos=self.compute_pos()


#################
    def set_joints(self,joints):
        #pdb.set_trace()
        for i,j in enumerate(joints):
            self.joints[i].angle=j

        return not self.arm_obstructed()


#################




    def arm_obstructed(self):
        obstructed=False
        bases=self.compute_pos()
        bases.insert(0,(0,0))

        O=self.environment.objects
        O.append((-30,-1,29,1))  # ground
        O.append((1,-1,29,1))  # ground


        for o in O:
            for i,c in enumerate(self.arm_comps):
                if self.line_intersect2(bases[i], sum(self.joints[a].angle for a in range(i+1)), c.length, (o[1],o[1]+o[3]), o[0])==1 or \
                self.line_intersect2(bases[i], sum(self.joints[a].angle for a in range(i+1)), c.length, (o[1],o[1]+o[3]), o[0]+o[2])==1 or \
                self.line_intersect2(bases[i], sum(self.joints[a].angle for a in range(i+1)), c.length, (o[0],o[0]+o[2]), None, o[1])==1 or \
                self.line_intersect2(bases[i], sum(self.joints[a].angle for a in range(i+1)), c.length, (o[0],o[0]+o[2]), None, o[1]+o[3])==1:
                    #pdb.set_trace()
                    obstructed=True


                    break
            if obstructed: break
        return obstructed


#################

    def line_intersect(self, base_1, angle_1, l_1, range_2, x_2=None , y_2=None):
        #line 1 starts at base_1, extends for l_1 at angle_1
        #line 2 is a straight x=x_2 or y=y_2 in range_2
        intersect=0


        if x_2!=None:
            y=math.tan(math.pi/2 - angle_1)*(x_2-base_1[0]) + base_1[1]   # y of intersection
            if y>=base_1[1] and y<=(base_1[1]+l_1*math.sin(math.pi/2-angle_1)) and   y>=range_2[0] and y<=range_2[1]:
                intersect=1

        elif y_2!=None:
            if abs(math.pi/2 - angle_1)<=0.01:
                if y_2==base_1[1]: intersect=1
                else: intersect=0
            else:
                x=base_1[0] + (y_2-base_1[1])/math.tan(math.pi/2 - angle_1)
                if x>=base_1[0] and x<=(base_1[0]+l_1*math.cos(math.pi/2-angle_1)) and x>=range_2[0] and x<=range_2[1]:
                    intersect=1


        # if intersect:
        #     print "Intersection: " , (base_1, angle_1, l_1, range_2, x_2 , y_2)
            #pdb.set_trace()
        return intersect


#################

    def line_intersect2(self, base_1, angle_1, l_1, range_2, x_2=None , y_2=None):
        #line 1 starts at base_1, extends for l_1 at angle_1
        #line 2 is a straight x=x_2 or y=y_2 in range_2
        intersect=0


        if x_2!=None:
            y=math.tan(math.pi/2 - angle_1)*(x_2-base_1[0]) + base_1[1]   # y of intersection
            u=(math.tan(math.pi/2 - angle_1)*(x_2-base_1[0]) + base_1[1]-range_2[0])/(range_2[1]-range_2[0])   # y of intersection
            z=(x_2-base_1[0])/(l_1*math.cos(math.pi/2 - angle_1))

            if u>=0 and u<=1 and z>=0 and z<=1:
                intersect=1

        elif y_2!=None:
            if abs(math.pi/2 - angle_1)<=0.01:
                if y_2==base_1[1]: intersect=1
                else: intersect=0
            else:
                u=((base_1[0]-range_2[0]) + (y_2-base_1[1])/(math.tan(math.pi/2 - angle_1)))/(range_2[1]-range_2[0])
                z=(y_2-base_1[1])/(l_1*math.sin(math.pi/2 - angle_1))
                if u>=0 and u<=1 and z>=0 and z<=1:
                    intersect=1


        # if intersect:
        #     print "Intersection: " , (base_1, angle_1, l_1, range_2, x_2 , y_2), intersect
            #pdb.set_trace()
        return intersect



#################

    def compute_pos(self, joints=[]):
        base=(0,0)
        pos=[]
        if joints==[]: joints=[j.angle for j in self.joints]

        for j in range(len(joints)):
            if j==0:
                pos.append( (self.arm_comps[j].length*math.sin(joints[j]), self.arm_comps[j].length*math.cos(joints[j]))  )
            else:
                a=sum([joints[h] for h in range(j+1)])
                #pos.append(   (pos[j-1][0]+self.arm_comps[j].length*math.sin(self.joints[j].angle), pos[j-1][1]+self.arm_comps[j].length*math.cos(self.joints[j].angle))  )
                pos.append(   (pos[j-1][0]+self.arm_comps[j].length*math.sin(a), pos[j-1][1]+self.arm_comps[j].length*math.cos(a))  )

        return pos


#################


    def display_robot(self, dur=0.1):
        if self.display==False: return
        base=(0,0)
        pos=self.compute_pos()

        pos.insert(0,base)

        #plt.cla()
        l=len(self.comp_plot)
        if(l!=0):
            for i in range(l):
                #pdb.set_trace()
                #cp.set_visible(False)
                cp=self.comp_plot[0]
                cp.remove()
                self.comp_plot.pop(0)


        for i,j in enumerate(self.joints):
            r=Rectangle(pos[i],  .5, self.arm_comps[i].length)
            # c=Circle((pos[i][0]-0.5,pos[i][1]-0.5),1,color='k')
            t = mpl.transforms.Affine2D().rotate_around(pos[i][0],pos[i][1],-1*sum([self.joints[h].angle for h in range(i+1) ]) ) + self.ax.transData
            r.set_transform(t)
            self.ax.add_patch(r)
            self.comp_plot.append(r)
            # ax.add_patch(c)

        plt.show()
        plt.pause(dur)


#################



    def display_robot_copy(self, joints,  dur=0.1 , colour=None):
        #import pdb; pdb.set_trace()


        if colour is None:
            colour=(1,0,0,0.5)
        # else:
            # import pdb; pdb.set_trace()
        l=len(self.comp_plot_copy)
        if(l!=0):
            for i in range(l):
                cp=self.comp_plot_copy[0]
                cp.remove()
                self.comp_plot_copy.pop(0)

        pos=self.compute_pos(joints)
        base=(0,0)
        pos.insert(0,base)

        for i,j in enumerate(joints):
            r=Rectangle(pos[i],  .5, self.arm_comps[i].length, color=colour)
            t = mpl.transforms.Affine2D().rotate_around(pos[i][0],pos[i][1],-1*sum([joints[h] for h in range(i+1) ]) ) + self.ax.transData
            r.set_transform(t)
            self.ax.add_patch(r)
            self.comp_plot_copy.append(r)

        plt.show()
        plt.pause(dur)




################################################################################


class arm_comp(object):
    """docstring for arm_comp."""
    def __init__(self, length):
        super(arm_comp, self).__init__()
        self.length=length




################################################################################


class joint(object):
    """docstring for joint."""
    def __init__(self, range):
        super(joint, self).__init__()
        self.range = range

        self.angle=0    ## default


#################


    def rotate(self,theta):
        limit_reached=0
        if self.angle+theta <= self.range and self.angle+theta >= -1*self.range:
            self.angle += theta
        elif self.angle+theta > self.range:
            self.angle = self.range
            limit_reached=1
            # print "limit reached!", self.angle
        elif self.angle + theta < -1*self.range:
            self.angle = -1* self.range
            limit_reached=1
            # print "limit reached!", self.angle
        return limit_reached


################################################################################


class environment(object):
    def __init__(self, objects):
        super(environment, self).__init__()
        self.objects=objects
        self.display=True

        if self.display==True:
            #plt.ion()
            fig,self.ax = plt.subplots(1)
            self.ax.set_xlim([-30,30])
            self.ax.set_ylim([0,30])

#################

    def display_environment(self):
        for o in self.objects:
            r=Rectangle((o[0],o[1]), o[2], o[3] , color='grey' )
            self.ax.add_patch(r)

        #plt.show()
        #plt.pause(2)




################################################################################


if __name__ == "__main__":

    E=environment([(12,6,3,4),(-27,4,11,6),(2,12,24,2)])    # x,y,w,h
    E.display_environment()

    #R=robot(3,[10,8,6],[math.pi/2,math.pi/1.5,math.pi/2],environment=E, display=False)

    import random



    for h in range(1000):
        R=robot(3,[10,8,6],[math.pi/2,math.pi/1.5,math.pi/2],environment=E, display=False)
        r=random.randint(10,30)
        for i in range(r):
            j=random.randint(0,2)
            a=random.randint(2,10)
            b=(2*random.randint(0,1))-1
            R.rotate_joint(j,b*math.pi/a)

        R.display_robot(2)

        '''save R.path'''
        with open('paths/path_%d.dat'%h,'w') as f:
            f.write('#a1_X a1_Y a2_X a2_Y a3_X a3_Y j_1 j_2 j_3\n')
            #print ["%.3f %.3f %.3f %.3f %.3f %.3f"%(p[0][0],p[0][1],p[1][0],p[1][1],p[2][0],p[2][1]) for [p,q] in R.path]
            f.write('\n'.join(["%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f"%(p[0][0],p[0][1],p[1][0],p[1][1],p[2][0],p[2][1],q[0],q[1],q[2]) for [p,q] in R.path]))

        # for j in range(len(R.path)-1):
        #     plt.plot([R.path[j][2][0],R.path[j+1][2][0]] ,[R.path[j][2][1],R.path[j+1][2][1]] , color='b', alpha=(float(j)/len(R.path))**2)
        # plt.show()
