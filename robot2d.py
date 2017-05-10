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
        self.pos=[]
        self.comp_plot=[]   #refernce for the plotted arm comps
        self.path=[]


        plt.ion()
        self.ax = self.environment.ax
        #plt.subplots(1)
        # self.ax.set_xlim([-30,30])
        # self.ax.set_ylim([0,30])

        for j in range(num_joints):
            x= joint(joint_ranges[j])
            self.joints.append(x)
            y=arm_comp(arm_comp_lengths[j])
            self.arm_comps.append(y)

#################

    def rotate_joint(self,joint_id, theta):
        step_size=math.pi/12
        #pdb.set_trace()
        steps=abs(theta/step_size)
        sign=steps/(theta/step_size)
        limit_reached=0
        for s in range(1,int(steps)+1):

            #check for joint limit
            limit_reached=self.joints[joint_id].rotate(sign*step_size)
            if limit_reached:
                break

            #check for ground
            pos=self.compute_pos()
            # if pos[-1][1]<0:
            #     #self.joints[joint_id].rotate(-1*sign*step_size)
            #     print "ground", joint_id, self.joints[joint_id].angle
            #     break

            if self.arm_obstructed(): self.joints[joint_id].rotate(-1*sign*step_size); break

            #pdb.set_trace()
            # self.pos=self.compute_pos()
            self.display_robot()

        # if theta != steps*step_size and limit_reached!=1:
        #     self.joints[joint_id].rotate(theta-step_size*steps)
        #     pos=self.compute_pos()
        #     if pos[-1][1]<0:
        #         print "ground"
        #         self.joints[joint_id].rotate(-1*(theta-step_size*steps))
        #
        #     else:
        #         bases=pos
        #         bases.insert(0,(0,0))
        #         #check for obstacles
        #         # for o in self.environment.objects:
        #         #     for i,c in enumerate(self.arm_comps):
        #         #         if self.line_intersect(bases[i], sum(self.joints[a].angle for a in range(i+1)), c.length, (o[1],o[1]+o[3]), o[0])==1 or \
        #         #         self.line_intersect(bases[i], sum(self.joints[a].angle for a in range(i+1)), c.length, (o[1],o[1]+o[3]), o[0]+o[2])==1 or \
        #         #         self.line_intersect(bases[i], sum(self.joints[a].angle for a in range(i+1)), c.length, (o[0],o[0]+o[2]), None, o[1])==1 or \
        #         #         self.line_intersect(bases[i], sum(self.joints[a].angle for a in range(i+1)), c.length, (o[0],o[0]+o[2]), None, o[1]+o[3])==1:
        #         #             print "obstacle"
        #         #             self.joints[joint_id].rotate(-1*sign*step_size)
        #         #             break
        #         if self.arm_obstructed(): self.joints[joint_id].rotate(-1*(theta-step_size*steps))

        self.pos=self.compute_pos()
        # self.display_robot()


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
                    print "obstacle"

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

    def compute_pos(self):
        base=(0,0)
        pos=[]
        for j in range(len(self.joints)):
            if j==0:
                pos.append( (self.arm_comps[j].length*math.sin(self.joints[j].angle), self.arm_comps[j].length*math.cos(self.joints[j].angle))  )
            else:
                a=sum([self.joints[h].angle for h in range(j+1)])
                #pos.append(   (pos[j-1][0]+self.arm_comps[j].length*math.sin(self.joints[j].angle), pos[j-1][1]+self.arm_comps[j].length*math.cos(self.joints[j].angle))  )
                pos.append(   (pos[j-1][0]+self.arm_comps[j].length*math.sin(a), pos[j-1][1]+self.arm_comps[j].length*math.cos(a))  )

        return pos


#################


    def display_robot(self, dur=0.1):
        if ~display: return
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
            print "limit reached!", self.angle
        elif self.angle + theta < -1*self.range:
            self.angle = -1* self.range
            limit_reached=1
            print "limit reached!", self.angle
        return limit_reached


################################################################################


class environment(object):
    def __init__(self, objects):
        super(environment, self).__init__()
        self.objects=objects

        plt.ion()
        fig,self.ax = plt.subplots(1)
        self.ax.set_xlim([-30,30])
        self.ax.set_ylim([0,30])

#################

    def display_environment(self):
        for o in self.objects:
            r=Rectangle((o[0],o[1]), o[2], o[3] , color='grey' )
            self.ax.add_patch(r)

        plt.show()
        plt.pause(0.01)




################################################################################


if __name__ == "__main__":

    E=environment([(12,6,3,4),(-27,4,11,6),(2,12,24,2)])    # x,y,w,h
    E.display_environment()

    R=robot(3,[10,8,6],[math.pi/1,math.pi/1.5,math.pi/2],E)

    import random
    r=random.randint(10,30)
    for i in range(r):
        j=random.randint(0,2)
        a=random.randint(2,10)
        b=(2*random.randint(0,1))-1
        R.rotate_joint(j,b*math.pi/a)
        # R.rotate_joint(1,-math.pi/2)
        # R.rotate_joint(2,-math.pi/2)

    R.display_robot(2)
