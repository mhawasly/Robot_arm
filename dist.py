def smallest_dist_to_segment(x,y,range_l, X_l=None, Y_l=None ):
    if Y_l is not None:
        w=range_l[1]-range_l[0]
        t= float(x-range_l[0])/w
        if t>=0 and t<=1: d=abs(y-Y_l)
        elif t<0: d=math.sqrt((x-range_l[0])**2+(y-Y_l)**2)

        elif t>1: import pdb; pdb.set_trace();d=math.sqrt((x-range_l[1])**2+(y-Y_l)**2)
    elif X_l is not None:
        h=range_l[1]-range_l[0]
        t= float(y-range_l[0])/h
        if t>=0 and t<=1: d=abs(x-X_l)
        elif t<0: d=math.sqrt((x-X_l)**2+(y-range_l[0])**2)
        elif t>1: d=math.sqrt((x-X_l)**2+(y-range_l[1])**2)


    return d


import math


if __name__ == "__main__":

    import pdb; pdb.set_trace()
    print smallest_dist_to_segment(-5,2, [-1,3], None, -2)
