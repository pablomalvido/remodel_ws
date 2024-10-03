import numpy as np
from scipy.optimize import minimize
import math
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import os

class GraspPointDerivative():

    def __init__(self):
        pass

    # Define the polynomials
    def polynomial(self, p, x):
        res=0
        for i in range(len(p)):
            res+=(p[i])*(x**i)
        return res

    def get_derivative(self, f_list):
        d_list = []
        for i in range(1,len(f_list)):
            d_list.append(i*f_list[i])
        return d_list

    def polynomial_dict(self, p, x):
        res=0
        #print(p)
        for f in p:
            #print(f)
            if x >= f['l'][0] and x<=f['l'][1]:
                pc=f['c']
        for i in range(len(pc)):
            res+=(pc[i])*(x**i)
        return res

    def get_derivative_dict(self, f_list):
        d_list = []
        for f in f_list:
            d_list.append({'l':f['l']})
            d_f=[]
            for i in range(1,len(f['c'])):
                d_f.append(i*f['c'][i])
            d_list[-1]['c'] = d_f
        return d_list

    """
    f1 = [-2, -1, 3] #3x^2 - x - 2
    d1 = get_derivative(f1) #6x - 1
    print(d1)
    """

    # Function to plot the polynomials and the circle
    def plot_FunctionAndDerivative(self, f1, x_interval):
        x_values = np.linspace(x_interval[0], x_interval[1], 100)
        y1_values=[]
        y2_values=[]
        d1=self.get_derivative_dict(f1)
        for x in x_values:
            y1_values.append(self.polynomial_dict(f1,x))
            y2_values.append(self.polynomial_dict(d1,x))

        plt.plot(x_values, y1_values, label='P1(x)')
        plt.plot(x_values, y2_values, label='D1(x)')

        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.title('Curve and derivative')
        plt.grid(True)
        plt.show()


    def plot_FunctionsAndAVGDerivate(self, f1, f2, x_interval):
        x_values = np.linspace(x_interval[0], x_interval[1], 100)
        y1_values=[]
        y2_values=[]
        yd_values=[]
        yd_f=[]
        d1=self.get_derivative_dict(f1)
        d2=self.get_derivative_dict(f2)
        for x in x_values:
            y1_values.append(self.polynomial_dict(f1,x))
            y2_values.append(self.polynomial_dict(f2,x))
            avg_d = [0, (self.polynomial_dict(d1,x)+self.polynomial_dict(d2,x))/2]
            yd_f.append(self.polynomial_dict(avg_d,x))
            yd_values.append((self.polynomial_dict(d1,x)+self.polynomial_dict(d2,x))/2)

        plt.plot(x_values, y1_values, label='P1(x)')
        plt.plot(x_values, y2_values, label='P2(x)')
        plt.plot(x_values, yd_values, label='D(x)')
        plt.plot(x_values, yd_f, label='D2(x)')

        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.title('Curve and derivative')
        plt.grid(True)
        plt.show()


    def plot_results(self, lines, y_limits, points=[], plot_name="plot.jpg"):
        i=0
        for line in lines:
            i+=1
            plt.plot(line[0], line[1], label='L'+str(i))
        for p in points:
            plt.plot(p[0], p[1], "ro")
        plt.xlabel('x')
        plt.ylabel('y')
        plt.ylim(y_limits)
        plt.legend()
        plt.title('Functions')
        plt.grid(True)
        plt.savefig(os.path.dirname(os.path.realpath(__file__)) + "/../plots/" + str(plot_name))
        #plt.show()
        plt.clf()

    
    def plot_results_inverted(self, lines, y_limits, points=[], plot_name="plot.jpg"):
        j=0
        for line in lines:
            j+=1
            inverted_line = [i * -1 for i in line[1]]
            #plt.plot(line[0], inverted_line, label='L'+str(j))
            if j<3:
                plt.plot(line[0], inverted_line)
            elif j==3:
                plt.plot(line[0], inverted_line, color="green")
            else:
                plt.plot(line[0], inverted_line, color="gray")
        j=0
        for p in points:
            if j==0:
                plt.plot(p[0], -p[1], "ro")
            else:
                plt.plot(p[0], -p[1], "bo")
            j+=1
        plt.xlabel('x')
        plt.ylabel('y')
        plt.ylim([-y_limits[1],-y_limits[0]])
        plt.legend()
        plt.title('Functions')
        plt.grid(True)
        plt.savefig(os.path.dirname(os.path.realpath(__file__)) + "/../plots/" + str(plot_name))
        #plt.show()
        plt.clf()


    def intersection_with_polynomial(self, point, direction, polynomial_coeff, bounds):
        x0, y0 = point
        if abs(self.polynomial_dict(polynomial_coeff, x0) - y0) <= 1e-5:
            return [point]
        dx, dy = direction
        if dx==0:
            return [[x0, self.polynomial_dict(polynomial_coeff, x0)]]
        else:
            t_diff = bounds[1] - bounds[0]
            t_min=-t_diff*2.5
            t_max=t_diff*2.5
    
        # Define the function to find roots of
        def equation(t):
            x = x0 + t * dx
            y = y0 + t * dy
            return self.polynomial_dict(polynomial_coeff, x) - y
        
        # Find all roots of the equation
        t_initial_guesses = np.linspace(t_min, t_max, 100)  # Initial guesses for t
        t_roots = []
        
        for t_initial in t_initial_guesses:
            t_root, info, ier, mesg = fsolve(equation, t_initial, full_output=True)
            if ier == 1 and np.isclose(equation(t_root), 0, atol=1e-5) and t_min <= t_root <= t_max:  # Check if the solution converged
                if not any(np.isclose(t_root, root, atol=1e-5) for root in t_roots):  # Check for uniqueness
                    t_roots.append(t_root[0])
        
        # Calculate intersection points
        intersection_points = [[x0 + t * dx, y0 + t * dy] for t in t_roots]
        return intersection_points

    def get_min_intersection(self, point, intersections):
        min=math.inf
        min_int_point = []
        for i in intersections:
            dist=self.euclidean_distance(i, point)
            if dist<min:
                min=dist
                min_int_point = i
        return min, min_int_point

    def euclidean_distance(self, point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


    def exec(self, f_up, f_down, x_interval, y_limits, n=15, plot_name="plot.jpg"):
        print("##############")
        print("F UP:")
        print(f_up)
        print("F DOWN:")
        print(f_down)
        print("X INTERVAL:")
        print(x_interval)
        print("Y LIMITS:")
        print(y_limits)
        ylines=[]
        x_values = np.linspace(x_interval[0], x_interval[1], 100)

        d1=self.get_derivative_dict(f_up)
        d2=self.get_derivative_dict(f_down)
        y1_values=[]
        y2_values=[]
        y_avg=[]
        avg_d_slope_values=[]
        #d_avg=[]
        for x in x_values:
            y1_values.append(self.polynomial_dict(f_up,x))
            y2_values.append(self.polynomial_dict(f_down,x))
            y_avg.append((y1_values[-1]+y2_values[-1])/2)
            #avg_d = [0, (polynomial_dict(d1,x)+polynomial_dict(d2,x))/2]
            #d_avg.append(polynomial_dict(avg_d,x))
            avg_d_slope_values.append((self.polynomial_dict(d1,x)+self.polynomial_dict(d2,x))/2)

        #plot_results(x_values, [y1_values, y2_values], y_limits)


        #"""
        max_dist = 0
        max_point = []
        max_line = []
        all_lines = []
        all_points = []
        if n == 0:
            n = len(x_values)
        for n_i in range(n):
            i = int((n_i * len(x_values))/n)
            print(n_i)
            print(i)
            print(y1_values[i])
            print(y2_values[i])
            print("###")
            if y1_values[i] > y2_values[i]:
                dir_perp = avg_d_slope_values[i]
                if abs(dir_perp) < 0.01:
                    dir_perp_unit = [1,0]
                    dir = [0,-1]
                    m = 100
                else:
                    mod = math.sqrt(1+dir_perp**2)
                    dir_perp_unit = [1/mod, dir_perp/mod]
                    dir = [dir_perp_unit[1], -dir_perp_unit[0]]
                    m = dir[1]/dir[0] #dy/dx
                n = y_avg[i]-x_values[i]*m    
                intersections1 = self.intersection_with_polynomial(point=[x_values[i], y_avg[i]], direction=dir, polynomial_coeff=f_up, bounds=x_interval)
                min1, min_int1 = self.get_min_intersection(point=[x_values[i], y_avg[i]], intersections=intersections1)
                intersections2 = self.intersection_with_polynomial(point=[x_values[i], y_avg[i]], direction=dir, polynomial_coeff=f_down, bounds=x_interval)
                min2, min_int2 = self.get_min_intersection(point=[x_values[i], y_avg[i]], intersections=intersections2)
                dist=self.euclidean_distance(min_int1,min_int2)
                all_points.append([(min_int1[0]+min_int2[0])/2,(min_int1[1]+min_int2[1])/2])
                all_lines.append([n,m])
                print(dist)
                if dist>max_dist:
                    print("AA")
                    max_dist=dist
                    max_line=[n,m]
                    max_point=[(min_int1[0]+min_int2[0])/2,(min_int1[1]+min_int2[1])/2]

        #print(max_line)
        points=[max_point]
        print("Position: " + str(max_point))
        angle = ((math.atan2(max_line[1],1))*(180/math.pi))-90
        if angle<-90:
            angle+=180
        print("Angle: " + str(angle))
        #"""
        """
        dir_perp = avg_d_slope_values[int(len(avg_d_slope_values)/2)]
        mod = math.sqrt(1+dir_perp**2)
        dir_perp_unit = [1/mod, dir_perp/mod]
        dir = [dir_perp_unit[1], -dir_perp_unit[0]]
        m = dir[1]/dir[0] #dy/dx
        n = y_avg[int(len(avg_d_slope_values)/2)]-x_values[int(len(avg_d_slope_values)/2)]*m
        print(n)
        print(m)
        max_line = [n,m]
        """
        #"""
        y_max=[]
        for x in x_values:
            y_max.append(self.polynomial(max_line,x)) #No dict, only list

        ylines.append(y1_values)
        ylines.append(y2_values)
        #ylines.append(d_avg)
        #ylines.append(avg_d_slope_values)
        ylines.append(y_max)
        #plot_results(x_values, ylines, y_limits, points, plot_name)

        #Small modification to see all the grasp point direction line when it is in the evaluation interval border
        x_tot = x_interval[1] - x_interval[0]
        x2_values = np.linspace(x_interval[0]-(x_tot*0.1), x_interval[1]+(x_tot*0.1), 120)
        y2_max=[]
        for x in x2_values:
            y2_max.append(self.polynomial(max_line,x)) #No dict, only list
        lines = [[x_values, y1_values], [x_values, y2_values], [x2_values, y2_max]]

        #PRINT ALL OTHER LINES
        y2_all=[]
        for line in all_lines:
            if line != max_line:
                y2_all.append([])
                for x in x2_values:
                    y2_all[-1].append(self.polynomial(line,x))
        for y_line_i in y2_all:
            lines.append([x2_values, y_line_i])
        for p in all_points:
            if not(p in points):
                points.append(p)

        self.plot_results_inverted(lines, y_limits, points, plot_name)
        #"""
        return max_point, angle

#"""
# Example usage
x_interval = (-5,5)#(-2,2)#(1, 4)
y_limits = (-1,6)#(10,-10)
f1 = [{'c':[3,0.25],'l':[-math.inf,0]},{'c':[3,-0.1],'l':[0,math.inf]}]#[5]#[-2, 10, -1] ######ABOVE
f2 = [{'c':[0,0,0.25],'l':[-math.inf,math.inf]}]#[0,0,1]#[0,0,1] ######BELOW
#function_best_point_polynomials(f1, f2, x_interval, y_limits)
gpd = GraspPointDerivative()
best_point, best_angle = gpd.exec(f1, f2, x_interval, y_limits, plot_name = 'test_new.jpg')

#"""