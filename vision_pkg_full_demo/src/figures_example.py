import math
from grasp_point_derivative_analysis import GraspPointDerivative

GPA=GraspPointDerivative()

#1
fb=[{'c':[-6,-1.1,0.05,0.01,0.0001],'l':[-math.inf,-5.215]},{'c':[1,0.5,0.05],'l':[-5.215,-1.381]},{'c':[1.5,0.8,0.005],'l':[-1.381,8.047]},{'c':[1,0.5,0.05],'l':[8.047,13.637]},{'c':[-6,-1.1,0.05,0.01,0.0001],'l':[13.637,math.inf]}]
ft=[{'c':[15,1.2,-0.05],'l':[-math.inf,0]},{'c':[15,0.4,0.04,0,-0.0001],'l':[0,9.383]},{'c':[21.5],'l':[9.383,math.inf]}]
#GPA.exec(ft,fb,[-10,16],[-12,27],'example1_testsv2.pdf', invert=False, plot_all=True, n_tests=12)
#GPA.exec(ft,fb,[-10,16],[-12,27],'example1_bestv2.pdf', invert=False)
#GPA.exec(ft,fb,[-10,16],[-12,27],'example1_envelopev2.pdf', invert=False)
#2
fb=[{'c':[-4,0.25,0.008],'l':[-math.inf,-12.669]},{'c':[4,0.35,-0.05,0,0.0001],'l':[-12.669,11.305]},{'c':[3.2],'l':[11.305,math.inf]}]
ft=[{'c':[14,0.25,-0.05],'l':[-math.inf,-6.213]},{'c':[13,0.5,0.02,0,-0.0001],'l':[-6.213,-1.32]},{'c':[12.7,0.25,0.002,-0.0005,-0.00001],'l':[-1.32,5.142]},{'c':[14,0.25,-0.05],'l':[5.142,math.inf]}]
#GPA.exec(ft,fb,[-20,19],[-10,17],'example2_testsv2.pdf', invert=False, plot_all=True, n_tests=14)
#GPA.exec(ft,fb,[-20,19],[-10,17],'example2_bestv2.pdf', invert=False)
#GPA.exec(ft,fb,[-20,19],[-10,17],'example2_envelopev2.pdf', invert=False)


##1
fb1=[{'c':[-6,-1.1,0.05,0.01,0.0001],'l':[-math.inf,math.inf]}]
fb2=[{'c':[1,0.5,0.05],'l':[-math.inf,math.inf]}]
fb3=[{'c':[1.5,0.8,0.005],'l':[-math.inf,math.inf]}]
ft1=[{'c':[15,1.2,-0.05],'l':[-math.inf,math.inf]}]
ft2=[{'c':[15,0.4,0.04,0,-0.0001],'l':[-math.inf,math.inf]}]
ft3=[{'c':[21.5],'l':[-math.inf,math.inf]}]
fb=[fb1,fb2,fb3]
ft=[ft1,ft2,ft3]
#GPA.exec(ft,fb,[-10,16],[-12,27],'example1_envelopev2.pdf', invert=False, plot_lines=True)

##2
fb1=[{'c':[-4,0.25,0.008],'l':[-math.inf,math.inf]}]
fb2=[{'c':[4,0.35,-0.05,0,0.0001],'l':[-math.inf,11.305]},{'c':[3.2],'l':[11.305,math.inf]}]
ft1=[{'c':[14,0.25,-0.05],'l':[-math.inf,math.inf]}]
ft2=[{'c':[13,0.5,0.02,0,-0.0001],'l':[-math.inf,math.inf]}]
ft3=[{'c':[12.7,0.25,0.002,-0.0005,-0.00001],'l':[-math.inf,math.inf]}]
fb=[fb1,fb2]
ft=[ft1,ft2,ft3]
GPA.exec(ft,fb,[-20,19],[-10,17],'example2_envelopev2.pdf', invert=False, plot_lines=True)