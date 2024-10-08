"""
Plotting tools for Sampling-based algorithms
@author: huiming zhou
"""
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os
import sys
import env

matplotlib.use('Agg')

#sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                "/../../Sampling_based_Planning/")

#from Sampling_based_Planning.rrt_2D import env


class Plotting:
    def __init__(self, collisions, x_start, x_goal):
        self.xI, self.xG = x_start, x_goal
        self.env = collisions
        self.obs_bound = self.env.obs_boundary
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_rectangle_colli = self.env.obs_rectangle_collision
        self.obs_circle_colli = self.env.obs_circle_collision
        #print(self.obs_circle_colli)

    def animation(self, nodelist, path, name, animation=False):
        self.plot_grid(name)
        self.plot_visited(nodelist, animation)
        self.plot_path(path)

    def animation_connect(self, V1, V2, xI_EE, xG_EE, path, path_EE, name):
        self.plot_grid(name)
        #self.plot_visited_connect(V1, V2)
        self.plot_path(path, self.xI, self.xG)
        self.plot_path_EE(path_EE, xI_EE, xG_EE, name)

    def plot_grid(self, name):
        #plt.axes(xlim=(-50,50),ylim=(45,95))
        fig, ax = plt.subplots()

        for (ox, oy, w, h) in self.obs_bound:
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h, a, type) in self.obs_rectangle:
            """
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='orange',
                    fill=True
                )
            )
            """
            hatch_type = ""
            color = "gray"
            label = "Obstacle"
            if type == "disabled":
                hatch_type = "////"
                color = "lightgray"
                label = "Disabled obstacle"
            elif type == "target":
                #hatch_type = "///"
                #color = "#D2691E"
                color = "#FF9933"
                label = "Target fixture"
            ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    angle=a,
                    edgecolor='black',
                    facecolor=color,
                    fill=True,
                    hatch=hatch_type, 
                    label=label
                )
            )
        """
        for (ox, oy, w, h) in self.obs_rectangle_colli:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), 0.5,
                    edgecolor='black',
                    facecolor='pink',
                    fill=True
                )
            )
        """
        for (ox, oy, r) in self.obs_circle:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='red',
                    fill=True
                )
            )
        """
        for (ox, oy, r) in self.obs_circle_colli:
            ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='red',
                    fill=True
                )
            )
        """
        #plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3) #Start
        #plt.plot(self.xG[0], self.xG[1], "g*", linewidth=3) #Goal

        plt.title(name)
        plt.axis("equal")
        plt.axis(adjustable="datalim")
        #plt.axis(xbound=(-50,50),ybound=(45,95))
        plt.xlim(-60,60)
        plt.ylim(40,100)

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:
            for node in nodelist:
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    @staticmethod
    def plot_path(path, xI, xG):
        if len(path) != 0:
            #if False:
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2, label="Connector path")
            #plt.pause(0.01)
        #plt.show()
        plt.plot(xI[0], xI[1], color="b", marker="s", linewidth=0, markersize=4) #Start
        plt.plot(xG[0], xG[1], color="#008800", marker="s", linewidth=0, markersize=4) #Goal

    @staticmethod
    def plot_path_EE(path, xI, xG, title="path"):
        if len(path) != 0:
            #if False:
            plt.plot([x[0] for x in path], [x[1] for x in path], '--k', linewidth=2, label="EE path")
            plt.pause(0.01)

        plt.plot(xI[0], xI[1], color="b", marker="s", linewidth=0, label="Start point", markersize=4) #Start
        plt.plot(xG[0], xG[1], color="#008800", marker="s", linewidth=0, label="Target point", markersize=4) #Goal

        plt.legend(loc=2, fontsize=8)
        plt.tick_params(labelsize=10)
        #plt.show()
        plt.savefig(os.path.dirname(os.path.realpath(__file__)) + "/../plots/" + str(title) + ".pdf")
        plt.close()
