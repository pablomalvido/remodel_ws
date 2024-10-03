import math
from grasp_point_derivative_analysis import GraspPointDerivative

class GraspPointCalculation():

    def __init__(self):
        self.gpd = GraspPointDerivative()

    def create_polynomial_intervals(self, poly_list, intervals):
        print(poly_list)
        print(intervals)
        p_result = []
        for i in intervals: #for each interval [index, min, max]
            for p_i in poly_list[i[0]]: #for each polynomial of the selected line for this interval
                if p_i['l'][0] <= i[1]: #p1l<=il
                    print(i)
                    if p_i['l'][1] <= i[1]: #p1h<=il
                        #Completely out in next
                        continue
                    elif p_i['l'][1] < i[2]:#p1h<ih
                        #Partially in - partially in next
                        p_new = {'c':p_i['c'], 'l':[i[1], p_i['l'][1]]}#[il,p1h]
                        pass
                    else:
                        #Completely in
                        p_new = {'c':p_i['c'], 'l':[i[1], i[2]]}#[il,ih]
                        pass
                else:
                    if p_i['l'][0] < i[2]:#p1l<ih
                        #Partially in - partially in prev
                        p_new = {'c':p_i['c'], 'l':[p_i['l'][0], i[2]]}#[p1l,ih]
                        pass
                    else:
                        #Completely out in prev
                        continue
                p_result.append(p_new)
        p_result[0]['l'][0] = -math.inf
        p_result[-1]['l'][1] = math.inf
        return p_result

    def exec(self, all_cables_poly, all_cables_input, index_upper, dist_per_pixel, img, analyzed_grasp_length, plot_name="", res_height=0):
        """
        all_cables_input: list with the index of the cable and its points
        indes_upper: index of the lower upper cable (index 0 is the cable in the bottom)
        """
        print(all_cables_poly)
        upper_cables_all = []
        lower_cables_all = []
        upper_cables_poly_all = []
        lower_cables_poly_all = []
        #i=0
        for index in all_cables_input:
            if index >= index_upper:
                upper_cables_all.append(all_cables_input[index])
                upper_cables_poly_all.append(all_cables_poly[index])
            else:
                lower_cables_all.append(all_cables_input[index])
                lower_cables_poly_all.append(all_cables_poly[index])
            #i+=1

        all_cables = []
        upper_cables = []
        upper_cables_poly = []
        n_top = len(upper_cables_all)
        n_top_wrong = 0
        n_low = len(lower_cables_all)
        n_low_wrong = 0
        wrong = False
        i=0
        for cable in upper_cables_all: #If a cable doesn't reach the limit complete it with an horizontal value from that point
            if len(cable)>0:
                upper_cables.append(cable)
                upper_cables_poly.append(upper_cables_poly_all[i])
                all_cables.append(cable)
            else:
                n_top_wrong += 1
            i+=1

        lower_cables = []
        lower_cables_poly = []
        i=0
        for cable in lower_cables_all:
            if len(cable)>0:
                lower_cables.append(cable)
                lower_cables_poly.append(lower_cables_poly_all[i])
                all_cables.append(cable)
            else:
                n_low_wrong += 1
            i+=1
    
        xmax = []
        xmin = []

        for cable in all_cables:
            xmax.append(max(p[1] for p in cable))
            xmin.append(min(p[1] for p in cable))
        x_limits = [max(xmin), max(xmax)] #We pick max xmax so we have to fill the cables that don't reach that xmax value

        if min(xmin) > (5/dist_per_pixel) and max(xmax) < img.shape[0] * (3/4):
            wrong = True

        #Complete the cables if there are no more points
        upper_cables_completed = []
        i=0
        for cable in upper_cables:
            if cable[-1][1] < x_limits[1]:
                if cable[-1][1] < img.shape[0]/2:
                    n_top_wrong += 1
                for new_x in range(cable[-1][1]+1, x_limits[1]+1):
                    cable.append([cable[-1][0], new_x]) #Keep the last value till the end
                upper_cables_poly[i].append({'c':[cable[-1][0]],'l':[upper_cables_poly[i][0]['l'][1], x_limits[1]]}) #Adds a new interval for the polynomial, this will be a horizontal line of the value of the last f(x)        
            upper_cables_completed.append(cable)
            i+=1

        lower_cables_completed = []
        i=0
        for cable in lower_cables:
            if cable[-1][1] < x_limits[1]:
                if cable[-1][1] < img.shape[0]/2:
                    n_low_wrong += 1
                for new_x in range(cable[-1][1]+1, x_limits[1]+1):
                    cable.append([cable[-1][0], new_x]) #Keep the last value till the end
                lower_cables_poly[i].append({'c':[cable[-1][0]],'l':[lower_cables_poly[i][0]['l'][1], x_limits[1]]}) #Adds a new interval for the polynomial, this will be a horizontal line of the value of the last f(x)
            lower_cables_completed.append(cable)
            i+=1

        if n_top_wrong > n_top/2 or n_low_wrong > n_low/2:
            pass
            #wrong = True
            #print(n_top_wrong)
            #print(n_low_wrong)

        cables_up_dict = []
        for cable_up in upper_cables_completed:
            dict_temp = {}
            for point_i in cable_up:
                #print(point_i)
                dict_temp[point_i[1]] = point_i[0]
            cables_up_dict.append(dict_temp)
        
        cables_down_dict = []
        for cable_down in lower_cables_completed:
            dict_temp = {}
            for point_i in cable_down:
                dict_temp[point_i[1]] = point_i[0]
            cables_down_dict.append(dict_temp)

        lower_up_cable_dict = {}
        upper_down_cable_dict = {}
        min_dist_cable_dict = {}
        finger_thickness = 14.0
        #for x in range(max(x_limits[0], int(((finger_thickness*0.75)/dist_per_pixel))), min(x_limits[1]+1, int((analyzed_grasp_length/dist_per_pixel))+1), 1): #Do it in all the length, not in these limits
        min_x_eval = x_limits[0] + int(((finger_thickness*0.75)+9)/dist_per_pixel)
        max_x_eval = min(x_limits[1]+1, int((analyzed_grasp_length/dist_per_pixel))+1)

        #Initialize intervals
        x0lower_up = -math.inf #The image axis is upside down
        for i in range(len(upper_cables_poly)):
            if cables_up_dict[i][min_x_eval] > x0lower_up:
                x0lower_up = cables_up_dict[i][min_x_eval]
                lower_up_index = i
        lower_up_cables_intervals = [[lower_up_index, min_x_eval]] #[index, min, (max)]

        x0upper_down = math.inf
        for i in range(len(lower_cables_poly)):
            if cables_down_dict[i][min_x_eval] < x0upper_down:
                x0upper_down = cables_down_dict[i][min_x_eval]
                upper_down_index = i
        upper_down_cables_intervals = [[upper_down_index, min_x_eval]] #[index, min, (max)]

        y_min = math.inf #y goes down
        y_max = -math.inf
        for x in range(min_x_eval, max_x_eval, 1): #Do it in all the length, not in these limits. Prev +5
            #print(x)
            lower_up_x = -math.inf #y axis goes down
            i=0
            for cable_up_dict_i in cables_up_dict:
                if cable_up_dict_i[x] < y_min:
                    y_min = cable_up_dict_i[x]
                if cable_up_dict_i[x] > lower_up_x:
                    lower_up_x = cable_up_dict_i[x]
                    lower_up_index = i
                i+=1
            lower_up_cable_dict[x] = lower_up_x
            if lower_up_index != lower_up_cables_intervals[-1][0]:
                lower_up_cables_intervals[-1].append(x)
                lower_up_cables_intervals.append([lower_up_index, x])

            upper_down_x = math.inf
            i=0
            for cable_down_dict_i in cables_down_dict:
                if cable_down_dict_i[x] > y_max:
                    y_max = cable_down_dict_i[x]
                if cable_down_dict_i[x] < upper_down_x:
                    upper_down_x = cable_down_dict_i[x]
                    upper_down_index = i
                i+=1
            upper_down_cable_dict[x] = upper_down_x
            if upper_down_index != upper_down_cables_intervals[-1][0]:
                upper_down_cables_intervals[-1].append(x)
                upper_down_cables_intervals.append([upper_down_index, x])

            min_dist_cable_dict[x] = upper_down_cable_dict[x] - lower_up_cable_dict[x]

        lower_up_cables_intervals[-1].append(max_x_eval)
        upper_down_cables_intervals[-1].append(max_x_eval)
        upper_cable_separation_poly = self.create_polynomial_intervals(upper_cables_poly, lower_up_cables_intervals)
        lower_cable_separation_poly = self.create_polynomial_intervals(lower_cables_poly, upper_down_cables_intervals)

        """
        best_x = max(min_dist_cable_dict, key=min_dist_cable_dict.get)
        best_y = int((upper_down_cable_dict[best_x] + lower_up_cable_dict[best_x])/2)

        best_x_prev = max(min(min_dist_cable_dict), best_x - int(3/dist_per_pixel))
        best_y_prev = int((upper_down_cable_dict[best_x_prev] + lower_up_cable_dict[best_x_prev])/2)
        best_x_next = min(max(min_dist_cable_dict), best_x + int(3/dist_per_pixel))
        best_y_next = int((upper_down_cable_dict[best_x_next] + lower_up_cable_dict[best_x_next])/2)
        best_theta = math.atan2(float(best_y_next-best_y_prev),float(best_x_next-best_x_prev))

        if min_dist_cable_dict[best_x] > 0 and not wrong:
            success_grasp = True
        else:
            print("Minimum distance: " + str(min_dist_cable_dict[best_x]))
            success_grasp = False

        return [best_x, best_y, best_theta], success_grasp
        """
        y_tot = y_max - y_min
        best_point, best_angle = self.gpd.exec(lower_cable_separation_poly, upper_cable_separation_poly, [min_x_eval, max_x_eval], [y_min, y_max], plot_name = plot_name, res_height=res_height)
        if min_dist_cable_dict[max(int(best_point[0]),min(min_dist_cable_dict))] > 0 and not wrong:
            success_grasp = True
        else:
            print("Minimum distance: " + str(min_dist_cable_dict[max(int(best_point[0]),min(min_dist_cable_dict))]))
            success_grasp = False

        return [int(best_point[0]),int(best_point[1]),best_angle], success_grasp