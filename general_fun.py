
import numpy as np



class table_cal:
    
    def line_equation(self, point1, point2):
        # Calculate the direction vector
        V = (point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2])
    
        # Choose one of the points as the known point
        P = point1
    
        # Prepare the equation as a string
        equation = f"r = ({P[0]}, {P[1]}, {P[2]}) + t * ({V[0]}, {V[1]}, {V[2]})"
    
        return P, V
    
        
    def corner_points(self, trs, id_default):      # id_default is the right side id on the table
        # only suitable for there are 2 aruco markers
        p1 = trs[id_default][0]
        
        for id in trs.keys():
            if id !=id_default:
                p2 = trs[id][0]
        
        if id_default == 50:
            
            pl = [p2[0] + 0.025, p2[1] - 0.025, p2[2]]
            pr = [p1[0] - 0.025, p1[1] - 0.025, p1[2]]
        elif id_default == 140:
            pl = [p2[0] + 0.025, p2[1] + 0.025, p2[2]]
            pr = [p1[0] - 0.025, p1[1] + 0.025, p1[2]]

        else:
            print("id default error, please check again")

        return pl, pr
    
    
    def calculate_junction_point(self, line1_equation, line2_equation):
        # Extract the point and direction vector from each line equation
        P1, V1 = line1_equation
        P2, V2 = line2_equation
    
        # Solve for the parameter values (t1 and t2) that satisfy the equation P1 + t1 * V1 = P2 + t2 * V2
        t1 = (P2[0] - P1[0]) / V1[0]
        t2 = (P2[1] - P1[1]) / V1[1]
    
        # Calculate the junction point by substituting the parameter values into one of the line equations
        junction_point = (P1[0] + t1 * V1[0], P1[1] + t1 * V1[1], P1[2] + t1 * V1[2])
    
        return junction_point
    

    def calculate_distance(self, point1, point2):
    # Calculate the Euclidean distance between two points in 3D space
        distance = np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2 + (point2[2] - point1[2])**2)
        return distance
    
    def calculate_center(self, points):
        # Calculate the center point of a list of points in 3D space
        center = [sum(coord) / len(points) for coord in zip(*points)]
        return center