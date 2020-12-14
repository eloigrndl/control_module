
import numpy as np
import math as mt


class Pure_model:

    HORIZON_N = 40
    SLOW_FACTOR = 3
    MAX_VELOCITY = 5 #unit is m/s
    MAX_ACCELERATION = 8 #unit is m/(s^2)
    LAST_CLOSEST_POINT_INDEX = 0
    LAST_LOOKAHEAD_POINT_INDEX = 0
    LOOK_AHEAD_DISTANCE = 0.07 #in report said that it should be between 12 and 25 but it is for small robots that are allowed to turn fast and
    STEERING_RATIO = 15; #average steering ration for a car
    WHEEL_BASE = 1 #in meters
    FEEDFORWARD_ACC_CONST = 1/MAX_VELOCITY
    FEEDFORWARD_VEL_CONST = 0.002
    FEEDBACK_CONST = 0.01

    path = np.empty((HORIZON_N, 2))
    curvatures = np.empty((HORIZON_N, 2))
    velocities = np.empty((HORIZON_N, 2))

    def acquiring_path(self, points):
        self.path = points
        self.curvatures = self.acquiring_curvature(points)
        self.velocities = self.acquiring_velocities(self.curvatures)
        self.LAST_CLOSEST_POINT_INDEX = 0
        self.LAST_LOOKAHEAD_POINT_INDEX = 0

    def acquiring_curvature(self, points):
        """Computes the curvature for each point of the path to follow

        Args:
            points (array of pair): array of points representing the path to follow

        Returns:
            1D-arrray: the array of computed curvatures
        """

        curvatures = np.empty((len(points),))
        curvatures[0] = 0.0
        for i in range(1, len(points) - 1): 
            current = points[i]
            pred = points[i - 1]
            next = points[i + 1]

            if current[0] == pred[0]:
                current[0] += 0.001

            k1 = 0.5 * ((current[0] * current[0]) + (current[1] * current[1]) - ((pred[0] * pred[0]) + (pred[1] * pred[1]))) / (current[0] - pred[0])
            k2 = (current[1] - pred[1]) / (current[0] - pred[0])
            if next[0] * k2 - next[1] + pred[1] - pred[0] * k2 == 0:
                r = mt.nan
            else:
                b = 0.5 * ((pred[0] * pred[0]) - (2 * pred[0] * k1) + (pred[1] * pred[1])  - (next[0] * next[0]) + 2 * next[0] * k1 - (next[1] * next[1]))/(next[0] * k2 - next[1] + pred[1] - pred[0] * k2)
                a = k1 - (k2 * b)
                r = mt.sqrt((current[0] - a) * (current[0] - a) + (current[1] - b * current[1] - b))

            to_push = 1/r
            if mt.isnan(to_push):
                to_push = 0
            curvatures[i] = to_push

        curvatures[len(points) - 1] = 0.0
        return curvatures


    def acquiring_velocities(self, curvatures) :
        """Initialize velocity for each point of the path to follow

        Args:
            curvatures (1D-array): array of curvature at each point of the path

        Returns:
            1D-array: the velocities at each point
        """

        velocities = np.empty((len(curvatures)))
        for i in range(len(curvatures)):
            if i == len(curvatures) - 1 or curvatures[i] == 0.0:
                velocities[i] = self.MAX_VELOCITY
            else :
                velocities[i] = min(self.SLOW_FACTOR/curvatures[i], self.MAX_VELOCITY)

        for i in range(len(velocities) - 2, 0, -1):
            distance = self.calc_distance(self.path[i+1], self.path[i])
            new_velocity = mt.sqrt(velocities[i + 1] * velocities[i + 1] + 2 * self.MAX_ACCELERATION * distance)
            velocities[i] = min(velocities[i], new_velocity)
            
        return velocities


    def calc_distance(self, next, curr):
        """Computes the distance between two points

        Args:
            next (pair): end point of vector
            curr (pair): starting point of vector

        Returns:
            double: the distance between the given points
        """
        diffX = (next[0] - curr[0]) * (next[0] - curr[0])
        diffY = (next[1] - curr[1]) * (next[1] - curr[1])
        return mt.sqrt(diffX + diffY)

    def closest_point(self, points, car):
        """Computes the closest path point to the car

        Args:
            points (array of pair): path to follow
            car (pair): current position of the car

        Returns:
            pair: closest point to the car
        """

        distance = self.calc_distance(points[self.LAST_CLOSEST_POINT_INDEX], car)
        if self.LAST_CLOSEST_POINT_INDEX == len(self.path) - 1:
            self.LAST_CLOSEST_POINT_INDEX = 0
        

        for i in range(self.LAST_CLOSEST_POINT_INDEX, len(points)):
            curr = self.calc_distance(points[i], car)
            if curr < distance:
                distance = curr
                self.LAST_CLOSEST_POINT_INDEX = i
            
        
        return points[self.LAST_CLOSEST_POINT_INDEX]


    def minus_pair(self, u1, u2) :
        """Computes the "direction" vector between 2 points

        Args:
            u1 (pair): starting point
            u2 (pair): end point

        Returns:
            pair: direction vector
        """

        d = [u2[0] - u1[0], u2[1] - u1[1]]
        return d


    def dot_product(self, p1, p2):
        """Performs a dot product between two points

        Args:
            p1 ([type]): first point of product
            p2 ([type]): second point of product

        Returns:
            double: result of ot product
        """

        return p1[0] * p2[0] + p1[1] * p2[1]

    def look_ahead_point(self, car_closest_point):
        """Given the lookahead 

        Args:
            points : path to follow
            car_closest_point ([type]): closest point to the car

        Returns:
            pair: next lookahead point
        """
        if self.LAST_LOOKAHEAD_POINT_INDEX == len(self.path) - 1:
            self.LAST_LOOKAHEAD_POINT_INDEX = 0

        for i in range(self.LAST_LOOKAHEAD_POINT_INDEX, len(self.path) - 1):
            d = self.minus_pair(self.path[i], self.path[i+1])
            f = self.minus_pair(car_closest_point, self.path[i])
            a = self.dot_product(d,d)
            b = 2 * self.dot_product(f,d)
            c = self.dot_product(f,f) - self.LOOK_AHEAD_DISTANCE * self.LOOK_AHEAD_DISTANCE
            discriminant = b * b - 4 * a * c


            if discriminant < 0 :
                return self.path[self.LAST_LOOKAHEAD_POINT_INDEX]
            

            discriminant = mt.sqrt(discriminant)
            t1 = (-b - discriminant) / (2*a)
            t2 = (-b + discriminant) / (2*a)

            if t1 >= 0 and t1 <= 1:
                self.LAST_LOOKAHEAD_POINT_INDEX = mt.ceil(i + t1 * self.LOOK_AHEAD_DISTANCE)
                break
            
            if t2 >= 0 and t2 <= 1:
                self.LAST_LOOKAHEAD_POINT_INDEX = mt.ceil(i + t2 * self.LOOK_AHEAD_DISTANCE)
                break

        return self.path[self.LAST_LOOKAHEAD_POINT_INDEX]

    def getting_correct_curvature(self, car_pos, car_curvature):
        """Given the position and current curvature of the car, computes the new curvature for the car to adopt


        Args:
            points (array of pair): path to follow
            curvatures (1D-array): curvature at each point of the path
            car_pos (pair): car position
            car_curvature (curuvature): curvature of the car

        Returns:
            double: curvature of the car
        """

        #first we want to calculate the curvature of the car
        lookahead_point = self.look_ahead_point(car_pos)
        print("next point to go is", lookahead_point)
        a = -mt.tan(car_curvature)
        b = 1
        c = mt.tan(car_curvature) * car_pos[0] - car_pos[1]
        x = abs(a * lookahead_point[0] + b * lookahead_point[1] + c) / mt.sqrt(a * a + b * b)
        curvature = 2 * x / (self.LOOK_AHEAD_DISTANCE * self.LOOK_AHEAD_DISTANCE)

        #then we have to compute that side (left or right) of this curvature
        side = mt.sin(car_curvature) * (lookahead_point[0] - car_pos[0]) - mt.cos(car_curvature) * (lookahead_point[1] - car_pos[1])
        side = np.sign(side)    
        return side * curvature

    def curvature_to_steering_angle1(self, curvature):
        """First method to convert curvature to steering angle

        Args:
            curvature : double

        Returns:
            double: steering angle
        """

        if curvature == 0.0:
            return 0.0
        else:
            ratio = curvature ** 2
        return self.STEERING_RATIO/ 2 * mt.acos(2 - self.WHEEL_BASE/(ratio))

    def curvature_to_steering_angle2(self, curvature):
        """First method to convert curvature to steering angle

        Args:
            curvature : double

        Returns:
            double: steering angle
        """

        if curvature == 0.0:
            return 0.0
        return mt.asin(self.WHEEL_BASE/curvature) * self.STEERING_RATIO

    def calculate_torque(self, car_speed, expected_speed):
        wanted_acceleration = expected_speed - car_speed
        scaled = wanted_acceleration / self.MAX_ACCELERATION
        return scaled

    def calculate_next_state(self, car_speed, car_position, car_curvature):

        curvature = self.getting_correct_curvature(car_position, car_curvature)
        velocity = self.velocities[self.LAST_LOOKAHEAD_POINT_INDEX]

        #we know that curvture = angular speed / velocity
        angular_speed = curvature * velocity

        steering_angle = self.curvature_to_steering_angle1(curvature)
        torque = self.calculate_torque(car_speed, velocity)

        return [angular_speed, torque]
