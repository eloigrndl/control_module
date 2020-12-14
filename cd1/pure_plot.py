"""
speed = 0
angular_speed = 0
position = [0,0]
delta_time = 0.5
acceleration = 0.0
if speed == 0:
    car_curvature = 0
else:
    car_curvature = angular_speed/speed

(angular_speed, torque) = test1.calculate_next_state(speed, position, car_curvature)
i = 0
while test1.calc_distance(points[len(points) - 1], position) >= 0.1:
    print("\nITERATION :", i)
    print((angular_speed, torque, car_curvature))
    position[0] = position[0] + speed * delta_time + 1/2 * torque * (delta_time ** 2)
    speed = speed + 1/2 * (acceleration + torque) * delta_time 
    acceleration = torque

    (angular_speed, torque) = test1.calculate_next_state(speed, position, car_curvature)

    if speed == 0:
        car_curvature = 0
    else:
        car_curvature = angular_speed/speed
    i +=1
    print(position)

"""