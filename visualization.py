import numpy as np
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import math as mt
from EPFL_RT_car_model.mpc_car_model import MPC_car_model
from Loomo_robot_model.mpc_robot_model import MPC_robot_model
import Tools.utils as utils
import time

def display_figure(points, position, predicted_pos, bounds):

    # Clear figure
    plt.clf()

    # Display track, start and end
    x_y = list(zip(*points))

    #Display path as points
    #plt.scatter(x_y[0], x_y[1], zorder=1, label="Track")
    #Display path as line
    plt.plot(x_y[0], x_y[1], zorder=1, label="Track")
    
    plt.scatter(points[0][0], points[0][1],color='red', zorder=2, label="Start/finish point")
    plt.scatter(points[len(points)-1][0], points[len(points)-1][1], color='red', zorder=2)

    # Display the position
    plt.scatter(position[0], position[1], c=10, s=20, zorder=2, label="Current position")

    # Display the predicted position
    x_y = list(zip(*predicted_pos))
    plt.plot(x_y[0], x_y[1], '--r', zorder=2, label="Predicted trajectory")


    # Add legend under the plot
    plt.legend(bbox_to_anchor=(0.5, -0.5), loc='lower center', borderaxespad=0.) 
    plt.figure(1).subplots_adjust(bottom=0.3)

    # Must pause the dispay otherwise the update are too fast for it to be generated
    plt.pause(0.00000001)


def plot_MPC(mpc_module, x0, points, bounds):

    final_pos = (points[len(points) - 1][0], points[len(points) - 1][1])
    current_pos = (x0[0], x0[1])

    #initialize path on MPC module 
    mpc_module.acquire_path(points.copy())

    #initialze records if wanted to be printed at the end of the algo.
    current_state = x0
    states = list(x0)
    u_vectors = [[0, 0]]
    dU_vectors = [[0, 0]]
    distance = mpc_module.calc_distance(final_pos, current_pos)
    mpc_module.set_state(current_state)


    #running it final goal is reached
    while(distance > 0.3):

        # Run the MPC to get next vectors
        new_u, predicted_U = mpc_module.run_MPC()
        head = u_vectors[len(u_vectors) - 1]
        new_dU = [x - y for x,y in zip(new_u, head)]
        
        #Update current values
        current_state = mpc_module.f_next_state(current_state, new_u, new_dU)
        current_pos = (current_state[0], current_state[1])

        mpc_module.set_state(current_state)
        mpc_module.shift_path()

        #Update stop condition
        distance = mpc_module.calc_distance(final_pos, current_pos)

        # Computes all predicted states to display them
        predicted_states = [None] * len(predicted_U)
        predicted_pos = [None] * len(predicted_U)
        predicted_states[0] = current_state
        predicted_pos[0] = current_pos
        predicted_dU = [list(map(lambda x, y: x - y, ii, jj)) for ii, jj in zip(predicted_U[1:] , predicted_U[:-1])]
        for i in range(len(predicted_states) - 1):
            predicted_states[i + 1] = mpc_module.f_next_state(predicted_states[i], predicted_U[i], predicted_dU[i])
            predicted_pos[i + 1] = (predicted_states[i + 1][0], predicted_states[i + 1][1])

        # Display the next position and predicted states
        display_figure(points, current_pos, predicted_states, bounds)


        #Update records
        u_vectors.append(new_u)
        dU_vectors.append(new_dU)
        states.append(current_state)

        #De-comment to see the updates
        # print("\n NEW ITERATION : \n")
        # print("Distance until final destination :", distance)
        # print("New U vector : ", new_u)
        # print("New DU vector : ", new_dU)
        # print("New state :", new_state)
        #print("Next position to go is : ", mpc_module.path[0])
        # print("The error is : ", mpc_module.error)
        #input("Continue ? Press enter to confirm or Ctrl+C to exit")

    #De-comment to see the records
    #print("States : ",states)
    #print("U_vectors : ", u_vectors)
    #print("dU_vectors : ", dU_vectors)
    input("End point reached : press any key to terminate program ")

def main():


    start_position = [0, 0, 0, 0, 0, 0]

    print("Which algorithm do you want to run ? \n"
            "1: MPC algorithm with a 2-wheel robot model\n"
            "2: MPC algorithm with a car model\n")

    alg_choice = 0
    while alg_choice < 1 or alg_choice > 2:
        
        alg_choice = int(input("Write the option's number or 0 to quit : "))
        if alg_choice == 1:
            print("Option 1 was chosen : Robot MPC\n")
            alg = MPC_robot_model()
        elif alg_choice == 2:
            print("Option 2 was chosen : Car MPC\n")
            alg = MPC_car_model()
        elif alg_choice == 0:
            print("Program terminated\n")
            exit()
        else:
            print("Unrecognized option, please try again or type 0 to quit\n")


    print("Which circuit do you want to test ? \n"
            "1: A straight line\n"
            "2: A line with sharp turn\n"
            "3: A long turn\n"
            "4: A example of racing track\n")

    cicruit_choice = 0
    while cicruit_choice < 1 or cicruit_choice > 4:
        
        cicruit_choice = int(input("Write the circuit's number or 0 to quit : "))
        if cicruit_choice == 1:
            print("Option 1 was chosen : Straight Line\n")

            if alg_choice == 2:
                goal = (10,5)
                bounds = [-0.5, 5.5, -0.5, 10.5]
                start_position = [0, 0, 0, 0, 0, 0]

            else:
                goal = (30,15)
                bounds = [-1, 35, -1, 20]
                start_position = [0, 0, 0]

            if goal[0] != 0.0:
                m = goal[1]/goal[0]
            else:
                m = 10000.0

            x = np.linspace(0.0, goal[0], num=50)
            y = m * x
            points = list(zip(x, y))

        elif cicruit_choice == 2:
            print("Option 2 was chosen : Sharp turn line\n")

            if alg_choice == 2:
                goal = (20,1)
                x = np.linspace(0.0, goal[0], num=70)
                y = np.sin(x/2) * 7
                bounds = [-1, 13, -8, 8]
                start_position = [0, 0, mt.pi/3, 0, 0, 0]

            else:
                x = np.arange(0,30, 1)
                y = np.sin(x/3) * 5
                bounds = [-1, 35, -4, 9]
                start_position = [0, 0, 0]

            points = list(zip(x, y))

            
        elif cicruit_choice == 3:
            print("Option 3 was chosen : Long turn line\n")
            if alg_choice == 2:
                goal = (3,1)
                x = np.linspace(0.0, goal[0], num=40)
                y = np.exp(x)
                bounds = [-0.5, 3.2, -0.5, 30]
                start_position = [0, 1, 0, 0, 0, 0]

            else:
                x = np.arange(0,30, 1)
                y = np.exp(x/10)
                bounds = [-1, 35, -1, 25]
                start_position = [0, 0, 0]

            points = list(zip(x, y))


        elif cicruit_choice == 4:
            print("Option 4 was chosen : Racing track\n")
            points = utils.CIRCUIT
            bounds = [-2, 35, -8, 20]
            if alg_choice == 2:
                start_position = [points[0][0], points[0][1], 0, 0, 0, 0]
            else:
                start_position = [points[0][0], points[0][1], 0]

        elif cicruit_choice == 0:
            print("Program terminated\n")
            exit()
        else:
            print("Unrecognized circuit, please try again or type 0 to quit\n")

    # if we want to simulate the car on the racing track, we have to smooth the path 
    # as for this model, the MPC needs more points to perform well
    if alg_choice == 2 and cicruit_choice == 4:
        new_points = utils.smooth_path(points)
    else:
        new_points = points

    plot_MPC(alg, start_position, new_points, bounds)

if __name__ == "__main__":
    main()
