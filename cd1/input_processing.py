


PAIR_SEPARATOR = "/"
COORDINATES_SEPARATOR = ","


"""
Convert the path as input string into a list of pair

Arguments:

    msg: 
        input string
"""


def parsing_path_input(msg):
    pair_of_coordinates = msg.split(PAIR_SEPARATOR)

    path_char = list()
    for string_pair in pair_of_coordinates:
        path_char.append(string_pair.split(COORDINATES_SEPARATOR))

    path_int = list()
    for pair in path_char:
        path_int.append((float(pair[0]),float(pair[1])))
    
    return path_int


VALUE_SEPARATOR = ","

"""
Convert the state of the car as input string into a list of values

Arguments:

    msg: 
        input string
"""

def parsing_car_state(msg):
    state_char = msg.split(VALUE_SEPARATOR)

    state = list()
    for value in state_char:
        state.append(float(value))

    print(state)
    return state