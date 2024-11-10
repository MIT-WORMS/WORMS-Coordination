import pandas as pd
import os

def find_robot_name(mac_address, spreadsheet_path):
    """
    Returns species name based on mac_address.
    
    Args:
    - mac_address: str, MAC address, retreived from get_mac_address
    - spreadsheet_path: str, file path to spreadsheet database.csv
    """
    df = pd.read_csv(spreadsheet_path)
    print(df)
    print("\n")
    match = df.loc[df['MAC Address'] == mac_address, 'Species']

    if not match.empty:
        return match.iloc[0]
    else:
        return None
    
def find_configuration(head, spreadsheet_path):
    """
    Returns species specialiation (motor directions) based on head.txt.

    Args:
    - head: str, head, read from head.txt
    - spreadsheet_path: str, file path to spreadsheet specialization_table.csv
    """
    df = pd.read_csv(spreadsheet_path)
    match = df.loc[df['Head'] == head]
    if not match.empty:
        return list(match.iloc[0].iloc[2:].values)
    else:
        return None
    
spreadsheet_path = os.path.expanduser('~/WORMS-software/src/worms_mech/worms_mech/database.csv')
specialization_path = os.path.expanduser('~/WORMS-software/src/worms_mech/worms_mech/specialization_table.csv')
mac_address = "dc:a6:32:c4:51:3b"

def find_all_worms(spreadsheet_path):
    """
    Returns all worms present in the system based on spreadsheet path.

    Args:
    - spreadsheet_path: str, file path to spreadsheet all_worms.csv
    """

    df = pd.read_csv(spreadsheet_path)
    all_worms = df["Species"]
    for i in all_worms:
        print(type(i))

def get_waypoints(action):
    """
    Returns waypoints from csv file given some action.
    """
    # waypoints_path = os.path.expanduser(f'~/WORMS-software/src/worms_mech/worms_mech/gait_data/{action}.csv')
    waypoints_path = os.path.expanduser(f'~/WORMS-testing/src/gait_package/gait_package/gait_data/{action}.csv')

    df = pd.read_csv(waypoints_path)
    return [list(i) for i in df.values]

def interpolate_waypoints(current_position, arrays, increment=0.3):
    """
    Transition from the current position to each array in sequence by 0.1 increments, producing a comprehensive list of
    lists representing each incremental step towards the waypoints.

    :param arrays: A sequence of arrays where each array is a list of numbers.
    :param increment: The incremental value to adjust the numbers. Default is 0.1.
    :return: A list of lists representing each step through the waypoints.
    """
    transition_steps = []

    # Start from the current position
    current_state = current_position

    for target_array in arrays:
        while True:
            step = []
            done = True
            for current_val, target_val in zip(current_state, target_array):
                if abs(target_val - current_val) > increment:
                    done = False
                    if target_val > current_val:
                        step.append(current_val + increment)
                    else:
                        step.append(current_val - increment)
                else:
                    step.append(target_val)
            
            current_state = step
            transition_steps.append(step)
            
            if done:
                break

    return transition_steps


# find_all_worms(spreadsheet_path)

waypoints = get_waypoints("stand_step")

interpolated = interpolate_waypoints([0,0,0], waypoints)
print(interpolated)




