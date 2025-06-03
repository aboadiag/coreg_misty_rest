from flask import Flask, request, jsonify, send_from_directory
import requests
import time
import json

## MISTY URL AND ENDPOINTS
MISTY_URL = "http://172.26.189.224"

# Expression endpoints
AUDIO_PLAY_ENDPOINT = "/api/audio/play"
LED_ENDPOINT = "/api/led"
# Motion endpoints
ARM_ENDPOINT = "/api/arms/set"
FACE_IMAGE_ENDPOINT = "/api/images/display"
HEAD_ENDPOINT = "/api/head"
# Sensor endpoints
SERIAL_ENDPOINT = "/api/serial"

# Misty full urls
led_url = f"{MISTY_URL}{LED_ENDPOINT}"
audio_url = f"{MISTY_URL}{AUDIO_PLAY_ENDPOINT}"
arms_url = f"{MISTY_URL}{ARM_ENDPOINT}"
face_url = f"{MISTY_URL}{FACE_IMAGE_ENDPOINT}"
head_url = f"{MISTY_URL}{HEAD_ENDPOINT}"
serial_url = f"{MISTY_URL}{SERIAL_ENDPOINT}"

## global values and variables 
#constants
BUFFER_SIZE = 10
ABS_HYSTERESIS = 0.098  # ±0.5% of 2g
ARM_MIN = -28 # max +1 straight up
ARM_MAX = 89 # max-1 straight down
FULL_SCALE_RANGE = 2 * 9.81  # 2g (19.62)

#variables
circQueue = []
acc_Init = None
prev_angle = 0
last_move_time = 0
DEBOUNCE_MS = 200

# helper function: read serial
# returns float: accelerometer value
def get_serial_data():
    print("getting serial data from misty")
    """get request from misty"""
    # try >> request data:
    try:
        # request data from misty
        response = requests.get(serial_url)
        # convert to json
        data = response.json()
        messages = data.get("result", []) # this is the format of response
        print(f"Raw messages: {messages}")

        if not messages:
            print("No messages found in result.")
            return None
        #ret: latest valid method
        for msg in reversed(messages):
            # try: parse message
            try:
                accel_val = float(eval(msg)["accelerometer_z"])
                print(f"accelerometer value {accel_val}")
                return accel_val  # return once a valid one is found
            #catch: parse message
            except Exception as e:
                print(f"Error parsing message: {e}")
                continue
            
    # catch >> request data:
    except Exception as e:
        print(f"Error fetching serial data: {e}")
    return None


# helper function:
def moving_mean_filter(new_accel):
    global circQueue
    circQueue.append(new_accel)

    if len(circQueue) > BUFFER_SIZE:
        # drop oldest sample (i.e., element from FRONT of queue)
        circQueue.pop(0)

    if len(circQueue) < BUFFER_SIZE:
        # add to the queue
        print("Not enough data added to queue")
        return None
    
    avgVal = sum(circQueue) / len(circQueue)
    print(f" The filtered accelerometer value is: {avgVal}")
    return avgVal

def misty_synchronize(filt_accel, accel_Init):
    global acc_Init, last_move_time

    ## 1) check if initial acceleration value accInit has been assigned
    if acc_Init is None:
        acc_Init = filt_accel
        print(f"Initial accelerometer value is {filt_accel}")
        # continue

    ## 2) compute change in acceleration current acceleration value (filteredAccVal)
    print("Computing delta")
    delta_accel = filt_accel - acc_Init

    print(f"delta a is {delta_accel}")

    ## 3) check if the absolute change in acceleration (deltaA) is greater than the absolute hysteresis (absHysteresis)
    if abs(delta_accel) > ABS_HYSTERESIS:
        curr_time_ms = time.time() * 1000
        if curr_time_ms - last_move_time < DEBOUNCE_MS:
            print("not enough time has passed. not moving arms yet...")
            return
        
        # else, its time to move robot arms
        last_move_time = curr_time_ms
   
        # map deltaA to robot
        delta_robot = map_to_robot(delta_accel)

        # move robot arms
        move_misty_arms(delta_robot)



def map_to_robot(delta_acc):
    global FULL_SCALE_RANGE, ARM_MIN, ARM_MAX
    robot_min = ARM_MIN
    robot_max = ARM_MAX
    robot_span = robot_max - robot_min
    # max delta accel should be between -19.62 and the the value that is smaller (either 19.62 or delta_accel)
    delta_acc_clamped = max(-FULL_SCALE_RANGE, min(FULL_SCALE_RANGE, delta_acc))
    # Convert the left range into a 0-1 range (float) accounts for edges: i.e. when delta = -fsr --> 0, delta = +fsr = 1
    normed_accel = (delta_acc_clamped + FULL_SCALE_RANGE)/ (2 * FULL_SCALE_RANGE)

    #Convert the 0-1 range into a value in the right range
    robot_motion = robot_min + (normed_accel * robot_span)

    # clamp robot motion so doesnt fall outside possible range --> find the min value between 90 (straight down) and (max of -29 (up) and robot motion)
    robot_motion_clamped = min(robot_max, max(robot_min, robot_motion))

    print(f"Mapped robot value is: {robot_motion_clamped}")

    return robot_motion_clamped


def move_misty_arms(delta_robot):
    print(f"Moving misty's arms to {delta_robot}")
    payload = {
        "leftArmPosition": delta_robot,
        "rightArmPosition": delta_robot,
        "LeftArmVelocity": 50,
        "RightArmVelocity": 50,
    }
    try:
            response = requests.post(
                arms_url,
                headers={"Content-Type": "application/json"},
                data=json.dumps(payload))
        
            if response.status_code == 200:
                print("Arms moved successfully")
                print(response.json())  # Print response from Misty (optional)
            else:
                print(f"Error: {response.status_code}")

    #timeout exception        
    except requests.exceptions.Timeout:
        print("Timeout occurred while trying to connect to Misty")


    #failure to send request request
    except requests.exceptions.RequestException as e:
        print(f"Error making request: {e}")

    # init current time
    # curr_time = time()
    #check if the time since last arm movement is less than debounce time -->




# loop 
def run_interaction():
    global acc_Init
    while True:
        unfilt_accel = get_serial_data()
        if unfilt_accel is None:
            print("Raw acceleration is None")
            time.sleep(0.1)
            continue

    ## 1)  --- MOVING AVERAGE FILTER:  find average of bufferSize most recent ---
        filt_accel = moving_mean_filter(unfilt_accel)
        if filt_accel is None:
            print("Filtered acceleration is None (buffer not full yet)")
            time.sleep(0.1)
            continue


    ## 2)  --- SYNCHRONIZE MISTY MOTION with ACCELEROMETER --- 
        misty_synchronize(filt_accel, acc_Init)


try:
    run_interaction()
except KeyboardInterrupt:
    print("Exiting loop.")


