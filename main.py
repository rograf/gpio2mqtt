from gpiozero import Button, OutputDevice
from signal import pause
from threading import Timer
import paho.mqtt.client as mqtt
import yaml
import json

# Load configuration from YAML file
with open('config.yaml', 'r') as yamlfile:
    config = yaml.safe_load(yamlfile)

# MQTT settings
MQTT_SERVER = config['mqtt']['server']
MQTT_PORT = config['mqtt']['port']
MQTT_SECURE = config['mqtt']['secure']
MQTT_USERNAME = config['mqtt']['username']
MQTT_PASSWORD = config['mqtt']['password']
MQTT_BASE_TOPIC = config['mqtt']['base_topic']
MQTT_TOPIC_STATUS_REQUEST = f"{MQTT_BASE_TOPIC}/status/request"
MQTT_TOPIC_STATUS_RESPONSE = f"{MQTT_BASE_TOPIC}/status/response"

# Set up MQTT client
client = mqtt.Client()
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

# Define the GPIO to monitor and control
gpio_inputs = config['gpio_inputs']
gpio_outputs = config['gpio_outputs']

# Debounce time in seconds
debounce_time = 0.5

# Timers for debouncing
timers = {gpio: None for gpio in gpio_inputs + gpio_outputs}

# State dictionary to track stable states
stable_states = {gpio: None for gpio in gpio_inputs}

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    send_all_statuses()
    # Subscribing to the control topic for each GPIO outputs
    client.subscribe(MQTT_TOPIC_STATUS_REQUEST)
    for gpio in gpio_outputs:
        client.subscribe(f"{MQTT_BASE_TOPIC}/{gpio}/set")

# Function to control relay based on GPIO
outputs_states = {}

def control_relay(gpio, power, duration):
    global outputs_states

    oposite_power = not power

    if power is True:
        if gpio not in outputs_states:
            outputs_states[gpio] = OutputDevice(gpio)
        outputs_states[gpio].on()
        publish_state(gpio, {
            "power": True,
        })
    elif power is False:
        if gpio in outputs_states:
            outputs_states[gpio].off()  # Ensure to turn off before deleting
            del outputs_states[gpio]
        publish_state(gpio, {
            "power": False,
        })
    elif power is None:
        if gpio in outputs_states:
            control_relay(gpio, False, 0)
            oposite_power = True
        else:
            control_relay(gpio, True, 0)
            oposite_power = False
    else:
        print(f"Unknown power state: {power}")

    if duration and duration != 0:
        # Toggle the relay after the duration
        t = Timer(duration / 1000.0, lambda: control_relay(gpio, oposite_power, 0))
        t.start()

def publish_state(gpio, data):
    topic = f"{MQTT_BASE_TOPIC}/{gpio}"
    client.publish(topic, json.dumps(data))

# Callback function to run when a GPIO state changes
def gpio_changed(gpio):
    global stable_states
    if gpio.is_pressed:
        stable_states[gpio.pin.number] = True
    else:
        stable_states[gpio.pin.number] = False
    
    if timers[gpio.pin.number] is not None:
        timers[gpio.pin.number].cancel()
    
    timers[gpio.pin.number] = Timer(debounce_time, send_status, [gpio.pin.number])
    timers[gpio.pin.number].start()

def send_all_statuses():
    global stable_states, outputs_states
    
    # Prepare the status of GPIO inputs
    input_statuses = {gpio: state for gpio, state in stable_states.items()}
    
    # Prepare the status of GPIO outputs
    output_statuses = {gpio: (True if gpio in outputs_states and outputs_states[gpio].value else False) for gpio in gpio_outputs}

    status_message = {
        "inputs": input_statuses,
        "outputs": output_statuses
    }
    print(f"Sending all statuses: {status_message}")
    client.publish(MQTT_TOPIC_STATUS_RESPONSE, json.dumps(status_message))

# Function to send status to MQTT and print to console
def send_status(gpio):
    global stable_states
    state = stable_states[gpio]
    message = f"GPIO {gpio} is {state} to GND"
    print(message)
    publish_state(gpio, {"connected": state})

# Callback function for MQTT messages
def on_message(client, userdata, message):
    print(f"Received MQTT message on topic {message.topic}: {message.payload.decode()}")
    try:
        if message.topic == MQTT_TOPIC_STATUS_REQUEST:
            send_all_statuses()
        elif message.topic.startswith(MQTT_BASE_TOPIC) and message.topic.endswith('/set'):
            gpio = int(message.topic.split('/')[-2])
            if gpio in gpio_outputs:
                data = json.loads(message.payload.decode())
                power = data.get('power')
                duration = data.get('duration', 0)
                control_relay(gpio, power, duration)
            else:
                print(f"GPIO {gpio} is not configured for control")
        else:
            print(f"Ignoring MQTT message on unexpected topic: {message.topic}")
    except Exception as e:
        print(f"Error processing MQTT message: {e}")

client.on_connect = on_connect
client.on_message = on_message

# Function to get initial GPIO states
def get_initial_states():
    global stable_states
    for gpio in gpio_inputs:
        button = Button(gpio, pull_up=True)
        stable_states[gpio] = button.is_pressed

# Get initial GPIO states
get_initial_states()



# Set up the GPIO GPIO as inputs with pull-up resistors
buttons = [Button(gpio, pull_up=True) for gpio in gpio_inputs]

# Attach the callback function to each GPIO
for button in buttons:
    button.when_pressed = lambda btn=button: gpio_changed(btn)
    button.when_released = lambda btn=button: gpio_changed(btn)

# Connect to MQTT broker
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

if MQTT_SECURE:
    client.tls_set()

client.connect(MQTT_SERVER, MQTT_PORT, 60)

# Loop to maintain network traffic flow with the broker
client.loop_forever()
