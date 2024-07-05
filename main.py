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
MQTT_BROKER = config['mqtt']['server']
MQTT_PORT = config['mqtt']['port']
MQTT_USERNAME = config['mqtt']['username']
MQTT_PASSWORD = config['mqtt']['password']
BASE_TOPIC = config['mqtt']['base_topic']

# Set up MQTT client
client = mqtt.Client()
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

# Define the GPIO pins to monitor and control
pins_to_monitor = config['gpio_to_monitor']
pins_to_control = config['gpio_to_control']

# Debounce time in seconds
debounce_time = 0.5

# Timers for debouncing
timers = {pin: None for pin in pins_to_monitor + pins_to_control}

# State dictionary to track stable states
stable_states = {pin: None for pin in pins_to_monitor}

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    # Subscribing to the control topic for each GPIO pin
    for pin in pins_to_control:
        client.subscribe(f"{BASE_TOPIC}/{pin}/set")

# Function to control relay based on GPIO pin
gpio_outputs = {}

def control_relay(pin, mode, duration):
    global gpio_outputs

    oposite_mode = 'off' if mode == 'on' else 'on'
 
    if mode == 'on':
        if pin not in gpio_outputs:
            gpio_outputs[pin] = OutputDevice(pin)
            gpio_outputs[pin].on()
            publish_state(pin, {
                "power": True,
            })
    elif mode == 'off':
        if pin in gpio_outputs:
            del gpio_outputs[pin]
            publish_state(pin, {
                "power": False,
            })
    elif mode == 'toggle':
        if pin in gpio_outputs:
            control_relay(pin, 'off', 0)
        else:
            control_relay(pin, 'on', 0)
    else:
        print(f"Unknown mode: {mode}")


    if duration and duration != 0:
        # Toggle the relay after the duration
        t = Timer(duration / 1000.0, lambda: control_relay(pin, oposite_mode, 0))
        t.start()

def publish_state(gpio_pin, data):
    topic = f"{BASE_TOPIC}/{gpio_pin}"
    client.publish(topic, json.dumps(data))

# Callback function to run when a pin state changes
def pin_changed(pin):
    global stable_states
    if pin.is_pressed:
        stable_states[pin.pin.number] = True
    else:
        stable_states[pin.pin.number] = False
    
    if timers[pin.pin.number] is not None:
        timers[pin.pin.number].cancel()
    
    timers[pin.pin.number] = Timer(debounce_time, send_status, [pin.pin.number])
    timers[pin.pin.number].start()

# Function to send status to MQTT and print to console
def send_status(pin_number):
    global stable_states
    state = stable_states[pin_number]
    message = f"Pin {pin_number} is {state} to GND"
    print(message)
    publish_state(pin_number, {"connected": state})

# Callback function for MQTT messages
def on_message(client, userdata, message):
    print(f"Received MQTT message on topic {message.topic}: {message.payload.decode()}")
    try:
        data = json.loads(message.payload.decode())
        gpio_pin = int(message.topic.split('/')[-2])
        mode = data.get('mode', 'toggle')
        duration = data.get('duration', 0)
        
        if gpio_pin in pins_to_control:
            control_relay(gpio_pin, mode, duration)
        else:
            print(f"GPIO pin {gpio_pin} is not configured for control")
    except Exception as e:
        print(f"Error processing MQTT message: {e}")

client.on_connect = on_connect
client.on_message = on_message

# Function to get initial pin states
def get_initial_states():
    global stable_states
    for pin in pins_to_monitor:
        button = Button(pin, pull_up=True)
        stable_states[pin] = button.is_pressed

# Get initial pin states
get_initial_states()



# Set up the GPIO pins as inputs with pull-up resistors
buttons = [Button(pin, pull_up=True) for pin in pins_to_monitor]

# Attach the callback function to each pin
for button in buttons:
    button.when_pressed = lambda btn=button: pin_changed(btn)
    button.when_released = lambda btn=button: pin_changed(btn)

# Connect to MQTT broker
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Send initial pin statuses to MQTT and print to console
for pin, state in stable_states.items():
    message = f"Initial state: Pin {pin} is {state} to GND"
    print(message)
    publish_state(pin, {
        "initial": True,
        "connected": state
    })

# Loop to maintain network traffic flow with the broker
client.loop_forever()
