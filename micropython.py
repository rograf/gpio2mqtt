from machine import Pin, Timer
import time
import sys
from umqtt.robust import MQTTClient
import network
import json
import gc
from _thread import start_new_thread
import tls
import time 

# Load configuration from JSON file
with open('config.json', 'r') as jsonfile:
    config = json.load(jsonfile)

# Device password if not exist set to abc
CONFIG_PASSWORD = config['password'] if 'password' in config else False

# Wi-Fi settings
WIFI_SSID = config['wifi']['ssid']
WIFI_PASSWORD = config['wifi']['password']

# MQTT settings
MQTT_SERVER = config['mqtt']['server']
MQTT_PORT = config['mqtt']['port']
MQTT_SECURE = config['mqtt']['secure']
MQTT_USERNAME = config['mqtt']['username']
MQTT_PASSWORD = config['mqtt']['password']
MQTT_CLIENT_ID = config['mqtt']['client_id']
MQTT_BASE_TOPIC = config['mqtt']['base_topic']
MQTT_TOPIC_STATUS_REQUEST = f"{MQTT_BASE_TOPIC}/status/request"
MQTT_TOPIC_STATUS_RESPONSE = f"{MQTT_BASE_TOPIC}/status/response"
MQTT_TOPIC_STATUS_UPDATE = f"{MQTT_BASE_TOPIC}/update"

# GPIO configuration
gpio_inputs = config['gpio_inputs']
gpio_outputs = config['gpio_outputs']
DEBOUNCE_TIME_MS = config['debounce_time_ms']  # Time in milliseconds for debounce

# MQTT client with TLS/SSL if secure is enabled
if MQTT_SECURE:
    ssl_context = tls.SSLContext(tls.PROTOCOL_TLS_CLIENT)
    ssl_context.verify_mode = tls.CERT_NONE
    client = MQTTClient(
        client_id=MQTT_CLIENT_ID,
        server=MQTT_SERVER,
        port=MQTT_PORT,
        user=MQTT_USERNAME,
        password=MQTT_PASSWORD,
        keepalive=7200,
        ssl=ssl_context
    )
else:
    client = MQTTClient(
        client_id=MQTT_CLIENT_ID,
        server=MQTT_SERVER,
        port=MQTT_PORT,
        user=MQTT_USERNAME,
        password=MQTT_PASSWORD,
        keepalive=7200
    )

# GPIO states
input_states = {gpio: None for gpio in gpio_inputs}
output_states = {}
debounce_timers = {}  # Debounce timers for each input pin
previous_states = {}  # Track the previous state for each input

# List to track output durations
output_durations = {}

pin_number_map = {}
last_message_time = time.ticks_ms()

# Function to connect to Wi-Fi with retries
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    failure_cycles = 0  # Counter for failed connection cycles
    
    if not wlan.isconnected():
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        attempt = 0
        while not wlan.isconnected():
            attempt += 1
            print(f"Attempting to connect to Wi-Fi (attempt {attempt})...")
            time.sleep(1)
            if attempt >= 10:  # Retry for 10 seconds
                print("Failed to connect to Wi-Fi. Retrying...")
                attempt = 0  # Reset attempts
                failure_cycles += 1  # Increment the failure cycle count

            # If failed 3 cycles of 10 attempts, restart the machine
            if failure_cycles >= 3:
                print("Failed to connect after 3 cycles. Rebooting device...")
                machine.reset()  # Reset the device after 3 failed cycles

    print(f"Connected to Wi-Fi: {wlan.ifconfig()}")

def monitor_wifi():
    wlan = network.WLAN(network.STA_IF)
    if not wlan.isconnected():
        print("Wi-Fi connection lost. Reconnecting...")
        connect_wifi()


# Function to reconnect to MQTT with retries
def mqtt_connect():
    connected = False
    attempt = 0
    while not connected:
        try:
            print(f"Attempting to connect to MQTT broker (attempt {attempt+1})...")
            client.connect()
            client.subscribe(MQTT_TOPIC_STATUS_REQUEST)
            client.subscribe(MQTT_TOPIC_STATUS_UPDATE)
            for gpio in gpio_outputs:
                client.subscribe(f"{MQTT_BASE_TOPIC}/GPIO_{gpio}/set")
            connected = True
            print("Connected to MQTT broker")
        except OSError as e:
            print(f"Failed to connect to MQTT broker: {e}")
            attempt += 1
            time.sleep(3)  # Retry every 3 seconds
            if attempt >= 5:  # Retry 5 times before resetting attempt
                attempt = 0

# Function to publish GPIO state
def publish_state(gpio, data):
    topic = f"{MQTT_BASE_TOPIC}/GPIO_{gpio}"
    client.publish(topic, json.dumps(data))
    print(f"MQTT message sent to topic {topic}: {data}")

# Function to turn on the GPIO pin
def turn_on_gpio(gpio):
    pin = Pin(gpio, Pin.OUT)  # Set the pin as an output
    pin.value(1)  # Turn GPIO ON
    output_states[gpio] = True
    publish_state(gpio, {"power": True})

# Function to turn off the GPIO pin
def turn_off_gpio(gpio):
    pin = Pin(gpio, Pin.OUT)  # Set the pin as an output
    pin.value(0)  # Turn GPIO OFF
    pin.init(Pin.IN)  # Switch pin to input mode
    output_states[gpio] = False
    publish_state(gpio, {"power": False})

# Function to control the relay with duration
def control_relay(gpio, power, duration):
    pin = Pin(gpio, Pin.OUT)  # Set the pin as an output
    current_state = pin.value()

    # If power is True, pin is off, and duration is set, turn on the GPIO
    if power and current_state == 0 and duration != 0:
        turn_on_gpio(gpio)
        time.sleep(duration / 1000)  # Convert milliseconds to seconds
        turn_off_gpio(gpio)

    # If power is False, pin is on, and duration is set, turn off the GPIO
    elif not power and current_state == 1 and duration != 0:
        turn_off_gpio(gpio)
        time.sleep(duration / 1000)  # Convert milliseconds to seconds
        turn_on_gpio(gpio)
        
# Function to handle GPIO input state change
def gpio_changed(pin):
    gpio = pin_number_map[pin]

    # Stop any existing debounce timer
    if gpio in debounce_timers and debounce_timers[gpio] is not None:
        debounce_timers[gpio].deinit()
    
    # Start a new debounce timer
    debounce_timers[gpio] = Timer()
    debounce_timers[gpio].init(
        period=DEBOUNCE_TIME_MS, mode=Timer.ONE_SHOT,
        callback=lambda t: handle_debounce(gpio, pin.value())
    )
    
def handle_debounce(gpio, current_state):
    """Handle the debounced GPIO input state change."""
    inverted_state = 0 if current_state == 1 else 1
    previous_state = previous_states.get(gpio)

    # Update the state for input pins
    input_states[gpio] = current_state == 0

    # Check for state change
    if inverted_state != previous_state:
        print(f"Input Pin {gpio} state changed to: {inverted_state}")
        send_status(gpio)

    # Store the new state
    previous_states[gpio] = inverted_state
    
# Function to send the status of all GPIO
def send_all_statuses():
    input_statuses = {f"GPIO_{gpio}": state for gpio, state in input_states.items()}
    output_statuses = {f"GPIO_{gpio}": state for gpio, state in output_states.items()}
    
    status_message = {
        "inputs": input_statuses,
        "outputs": output_statuses
    }
    
    client.publish(MQTT_TOPIC_STATUS_RESPONSE, json.dumps(status_message))
    print(f"All statuses sent: {status_message}")

# Function to send the status of a single GPIO
def send_status(gpio):
    state = input_states[gpio]  # Send True or False directly
    print(f"GPIO {gpio} is {'connected' if state else 'disconnected'}")
    publish_state(gpio, {"connected": state})  # Publish as True/False to MQTT

# MQTT message handler
def on_message(topic, msg):
    print(f"Received MQTT message on topic {topic}: {msg}")
    if topic == MQTT_TOPIC_STATUS_REQUEST.encode():  # Convert to bytes
        send_all_statuses()
    if topic == MQTT_TOPIC_STATUS_UPDATE.encode():
         handle_update(json.loads(msg))
    elif topic.startswith(MQTT_BASE_TOPIC.encode()) and topic.endswith(b'/set'):  # Convert to bytes
        gpio = int(topic.split(b'/')[-2].split(b'_')[1])  # Parse as bytes
        if gpio in gpio_outputs:
            data = json.loads(msg.decode())  # Decode message from bytes to str
            power = data.get('power')
            duration = data.get('duration', 0)
            control_relay(gpio, power, duration)

# Initialize MQTT
def mqtt_init():
    client.set_callback(on_message)
    mqtt_connect()
    
# Update Config MQTT
def handle_update(data):
    global config
    
    if CONFIG_PASSWORD is False:
        print("Password not set. Please set a password in the config file.")
        return

    # Verify the password
    if 'password' not in data or data['password'] != CONFIG_PASSWORD:
        print("Invalid password")
        return
    
    # Check if a new password is provided
    if 'new_password' in data:
        config['password'] = data['new_password']  # Replace the old password with the new one
        print("Password updated")
    
    # Update other config values, excluding password-related fields
    for key, value in data.items():
        if key in config and key != 'password':  # Ensure 'password' is managed separately
            config[key] = value
    
    # Save updated config back to file
    with open('config.json', 'w') as jsonfile:
        json.dump(config, jsonfile)
    
    print("Config updated, restarting...")
    machine.reset()  # Restart the device

# Non-blocking function to handle output durations
def check_output_durations():
    current_time = time.ticks_ms()
    for gpio, timer in list(output_durations.items()):
        if time.ticks_diff(current_time, timer['start_time']) > timer['duration']:
            power = timer['power']
            pin = machine.Pin(gpio, machine.Pin.OUT)
            if power is False:
                pin.value(1)
                publish_state(gpio, {"power": False})
            elif power is True:
                pin.value(0)
                publish_state(gpio, {"power": True})
            del output_durations[gpio]  # Remove the completed timer
            
def keep_connection():
    global last_message_time
    if time.ticks_diff(time.ticks_ms(), last_message_time) > 60000:
        monitor_wifi()
        topic = f"{MQTT_BASE_TOPIC}/KEEP_ALIVE"
        client.publish(topic, json.dumps({}))
        last_message_time = time.ticks_ms()

# Clean shutdown function
def clean_shutdown():
    print("Shutting down...")
    client.disconnect()  # Disconnect from MQTT
    print("Shutdown complete.")
  
    
# Main loop
def main():
    try:
        connect_wifi()  # Connect to Wi-Fi with retries
        mqtt_init()  # Initialize and connect to MQTT
        
        # Configure input GPIO
        for gpio in gpio_inputs:
            pin = machine.Pin(gpio, machine.Pin.IN, machine.Pin.PULL_UP)
            input_states[gpio] = not pin.value()
            pin_number_map[pin] = gpio
            pin.irq(trigger=machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, handler=lambda pin: gpio_changed(pin))
        
        for gpio in gpio_outputs:
            pin = Pin(gpio, Pin.OUT)
            pin.value(1)
            output_states[gpio] = not pin.value()
            
        send_all_statuses()  # Send all statuses on startup
            
        while True:
            try:
                client.check_msg()  # Check for MQTT messages
            except OSError as e:
                print(f"MQTT error: {e}")
                mqtt_connect()  # Attempt to reconnect MQTT if it disconnects

            check_output_durations()  # Check if any output duration timers have expired
            keep_connection()
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        clean_shutdown()

if __name__ == "__main__":
    main()
