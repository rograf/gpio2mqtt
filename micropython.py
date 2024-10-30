from machine import Pin, Timer
import time
import sys
from umqtt.robust import MQTTClient
import network
import json
import gc
from _thread import start_new_thread
import tls

# Load configuration from JSON file
with open('config.json', 'r') as jsonfile:
    config = json.load(jsonfile)

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

# GPIO configuration
gpio_inputs = config['gpio_inputs']
gpio_outputs = config['gpio_outputs']

# MQTT client with TLS/SSL if secure is enabled
context = tls.SSLContext(tls.PROTOCOL_TLS_CLIENT)
context.verify_mode = tls.CERT_NONE
client = MQTTClient(
    client_id=MQTT_CLIENT_ID,
    server=MQTT_SERVER,
    port=MQTT_PORT,
    user=MQTT_USERNAME,
    password=MQTT_PASSWORD,
    keepalive=7200,
    ssl=context
)

# GPIO states
input_states = {gpio: None for gpio in gpio_inputs}
output_states = {}

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

# Function to control output GPIO
def control_relay(gpio, power, duration):

    pin = Pin(gpio, Pin.OUT)
    current_state = pin.value()
    
    # If power is True and pin is off, turn it on
    if power is True and current_state == 1:
        pin.value(0)
        output_states[gpio] = True
        publish_state(gpio, {"power": True})

    elif power is False and current_state == 0:
        pin.value(1)
        output_states[gpio] = False
        publish_state(gpio, {"power": False})
    
    if duration and duration != 0:
        output_durations[gpio] = {
            "start_time": time.ticks_ms(),
            "duration": duration,
            "power": not power
        }


# Function to handle GPIO input state change
def gpio_changed(pin):
    gpio = pin_number_map[pin]
    if input_states[gpio] != (not pin.value()):
      input_states[gpio] = not pin.value()
      send_status(gpio)

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






