# GPIO2MQTT

## License
This project is licensed for non-commercial use. You are allowed to use, modify, and distribute this code for non-commercial purposes, provided that you include proper attribution to the author, Rafał Rogulski.

## Introduction
This project bridges GPIO inputs and outputs on a Raspberry Pi to MQTT topics. It allows you to monitor and control GPIO remotely via MQTT.

## Configuration

Create a `config.yaml` file in the root directory with the following structure:

```yaml
mqtt:
  server: "YOUR_MQTT_SERVER"
  port: 8883  # Use 1883 for non-secure connections
  secure: true  # Set to false if not using TLS/SSL
  username: "YOUR_MQTT_USERNAME"
  password: "YOUR_MQTT_PASSWORD"
  base_topic: "gpio2mqtt"

gpio_inputs:
  - 17  # GPIO to monitor

gpio_outputs:
  - 18  # GPIO to control
```

## Installation
1. Install the required dependencies:
```bash
  pip install paho-mqtt gpiozero pyyaml
```

2. Clone the repository:
```bash
git clone https://github.com/your-repo/gpio2mqtt.git
cd gpio2mqtt
```

3. Create and configure `config.yaml` as described above

## Running the Program
Run the Python script:
```bash
sudo python3 main.py
```

## MQTT Messages

### Generated Messages
- Status Request: Sent to `gpio2mqtt/status/request` to request the status of all monitored and controlled GPIO.
- Status Response: Published to `gpio2mqtt/status/response` containing the status of all monitored and controlled GPIO.
```bash
{
  "inputs": {
    "GPIO_17": true  # True if connected to GND, False otherwise
  },
  "outputs": {
    "GPIO_18": true  # True if the output is ON, False otherwise
  }
}
```
- State Update: Published to `gpio2mqtt/GPIO_{NUMBER}` for each monitored GPIO change.
```bash
{
  "connected": true  # True if connected to GND, False otherwise
}
```
### Received Messages
- Control Message: Received on `gpio2mqtt/GPIO_{NUMBER}/set` to control an output GPIO.
```bash
{
  "power": true,  # True to turn ON, False to turn OFF, None to toggle
  "duration": 5000  # Optional: Duration in milliseconds to maintain the state
}
```

### Disclaimer: 
This code is provided as-is, without any warranty of its functionality. I do not assume any responsibility for any errors that may occur.