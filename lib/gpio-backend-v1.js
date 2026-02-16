// GPIO backend for libgpiod v1 (older systems)
// Uses polling for monitoring (gpioget every 50ms)
// Command syntax: gpioget --bias=pull-up gpiochip0 NUM
//                 gpioset --mode=signal gpiochip0 NUM=VALUE

const { spawn, exec } = require('child_process');

function readGpioState(gpioNum) {
  return new Promise((resolve, reject) => {
    exec(`gpioget --bias=pull-up gpiochip0 ${gpioNum}`, (error, stdout) => {
      if (error) {
        reject(error);
        return;
      }
      // gpioget returns "0" (LOW/connected) or "1" (HIGH/disconnected)
      const value = parseInt(stdout.trim());
      resolve(value === 0); // true = connected to ground (LOW)
    });
  });
}

function setGpioOutput(gpioNum, value, gpioOutputStates) {
  const gpioId = `GPIO${gpioNum}`;

  return new Promise((resolve, reject) => {
    // Kill any existing process for this GPIO
    if (gpioOutputStates[gpioId] && gpioOutputStates[gpioId].process) {
      gpioOutputStates[gpioId].process.kill();
      setTimeout(() => doSet(), 50);
    } else {
      doSet();
    }

    function doSet() {
      const pinValue = value === 1 ? 0 : 1; // active-low relay
      const proc = spawn('gpioset', ['--mode=signal', 'gpiochip0', `${gpioNum}=${pinValue}`]);

      proc.on('error', (err) => {
        console.error(`Error setting GPIO${gpioNum}:`, err.message);
        reject(err);
      });

      const power = value === 1;
      gpioOutputStates[gpioId] = { power, process: proc };
      resolve(power);
    }
  });
}

function startMonitoring(gpio, gpioStates, onEdge) {
  const gpioId = `GPIO${gpio.gpio}`;

  // Poll GPIO state every 50ms
  const pollInterval = setInterval(async () => {
    try {
      const connected = await readGpioState(gpio.gpio);
      const state = gpioStates[gpioId];

      if (state.currentValue !== connected) {
        state.currentValue = connected;
        onEdge(gpioId, gpio, connected);
      }
    } catch (err) {
      console.error(`Error reading ${gpioId} (${gpio.name}):`, err.message);
    }
  }, 50);

  console.log(`Started monitoring ${gpioId} (${gpio.name}) using polling...`);

  // Return handle for cleanup
  return {
    stop() {
      clearInterval(pollInterval);
    }
  };
}

function stopControl(gpioNum) {
  spawn('gpioset', ['gpiochip0', `${gpioNum}=1`]);
}

module.exports = { readGpioState, setGpioOutput, startMonitoring, stopControl };
