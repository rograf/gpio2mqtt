// GPIO backend for libgpiod v2 (newer systems)
// Uses gpiomon for interrupt-based monitoring (zero CPU when idle)
// Command syntax: gpioget --chip gpiochip0 --bias=pull-up NUM
//                 gpioset --chip gpiochip0 NUM=VALUE

const { spawn, exec } = require('child_process');

function readGpioState(gpioNum) {
  return new Promise((resolve, reject) => {
    exec(`gpioget --chip gpiochip0 --bias=pull-up ${gpioNum}`, (error, stdout, stderr) => {
      if (error) {
        console.error(`gpioget error for GPIO${gpioNum}:`, error.message);
        if (stderr) console.error(`gpioget stderr:`, stderr);
        reject(error);
        return;
      }
      // libgpiod v2.x returns format: "21"=active or "21"=inactive
      const output = stdout.trim();
      const isActive = output.includes('=active');
      resolve(!isActive); // inactive = connected, active = disconnected
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
      const proc = spawn('gpioset', ['--chip', 'gpiochip0', `${gpioNum}=${pinValue}`]);

      proc.stderr.on('data', (data) => {
        console.error(`gpioset stderr: ${data}`);
      });

      proc.on('error', (err) => {
        console.error(`Error setting GPIO${gpioNum}:`, err.message);
        reject(err);
      });

      proc.on('exit', (code) => {
        if (code !== null && code !== 0) {
          console.error(`gpioset exited with code ${code}`);
        }
      });

      const power = value === 1;
      gpioOutputStates[gpioId] = { power, process: proc };
      resolve(power);
    }
  });
}

function startMonitoring(gpio, gpioStates, onEdge) {
  const gpioId = `GPIO${gpio.gpio}`;

  // Read initial state
  readGpioState(gpio.gpio).then(initialState => {
    gpioStates[gpioId].currentValue = initialState;
    gpioStates[gpioId].lastConnected = initialState;
    console.log(`${gpioId} (${gpio.name}) initial state: ${initialState ? 'connected' : 'disconnected'}`);
  }).catch(err => {
    console.error(`Error reading initial state of ${gpioId}:`, err.message);
  });

  // Start gpiomon process (uses hardware interrupts - zero CPU when idle)
  const gpiomonProcess = spawn('gpiomon', [
    '--chip', 'gpiochip0',
    '--bias=pull-up',
    '--edges=both',
    `${gpio.gpio}`
  ]);

  let lineBuffer = '';

  gpiomonProcess.stdout.on('data', (data) => {
    lineBuffer += data.toString();
    const lines = lineBuffer.split('\n');
    lineBuffer = lines.pop() || '';

    lines.forEach((line) => {
      if (!line.trim()) return;

      let connected;
      if (line.includes('falling')) {
        connected = true;
      } else if (line.includes('rising')) {
        connected = false;
      } else {
        return;
      }

      onEdge(gpioId, gpio, connected);
    });
  });

  gpiomonProcess.stderr.on('data', (data) => {
    console.error(`gpiomon stderr for ${gpioId}:`, data.toString());
  });

  gpiomonProcess.on('error', (err) => {
    console.error(`Error starting gpiomon for ${gpioId}:`, err.message);
  });

  gpiomonProcess.on('exit', (code) => {
    if (code !== null && code !== 0) {
      console.error(`gpiomon for ${gpioId} exited with code ${code}`);
    }
  });

  console.log(`Started monitoring ${gpioId} (${gpio.name}) using gpiomon (interrupt-based)...`);

  // Return handle for cleanup
  return {
    stop() {
      gpiomonProcess.kill();
    }
  };
}

function stopControl(gpioNum) {
  spawn('gpioset', ['--chip', 'gpiochip0', `${gpioNum}=1`]);
}

module.exports = { readGpioState, setGpioOutput, startMonitoring, stopControl };
