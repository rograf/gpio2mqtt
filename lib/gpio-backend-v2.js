// GPIO backend for libgpiod v2 (newer systems)
// Uses a single gpiomon process for interrupt-based monitoring (zero CPU when idle)
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

function parseGpiomonLine(line) {
  const normalized = line.toLowerCase();
  const isFalling = normalized.includes('falling');
  const isRising = normalized.includes('rising');

  if (!isFalling && !isRising) return null;

  const offsetMatch =
    normalized.match(/offset:\s*(\d+)/) ||
    normalized.match(/\boffset\s+(\d+)/) ||
    normalized.match(/\bline\s+(\d+)/);

  if (!offsetMatch) return null;

  return {
    gpioNum: parseInt(offsetMatch[1], 10),
    connected: isFalling
  };
}

function startMonitoringMany(gpios, gpioStates, onEdge) {
  const monitoredGpios = gpios.filter(gpio => Number.isInteger(gpio.gpio));
  const gpioByNumber = new Map(monitoredGpios.map(gpio => [gpio.gpio, gpio]));

  monitoredGpios.forEach(gpio => {
    const gpioId = `GPIO${gpio.gpio}`;

    readGpioState(gpio.gpio).then(initialState => {
      console.log(`${gpioId} (${gpio.name}) initial state: ${initialState ? 'connected' : 'disconnected'}`);
      onEdge(gpioId, gpio, initialState);
    }).catch(err => {
      console.error(`Error reading initial state of ${gpioId}:`, err.message);
    });
  });

  if (monitoredGpios.length === 0) {
    return { stop() {} };
  }

  // Start one gpiomon process (uses hardware interrupts - zero CPU when idle)
  const gpiomonProcess = spawn('gpiomon', [
    '--chip', 'gpiochip0',
    '--bias=pull-up',
    '--edges=both',
    ...monitoredGpios.map(gpio => `${gpio.gpio}`)
  ]);

  let lineBuffer = '';

  gpiomonProcess.stdout.on('data', (data) => {
    lineBuffer += data.toString();
    const lines = lineBuffer.split('\n');
    lineBuffer = lines.pop() || '';

    lines.forEach((line) => {
      let event = parseGpiomonLine(line);
      if (!event && monitoredGpios.length === 1) {
        const normalized = line.toLowerCase();
        if (normalized.includes('falling') || normalized.includes('rising')) {
          event = {
            gpioNum: monitoredGpios[0].gpio,
            connected: normalized.includes('falling')
          };
        }
      }
      if (!event) return;

      const gpio = gpioByNumber.get(event.gpioNum);
      if (!gpio) return;

      onEdge(`GPIO${event.gpioNum}`, gpio, event.connected);
    });
  });

  gpiomonProcess.stderr.on('data', (data) => {
    console.error('gpiomon stderr:', data.toString());
  });

  gpiomonProcess.on('error', (err) => {
    console.error('Error starting gpiomon:', err.message);
  });

  gpiomonProcess.on('exit', (code) => {
    if (code !== null && code !== 0) {
      console.error(`gpiomon exited with code ${code}`);
    }
  });

  console.log(`Started monitoring ${monitoredGpios.length} GPIO(s) using one gpiomon process (interrupt-based)...`);

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

module.exports = { readGpioState, setGpioOutput, startMonitoringMany, stopControl };
