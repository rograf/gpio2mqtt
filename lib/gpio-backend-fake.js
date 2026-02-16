// GPIO backend for development/testing (no real hardware)
// Simulates GPIO states with random toggling

const simulated = {};

function readGpioState(gpioNum) {
  const gpioId = `GPIO${gpioNum}`;
  if (simulated[gpioId] === undefined) {
    simulated[gpioId] = false;
  }
  return Promise.resolve(simulated[gpioId]);
}

function setGpioOutput(gpioNum, value, gpioOutputStates) {
  const gpioId = `GPIO${gpioNum}`;
  const power = value === 1;
  gpioOutputStates[gpioId] = { power, process: null };
  console.log(`[FAKE] ${gpioId} turned ${power ? 'ON' : 'OFF'}`);
  return Promise.resolve(power);
}

function startMonitoring(gpio, gpioStates, onEdge) {
  const gpioId = `GPIO${gpio.gpio}`;
  simulated[gpioId] = false;

  // Simulate random state changes every 2-10s
  let timer;
  function scheduleToggle() {
    const delay = Math.floor(Math.random() * (10000 - 2000 + 1)) + 2000;
    timer = setTimeout(() => {
      simulated[gpioId] = !simulated[gpioId];
      console.log(`[FAKE] ${gpioId} (${gpio.name}) -> ${simulated[gpioId] ? 'connected' : 'disconnected'}`);
      onEdge(gpioId, gpio, simulated[gpioId]);
      scheduleToggle();
    }, delay);
  }
  scheduleToggle();

  console.log(`[FAKE] Started monitoring ${gpioId} (${gpio.name}) with random simulator...`);

  return {
    stop() {
      clearTimeout(timer);
    }
  };
}

function stopControl(gpioNum) {
  // no-op in fake mode
}

module.exports = { readGpioState, setGpioOutput, startMonitoring, stopControl };
