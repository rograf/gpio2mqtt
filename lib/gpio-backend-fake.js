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

function startMonitoringMany(gpios, gpioStates, onEdge) {
  const timers = [];

  gpios.forEach(gpio => {
    const gpioId = `GPIO${gpio.gpio}`;
    simulated[gpioId] = false;
    onEdge(gpioId, gpio, false);

    function scheduleToggle() {
      const delay = Math.floor(Math.random() * (10000 - 2000 + 1)) + 2000;
      const timer = setTimeout(() => {
        simulated[gpioId] = !simulated[gpioId];
        console.log(`[FAKE] ${gpioId} (${gpio.name}) -> ${simulated[gpioId] ? 'connected' : 'disconnected'}`);
        onEdge(gpioId, gpio, simulated[gpioId]);
        scheduleToggle();
      }, delay);
      timers.push(timer);
    }

    scheduleToggle();
  });

  console.log(`[FAKE] Started monitoring ${gpios.length} GPIO(s) with random simulator...`);

  return {
    stop() {
      timers.forEach(timer => clearTimeout(timer));
    }
  };
}

function stopControl(gpioNum) {
  // no-op in fake mode
}

module.exports = { readGpioState, setGpioOutput, startMonitoringMany, stopControl };
