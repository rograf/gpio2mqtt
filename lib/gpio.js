const { config } = require('./config');
const { mqttPublish, isMqttConnected, mqttSubscribe } = require('./mqtt');
const { execFileSync } = require('child_process');

// Load backend based on config
let backend;

function commandHelpIncludes(command, text) {
  try {
    const output = execFileSync(command, ['--help'], { encoding: 'utf8', stdio: ['ignore', 'pipe', 'pipe'] });
    return output.includes(text);
  } catch (err) {
    const output = `${err.stdout || ''}${err.stderr || ''}`;
    return output.includes(text);
  }
}

function supportsGpiodV2Cli() {
  return commandHelpIncludes('gpioget', '--chip') && commandHelpIncludes('gpiomon', '--chip');
}

const requestedBackend = config.gpioBackend || 'gpiod-v2';
const effectiveBackend = requestedBackend === 'gpiod-v2' && !supportsGpiodV2Cli()
  ? 'gpiod-v1'
  : requestedBackend;

if (requestedBackend === 'gpiod-v2' && effectiveBackend === 'gpiod-v1') {
  console.warn('GPIO backend: requested libgpiod v2, but installed gpiod CLI does not support --chip. Falling back to libgpiod v1.');
}

if (effectiveBackend === 'gpiod-v1') {
  backend = require('./gpio-backend-v1');
  console.log('GPIO backend: libgpiod v1 (gpiomon)');
} else if (effectiveBackend === 'fake') {
  backend = require('./gpio-backend-fake');
  console.log('GPIO backend: fake (no real hardware)');
} else {
  backend = require('./gpio-backend-v2');
  console.log('GPIO backend: libgpiod v2 (interrupt-based)');
}

// Shared state
const gpioStates = {};
const gpioOutputStates = {};
const gpioHistory = {};
const sseClients = [];
let monitorHandle = null;

// --- History ---

function addHistory(gpioNum, payload) {
  const key = `GPIO${gpioNum}`;
  if (!gpioHistory[key]) gpioHistory[key] = [];
  const entry = { payload, date: new Date().toLocaleString('pl-PL') };
  gpioHistory[key].unshift(entry);
  if (gpioHistory[key].length > 100) gpioHistory[key].pop();
  return entry;
}

function addHistoryIfChanged(gpioNum, payload) {
  const key = `GPIO${gpioNum}`;
  const history = gpioHistory[key] || [];
  if (history.length > 0 && history[0].payload === payload) {
    return null;
  }
  return addHistory(gpioNum, payload);
}

// --- SSE ---

function broadcastStateChange(gpio, type, state, historyEntry) {
  const data = JSON.stringify({ gpio, type, state, history: historyEntry, timestamp: new Date().toISOString() });
  sseClients.forEach(client => {
    client.write(`data: ${data}\n\n`);
  });
}

// --- GPIO output ---

function publishGpioState(gpio, power) {
  publishControlState(gpio, power, true, true);
}

function setGpioOutput(gpio, value) {
  const gpioId = `GPIO${gpio.gpio}`;

  return backend.setGpioOutput(gpio.gpio, value, gpioOutputStates).then(power => {
    console.log(`${gpioId} (${gpio.name}) turned ${power ? 'ON' : 'OFF'}`);
    return power;
  });
}

async function controlRelay(gpio, power, duration = 0) {
  let oppositePower;
  let currentPower;

  try {
    if (power === true) {
      await setGpioOutput(gpio, 1);
      currentPower = true;
      oppositePower = false;
      publishGpioState(gpio, true);
    } else if (power === false) {
      await setGpioOutput(gpio, 0);
      currentPower = false;
      oppositePower = true;
      publishGpioState(gpio, false);
    } else if (power === null) {
      const gpioId = `GPIO${gpio.gpio}`;
      const currentState = gpioOutputStates[gpioId];
      if (currentState && currentState.power) {
        await setGpioOutput(gpio, 0);
        currentPower = false;
        oppositePower = true;
      } else {
        await setGpioOutput(gpio, 1);
        currentPower = true;
        oppositePower = false;
      }
      publishGpioState(gpio, currentPower);
    }

    if (duration && duration > 0) {
      console.log(`GPIO${gpio.gpio} (${gpio.name}) will toggle in ${duration}ms`);
      setTimeout(() => {
        controlRelay(gpio, oppositePower, 0);
      }, duration);
    }
  } catch (err) {
    console.error(`Error controlling ${gpio.name}:`, err.message);
  }
}

// --- Monitor edge handler (with debounce) ---

function handleEdge(gpioId, gpio, connected) {
  const state = gpioStates[gpioId];
  state.currentValue = connected;

  if (state.debounceTimer) {
    clearTimeout(state.debounceTimer);
  }

  state.debounceTimer = setTimeout(() => {
    if (state.lastConnected !== connected) {
      state.lastConnected = connected;

      console.log(`${gpioId} (${gpio.name}) ${connected ? 'connected' : 'disconnected'} to ground`);
      publishMonitorState(gpio, connected, true, true);
    }
  }, 500);
}

// --- Init monitoring ---

function initMonitoring() {
  restartMonitoring();
}

function ensureMonitorState(gpio) {
  const gpioId = `GPIO${gpio.gpio}`;

  if (!gpioStates[gpioId]) {
    gpioStates[gpioId] = {
      lastConnected: null,
      debounceTimer: null,
      currentValue: null
    };
  }
}

function stopMonitoringProcess() {
  if (monitorHandle) {
    monitorHandle.stop();
    monitorHandle = null;
  }
}

function restartMonitoring() {
  stopMonitoringProcess();

  const monitors = (config.gpio && config.gpio.monitor) ? config.gpio.monitor : [];
  monitors.forEach(ensureMonitorState);

  if (monitors.length === 0) {
    return;
  }

  monitorHandle = backend.startMonitoringMany(monitors, gpioStates, handleEdge);
}

function startMonitoringPin(gpio) {
  ensureMonitorState(gpio);
  restartMonitoring();
}

// --- MQTT integration ---

function subscribeControlTopics() {
  if (!isMqttConnected()) return;
  const getTopic = `${config.mqtt.topic}/get`;
  mqttSubscribe(getTopic, (err) => {
    if (err) {
      console.error(`Error subscribing to ${getTopic}:`, err.message);
    } else {
      console.log(`Subscribed to ${getTopic} (state request)`);
    }
  });

  if (config.gpio && config.gpio.control) {
    config.gpio.control.forEach(gpio => {
      const topic = `${config.mqtt.topic}/GPIO${gpio.gpio}/set`;
      mqttSubscribe(topic, (err) => {
        if (err) {
          console.error(`Error subscribing to ${topic}:`, err.message);
        } else {
          console.log(`Subscribed to ${topic} (${gpio.name})`);
        }
      });
    });
  }
}

async function publishInitialStates() {
  if (!config.gpio) return;

  for (const gpio of (config.gpio.monitor || [])) {
    publishKnownMonitorState(gpio);
  }

  (config.gpio.control || []).forEach(gpio => {
    const gpioId = `GPIO${gpio.gpio}`;
    const state = gpioOutputStates[gpioId];
    publishControlState(gpio, state?.power === true, true);
  });
}

function publishKnownMonitorState(gpio) {
  const gpioId = `GPIO${gpio.gpio}`;
  const state = gpioStates[gpioId];
  if (!state) return;

  const connected = state.lastConnected !== null && state.lastConnected !== undefined
    ? state.lastConnected
    : state.currentValue;

  if (connected === null || connected === undefined) {
    console.log(`${gpioId} (${gpio.name}) state is not available yet`);
    return;
  }

  publishMonitorState(gpio, connected === true, true);
}

function publishMonitorState(gpio, connected, recordHistory = false, forceHistory = false) {
  const gpioId = `GPIO${gpio.gpio}`;
  const payload = JSON.stringify({ connected });
  const topic = `${config.mqtt.topic}/${gpioId}`;
  let historyEntry;

  if (recordHistory) {
    historyEntry = forceHistory ? addHistory(gpio.gpio, payload) : addHistoryIfChanged(gpio.gpio, payload);
    broadcastStateChange(gpio.gpio, 'monitor', connected, historyEntry);
  }

  mqttPublish(topic, payload, (err) => {
    if (err) {
      console.error(`Error publishing ${gpioId} status:`, err.message);
    } else {
      console.log(`Published to ${topic}: ${payload}`);
    }
  });
}

function publishControlState(gpio, power, recordHistory = false, forceHistory = false) {
  const gpioId = `GPIO${gpio.gpio}`;
  const payload = JSON.stringify({ power });
  const topic = `${config.mqtt.topic}/${gpioId}`;
  let historyEntry;

  if (recordHistory) {
    historyEntry = forceHistory ? addHistory(gpio.gpio, payload) : addHistoryIfChanged(gpio.gpio, payload);
    broadcastStateChange(gpio.gpio, 'control', power ? 'on' : 'off', historyEntry);
  }

  mqttPublish(topic, payload, (err) => {
    if (err) {
      console.error(`Error publishing GPIO${gpio.gpio} (${gpio.name}) state:`, err.message);
    } else {
      console.log(`Published to ${topic}: ${payload}`);
    }
  });
}

function handleMqttMessage(topic, message) {
  try {
    if (topic.endsWith('/set')) {
      const gpioTopic = topic.split('/')[1];
      const gpioNum = parseInt(gpioTopic.replace('GPIO', ''));
      const data = JSON.parse(message.toString());
      const { power, duration = 0 } = data;

      const gpio = config.gpio.control.find(g => g.gpio === gpioNum);
      if (!gpio) {
        console.log(`${gpioTopic} not found in control config`);
        return;
      }

      controlRelay(gpio, power, duration);
    }
  } catch (err) {
    console.error('Error processing MQTT message:', err.message);
  }
}

// --- Configure GPIO (add/remove pin) ---

function stopMonitoringPin(gpioId) {
  stopMonitoringProcess();

  if (gpioStates[gpioId] && gpioStates[gpioId].debounceTimer) {
    clearTimeout(gpioStates[gpioId].debounceTimer);
  }
  delete gpioStates[gpioId];
  console.log(`Stopped monitoring ${gpioId}`);
}

function stopControlPin(gpioId, gpioNum) {
  if (gpioOutputStates[gpioId]) {
    if (gpioOutputStates[gpioId].process) {
      gpioOutputStates[gpioId].process.kill();
    }
    delete gpioOutputStates[gpioId];
    backend.stopControl(gpioNum);
    console.log(`Stopped control ${gpioId}, pin set to OFF`);
  }
}

// --- Cleanup ---

function cleanup() {
  stopMonitoringProcess();

  Object.values(gpioOutputStates).forEach(state => {
    if (state.process) state.process.kill();
  });
}

module.exports = {
  gpioStates,
  gpioOutputStates,
  gpioHistory,
  sseClients,
  initMonitoring,
  startMonitoringPin,
  restartMonitoring,
  controlRelay,
  subscribeControlTopics,
  publishInitialStates,
  handleMqttMessage,
  addHistory,
  broadcastStateChange,
  stopMonitoringPin,
  stopControlPin,
  cleanup,
};
