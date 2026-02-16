const { config } = require('./config');
const { mqttPublish, isMqttConnected, mqttSubscribe } = require('./mqtt');

// Load backend based on config
let backend;
if (config.gpioBackend === 'gpiod-v1') {
  backend = require('./gpio-backend-v1');
  console.log('GPIO backend: libgpiod v1 (polling)');
} else if (config.gpioBackend === 'fake') {
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
const monitorHandles = {};

// --- History ---

function addHistory(gpioNum, payload) {
  const key = `GPIO${gpioNum}`;
  if (!gpioHistory[key]) gpioHistory[key] = [];
  const entry = { payload, date: new Date().toLocaleString('pl-PL') };
  gpioHistory[key].unshift(entry);
  if (gpioHistory[key].length > 100) gpioHistory[key].pop();
  return entry;
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
  const topic = `${config.mqtt.topic}/GPIO${gpio.gpio}`;
  const payload = JSON.stringify({ power });

  const historyEntry = addHistory(gpio.gpio, payload);
  broadcastStateChange(gpio.gpio, 'control', power ? 'on' : 'off', historyEntry);

  mqttPublish(topic, payload, (err) => {
    if (err) {
      console.error(`Error publishing GPIO${gpio.gpio} (${gpio.name}) state:`, err.message);
    } else {
      console.log(`Published to ${topic}: ${payload}`);
    }
  });
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

      const payload = JSON.stringify({ connected });
      const historyEntry = addHistory(gpio.gpio, payload);
      broadcastStateChange(gpio.gpio, 'monitor', connected, historyEntry);

      if (isMqttConnected()) {
        const topic = `${config.mqtt.topic}/${gpioId}`;
        mqttPublish(topic, payload, (err) => {
          if (err) {
            console.error(`Error publishing ${gpioId} status:`, err.message);
          } else {
            console.log(`Published to ${topic}: ${payload}`);
          }
        });
      }
    }
  }, 500);
}

// --- Init monitoring ---

function initMonitoring() {
  if (!config.gpio || !config.gpio.monitor) return;

  config.gpio.monitor.forEach(gpio => {
    const gpioId = `GPIO${gpio.gpio}`;

    gpioStates[gpioId] = {
      lastConnected: null,
      debounceTimer: null,
      currentValue: null
    };

    const handle = backend.startMonitoring(gpio, gpioStates, handleEdge);
    monitorHandles[gpioId] = handle;
  });
}

// --- MQTT integration ---

function subscribeControlTopics() {
  if (!isMqttConnected()) return;
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

function publishInitialStates() {
  if (!config.gpio || !config.gpio.monitor) return;
  config.gpio.monitor.forEach(gpio => {
    const gpioId = `GPIO${gpio.gpio}`;
    if (gpioStates[gpioId] && gpioStates[gpioId].lastConnected !== null) {
      const payload = JSON.stringify({ connected: gpioStates[gpioId].lastConnected });
      const topic = `${config.mqtt.topic}/${gpioId}`;
      mqttPublish(topic, payload, (err) => {
        if (!err) {
          console.log(`Published initial state to ${topic}: ${payload}`);
        }
      });
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
  if (monitorHandles[gpioId]) {
    monitorHandles[gpioId].stop();
    delete monitorHandles[gpioId];
  }

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
  Object.keys(monitorHandles).forEach(gpioId => {
    monitorHandles[gpioId].stop();
  });

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
