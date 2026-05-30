const path = require('path');
const { config } = require('./config');
const { mqttPublish, isMqttConnected } = require('./mqtt');

const uartSseClients = [];
const uartHistory = [];
let uartIntervalHandle = null;

function addUartHistory(payload) {
  const entry = { payload, date: new Date().toLocaleString('pl-PL') };
  uartHistory.unshift(entry);
  if (uartHistory.length > 100) uartHistory.pop();
  return entry;
}

function broadcastUartHistory(historyEntry) {
  const data = JSON.stringify({ type: 'uart', history: historyEntry, timestamp: new Date().toISOString() });
  uartSseClients.forEach(client => {
    client.write(`data: ${data}\n\n`);
  });
}

async function runUartOnce() {
  try {
    const uartPath = path.join(__dirname, '..', 'uart.js');
    delete require.cache[require.resolve(uartPath)];
    const runUart = require(uartPath);

    if (typeof runUart !== 'function') {
      const payloadObj = { error: 'uart.js musi exportować funkcję (module.exports = function ...)' };
      const entry = addUartHistory(JSON.stringify(payloadObj));
      broadcastUartHistory(entry);

      if (isMqttConnected()) {
        const topic = `${config.mqtt.topic}/uart`;
        mqttPublish(topic, JSON.stringify(payloadObj), (err) => {
          if (err) console.error(`Error publishing UART error to ${topic}:`, err.message);
        });
      }
      return;
    }

    const result = await Promise.resolve(runUart());
    const entry = addUartHistory(JSON.stringify(result));
    broadcastUartHistory(entry);

    if (isMqttConnected()) {
      const topic = `${config.mqtt.topic}/uart`;
      mqttPublish(topic, JSON.stringify(result), (err) => {
        if (err) {
          console.error(`Error publishing UART to ${topic}:`, err.message);
        } else {
          console.log(`Published to ${topic}: ${JSON.stringify(result)}`);
        }
      });
    }
  } catch (err) {
    const payloadObj = { error: err.message };
    const entry = addUartHistory(JSON.stringify(payloadObj));
    broadcastUartHistory(entry);

    if (isMqttConnected()) {
      const topic = `${config.mqtt.topic}/uart`;
      mqttPublish(topic, JSON.stringify(payloadObj), (e) => {
        if (e) console.error(`Error publishing UART error to ${topic}:`, e.message);
      });
    }
  }
}

function publishLatestUartState() {
  if (!isMqttConnected()) return;

  const latest = uartHistory[0];
  if (!latest) {
    console.log('No UART state available to publish');
    return;
  }

  const topic = `${config.mqtt.topic}/uart`;
  mqttPublish(topic, latest.payload, (err) => {
    if (err) {
      console.error(`Error publishing latest UART state to ${topic}:`, err.message);
    } else {
      console.log(`Published latest UART state to ${topic}: ${latest.payload}`);
    }
  });
}

function restartUartScheduler() {
  if (uartIntervalHandle) {
    clearInterval(uartIntervalHandle);
    uartIntervalHandle = null;
  }

  const intervalMinutes = config.uart && Number.isFinite(config.uart.intervalMinutes)
    ? config.uart.intervalMinutes
    : parseInt(config.uart?.intervalMinutes);

  if (!intervalMinutes || intervalMinutes <= 0) return;

  const ms = intervalMinutes * 60 * 1000;
  uartIntervalHandle = setInterval(() => {
    runUartOnce();
  }, ms);

  runUartOnce();
}

function cleanup() {
  if (uartIntervalHandle) {
    clearInterval(uartIntervalHandle);
    uartIntervalHandle = null;
  }
}

module.exports = {
  uartSseClients,
  uartHistory,
  publishLatestUartState,
  restartUartScheduler,
  cleanup,
};
