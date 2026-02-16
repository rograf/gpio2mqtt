const mqtt = require('mqtt');
const { config } = require('./config');

let mqttClient = null;
let mqttConnected = false;
let mqttRetryCount = 0;
let mqttRetryTimer = null;
const MQTT_MAX_RETRIES = 10;
const MQTT_RETRY_INTERVAL = 5000;
const MQTT_HOURLY_INTERVAL = 60 * 60 * 1000;

// Callbacks set by other modules
let onConnect = null;
let onMessage = null;

function getMqttOptions() {
  const mqttOptions = {};
  if (config.mqtt && config.mqtt.username) {
    mqttOptions.username = config.mqtt.username;
    mqttOptions.password = config.mqtt.password;
  }
  return mqttOptions;
}

function mqttPublish(topic, payload, callback) {
  if (!mqttClient || !mqttConnected) {
    if (callback) callback(new Error('MQTT not connected'));
    return;
  }
  mqttClient.publish(topic, payload, callback);
}

function mqttSubscribe(topic, callback) {
  if (!mqttClient || !mqttConnected) return;
  mqttClient.subscribe(topic, callback);
}

function stopMqttRetry() {
  if (mqttRetryTimer) {
    clearTimeout(mqttRetryTimer);
    mqttRetryTimer = null;
  }
}

function scheduleMqttRetry() {
  stopMqttRetry();
  mqttRetryCount++;

  if (mqttRetryCount <= MQTT_MAX_RETRIES) {
    console.log(`MQTT retry ${mqttRetryCount}/${MQTT_MAX_RETRIES} in ${MQTT_RETRY_INTERVAL / 1000}s...`);
    mqttRetryTimer = setTimeout(() => connectMqtt(), MQTT_RETRY_INTERVAL);
  } else if (mqttRetryCount === MQTT_MAX_RETRIES + 1) {
    console.error(`MQTT: ${MQTT_MAX_RETRIES} attempts failed. Retrying every hour.`);
    mqttRetryTimer = setTimeout(() => connectMqtt(), MQTT_HOURLY_INTERVAL);
  } else {
    mqttRetryTimer = setTimeout(() => connectMqtt(), MQTT_HOURLY_INTERVAL);
  }
}

function connectMqtt() {
  stopMqttRetry();

  if (!config.mqtt || !config.mqtt.broker) {
    console.log('MQTT: no broker configured, skipping connection');
    return;
  }

  try {
    if (mqttClient) {
      mqttClient.removeAllListeners();
      mqttClient.end(true);
      mqttClient = null;
    }

    mqttConnected = false;
    mqttClient = mqtt.connect(config.mqtt.broker, {
      ...getMqttOptions(),
      reconnectPeriod: 0,
      connectTimeout: 10000,
    });

    mqttClient.on('connect', () => {
      mqttConnected = true;
      mqttRetryCount = 0;
      stopMqttRetry();
      console.log('MQTT connected to', config.mqtt.broker);
      if (onConnect) onConnect();
    });

    mqttClient.on('error', (err) => {
      console.error('MQTT error:', err.message);
    });

    mqttClient.on('close', () => {
      const wasConnected = mqttConnected;
      mqttConnected = false;
      if (wasConnected) {
        mqttRetryCount = 0;
        scheduleMqttRetry();
      }
    });

    mqttClient.on('offline', () => {
      if (!mqttRetryTimer) {
        scheduleMqttRetry();
      }
    });

    mqttClient.on('message', (topic, message) => {
      if (onMessage) onMessage(topic, message);
    });
  } catch (err) {
    console.error('Error connecting MQTT:', err.message);
    scheduleMqttRetry();
  }
}

function disconnectMqtt() {
  stopMqttRetry();
  if (mqttClient) {
    mqttClient.end(true);
    mqttClient = null;
  }
  mqttConnected = false;
}

function isMqttConnected() {
  return mqttConnected;
}

function setOnConnect(cb) { onConnect = cb; }
function setOnMessage(cb) { onMessage = cb; }

module.exports = {
  connectMqtt,
  disconnectMqtt,
  mqttPublish,
  mqttSubscribe,
  isMqttConnected,
  setOnConnect,
  setOnMessage,
};
