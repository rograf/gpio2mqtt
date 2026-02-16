const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

const CONFIG_PATH = path.join(__dirname, '..', 'config.json');

const config = require(CONFIG_PATH);

// Initialize defaults
if (!config.language) {
  config.language = 'en';
}
if (!config.uart) config.uart = {};
if (!config.gpioBackend) config.gpioBackend = 'gpiod-v2';
if (!config.port) config.port = 3521;
if (!config.auth) config.auth = {};
if (!config.auth.password) config.auth.password = 'admin';
if (!config.auth.accessTokenDays) config.auth.accessTokenDays = 14;
if (!config.auth.tokenRefreshDays) config.auth.tokenRefreshDays = 1;
if (!config.auth.maxFailedAttempts) config.auth.maxFailedAttempts = 10;
if (!config.auth.failedAttemptWindowSeconds) config.auth.failedAttemptWindowSeconds = 60;
if (!config.auth.blockDurationMinutes) config.auth.blockDurationMinutes = 10;
if (!config.auth.secret) {
  config.auth.secret = crypto.randomBytes(32).toString('hex');
  // Save immediately so secret persists across restarts
  fs.writeFileSync(CONFIG_PATH, JSON.stringify(config, null, 2));
}

function saveConfigToFile() {
  fs.writeFileSync(CONFIG_PATH, JSON.stringify(config, null, 2));
}

module.exports = { config, saveConfigToFile };
