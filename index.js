const express = require('express');
const { engine } = require('express-handlebars');
const cookieParser = require('cookie-parser');
const path = require('path');

const { connectMqtt, disconnectMqtt, setOnConnect, setOnMessage } = require('./lib/mqtt');
const gpio = require('./lib/gpio');
const uart = require('./lib/uart');
const { setupRoutes, setShutdownFn } = require('./lib/routes');

const { config } = require('./lib/config');

const app = express();
const PORT = config.port || 3000;

// Handlebars
app.engine('hbs', engine({
  extname: '.hbs',
  helpers: {
    json: (context) => JSON.stringify(context ?? ''),
    eq: (a, b) => a === b,
  },
}));
app.set('view engine', 'hbs');
app.set('views', path.join(__dirname, 'views'));

// Static assets
app.use('/assets', express.static(path.join(__dirname, 'assets')));

// Middleware
app.use(express.urlencoded({ extended: false }));
app.use(cookieParser());

// Wire MQTT callbacks to GPIO module
setOnConnect(() => {
  gpio.subscribeControlTopics();
  gpio.publishInitialStates();
});
setOnMessage((topic, message) => {
  gpio.handleMqttMessage(topic, message);
});

// Connect MQTT
connectMqtt();

// Start GPIO monitoring
gpio.initMonitoring();

// Start UART scheduler
uart.restartUartScheduler();

// Setup routes
setupRoutes(app);

// Graceful shutdown
function gracefulShutdown(reason) {
  try {
    console.log(`\nShutting down (${reason})...`);
    gpio.cleanup();
    uart.cleanup();
    disconnectMqtt();
  } catch (err) {
    console.error('Error during shutdown:', err.message);
  } finally {
    process.exit(0);
  }
}

setShutdownFn(gracefulShutdown);

app.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
});

process.on('SIGINT', () => {
  gracefulShutdown('SIGINT');
});
