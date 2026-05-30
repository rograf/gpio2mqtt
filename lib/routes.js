const express = require('express');
const fs = require('fs');
const path = require('path');
const { config, saveConfigToFile } = require('./config');
const { connectMqtt, isMqttConnected } = require('./mqtt');
const gpio = require('./gpio');
const { uartSseClients, uartHistory, restartUartScheduler } = require('./uart');
const auth = require('./auth');

let shutdownFn = null;

function setShutdownFn(fn) { shutdownFn = fn; }

function setupRoutes(app) {
  // Layout globals
  app.use((req, res, next) => {
    // Check auth state without enforcing (for layout loggedIn flag)
    const accessToken = req.cookies && req.cookies.accessToken;
    res.locals.loggedIn = !!(accessToken && auth.verifyAccessToken(accessToken));
    res.locals.mqttConnected = isMqttConnected();
    res.locals.accessTokenDays = config.auth.accessTokenDays;

    const lang = config.language || 'en';
    res.locals.currentLanguage = lang;
    try {
      const translationsPath = path.join(__dirname, '..', 'assets', 'i18n', `${lang}.json`);
      res.locals.t = JSON.parse(fs.readFileSync(translationsPath, 'utf8'));
    } catch (err) {
      console.error(`Error loading translations for ${lang}:`, err.message);
      try {
        const translationsPath = path.join(__dirname, '..', 'assets', 'i18n', 'en.json');
        res.locals.t = JSON.parse(fs.readFileSync(translationsPath, 'utf8'));
        res.locals.currentLanguage = 'en';
      } catch (fallbackErr) {
        res.locals.t = {};
      }
    }

    // Flash messages from query params
    if (req.query.message) res.locals.message = req.query.message;
    if (req.query.error) res.locals.error = req.query.error;

    next();
  });

  // --- Auth routes ---

  app.get('/', (req, res) => {
    const accessToken = req.cookies && req.cookies.accessToken;
    const loggedIn = accessToken && auth.verifyAccessToken(accessToken);
    res.redirect(loggedIn ? '/dashboard' : '/login');
  });

  app.get('/login', (req, res) => {
    const accessToken = req.cookies && req.cookies.accessToken;
    if (accessToken && auth.verifyAccessToken(accessToken)) {
      return res.redirect('/dashboard');
    }
    res.render('login', { error: req.query.error || null });
  });

  app.post('/login', (req, res) => {
    const ip = req.ip;
    if (auth.isBlocked(ip)) {
      return res.render('login', { error: 'Too many failed attempts. Try again later.' });
    }
    if (req.body.password === config.auth.password) {
      const accessToken = auth.generateAccessToken();
      auth.setAuthCookie(res, accessToken);
      res.redirect('/dashboard');
    } else {
      auth.recordFailedAttempt(ip);
      res.render('login', { error: 'Nieprawidłowe hasło' });
    }
  });

  app.post('/logout', (req, res) => {
    auth.clearAuthCookie(res);
    res.redirect('/login');
  });

  // --- Dashboard ---

  app.get('/dashboard', auth.requireAuth, (req, res) => {
    res.render('dashboard', {
      mqttConnected: isMqttConnected(),
      topic: config.mqtt.topic,
      uartConfigured: !!(config.uart && config.uart.intervalMinutes && Number(config.uart.intervalMinutes) > 0),
      uartName: (config.uart && config.uart.name) ? config.uart.name : 'UART',
    });
  });

  // --- Config ---

  app.get('/config', auth.requireAuth, (req, res) => {
    let uartScript = '';
    try {
      uartScript = fs.readFileSync(path.join(__dirname, '..', 'uart.js'), 'utf8');
    } catch (err) {
      uartScript = '';
    }

    const uartInterval = Number(config.uart?.intervalMinutes || 0);
    const uartScriptEnabled = Number.isFinite(uartInterval) && uartInterval > 0;

    res.render('config', {
      mqttConnected: isMqttConnected(),
      mqtt: {
        broker: config.mqtt?.broker || '',
        topic: config.mqtt?.topic || '',
        username: config.mqtt?.username || '',
        password: config.mqtt?.password || '',
      },
      uart: {
        name: config.uart?.name || '',
        intervalMinutes: config.uart?.intervalMinutes || '',
        script: uartScript,
        scriptEnabled: uartScriptEnabled,
      },
    });
  });

  app.post('/config', auth.requireAuth, (req, res) => {
    try {
      const section = (req.body.section || '').toString();

      if (section === 'mqtt' || section === 'all' || section === '') {
        config.mqtt = config.mqtt || {};
        const newBroker = (req.body.mqttBroker || '').trim();
        const newTopic = (req.body.mqttTopic || '').trim();
        const newUsername = (req.body.mqttUsername || '').trim();
        const newPassword = (req.body.mqttPassword || '').toString();

        if (!newBroker) {
          return res.redirect('/config?error=' + encodeURIComponent('Broker MQTT jest wymagany'));
        }
        if (!newTopic) {
          return res.redirect('/config?error=' + encodeURIComponent('Topic MQTT jest wymagany'));
        }

        const mqttChanged =
          config.mqtt.broker !== newBroker ||
          config.mqtt.username !== newUsername ||
          config.mqtt.password !== newPassword;

        const topicChanged = config.mqtt.topic !== newTopic;

        config.mqtt.broker = newBroker;
        config.mqtt.topic = newTopic;
        config.mqtt.username = newUsername;
        config.mqtt.password = newPassword;

        saveConfigToFile();

        if (mqttChanged) {
          connectMqtt();
        } else if (topicChanged && isMqttConnected()) {
          gpio.subscribeControlTopics();
        }

        return res.redirect('/config?message=' + encodeURIComponent('Zapisano MQTT'));
      }

      if (section === 'uart') {
        config.uart = config.uart || {};
        config.uart.name = (req.body.uartName || '').trim();

        const intervalRaw = (req.body.uartIntervalMinutes || '').toString().trim();
        const interval = intervalRaw === '' ? 0 : parseInt(intervalRaw);
        if (Number.isNaN(interval) || interval < 0) {
          return res.redirect('/config?error=' + encodeURIComponent('Interval UART musi być liczbą >= 0'));
        }
        config.uart.intervalMinutes = interval;

        if (interval > 0) {
          const script = (req.body.uartScript || '').toString();
          if (!script.trim()) {
            return res.redirect('/config?error=' + encodeURIComponent('Skrypt jest wymagany gdy UART jest włączony'));
          }
          fs.writeFileSync(path.join(__dirname, '..', 'uart.js'), script, 'utf8');
        }

        saveConfigToFile();
        restartUartScheduler();

        return res.redirect('/config?message=' + encodeURIComponent('Zapisano UART'));
      }

      if (section === 'app') {
        const currentPassword = (req.body.currentPassword || '').toString();
        const newPassword = (req.body.newPassword || '').toString();
        const confirmPassword = (req.body.confirmPassword || '').toString();

        if (!currentPassword || !newPassword || !confirmPassword) {
          return res.redirect('/config?error=' + encodeURIComponent('Wszystkie pola są wymagane'));
        }

        if (currentPassword !== config.auth.password) {
          return res.redirect('/config?error=' + encodeURIComponent('Nieprawidłowe obecne hasło'));
        }

        if (newPassword !== confirmPassword) {
          return res.redirect('/config?error=' + encodeURIComponent('Nowe hasła nie są zgodne'));
        }

        if (newPassword.length < 4) {
          return res.redirect('/config?error=' + encodeURIComponent('Nowe hasło musi mieć co najmniej 4 znaki'));
        }

        config.auth.password = newPassword;
        saveConfigToFile();

        return res.redirect('/config?message=' + encodeURIComponent('Hasło zostało zmienione'));
      }

      return res.redirect('/config?error=' + encodeURIComponent('Nieznana sekcja konfiguracji'));
    } catch (err) {
      console.error('Error saving /config:', err.message);
      res.redirect('/config?error=' + encodeURIComponent('Błąd zapisu konfiguracji'));
    }
  });

  // --- i18n API ---

  app.get('/api/languages', (req, res) => {
    try {
      const i18nDir = path.join(__dirname, '..', 'assets', 'i18n');
      const files = fs.readdirSync(i18nDir);
      const languages = files
        .filter(f => f.endsWith('.json'))
        .map(f => f.replace('.json', ''));
      res.json(languages);
    } catch (err) {
      console.error('Error reading languages:', err.message);
      res.json(['en']);
    }
  });

  app.post('/api/language', auth.requireAuthApi, express.json(), (req, res) => {
    try {
      const lang = (req.body.language || '').toString().trim();
      if (!lang) {
        return res.status(400).json({ success: false, error: 'Language is required' });
      }

      const langFile = path.join(__dirname, '..', 'assets', 'i18n', `${lang}.json`);
      if (!fs.existsSync(langFile)) {
        return res.status(400).json({ success: false, error: 'Language not found' });
      }

      config.language = lang;
      saveConfigToFile();

      res.json({ success: true });
    } catch (err) {
      console.error('Error changing language:', err.message);
      res.status(500).json({ success: false, error: 'Server error' });
    }
  });

  // --- GPIO API ---

  app.get('/api/gpio', auth.requireAuthApi, (req, res) => {
    const gpioList = [];

    for (let i = 0; i <= 27; i++) {
      const monitorConfig = config.gpio.monitor.find(g => g.gpio === i);
      const controlConfig = config.gpio.control.find(g => g.gpio === i);

      const gpioData = {
        gpio: i,
        name: monitorConfig?.name || controlConfig?.name || null,
        type: monitorConfig ? 'monitor' : controlConfig ? 'control' : null,
        state: null
      };

      if (monitorConfig) {
        const gpioId = `GPIO${i}`;
        const state = gpio.gpioStates[gpioId];
        gpioData.state = state?.lastConnected === true;
      }

      if (controlConfig) {
        const gpioId = `GPIO${i}`;
        const state = gpio.gpioOutputStates[gpioId];
        gpioData.state = state?.power ? 'on' : 'off';
      }

      gpioList.push(gpioData);
    }

    res.json(gpioList);
  });

  app.post('/api/gpio/configure', auth.requireAuthApi, express.json(), (req, res) => {
    const { gpio: gpioNum, name, type } = req.body;

    if (typeof gpioNum !== 'number' || gpioNum < 0 || gpioNum > 27) {
      return res.status(400).json({ error: 'Invalid GPIO number' });
    }

    if (type && !['monitor', 'control', null].includes(type)) {
      return res.status(400).json({ error: 'Invalid type' });
    }

    const gpioId = `GPIO${gpioNum}`;

    gpio.stopMonitoringPin(gpioId);
    gpio.stopControlPin(gpioId, gpioNum);

    config.gpio.monitor = (config.gpio.monitor || []).filter(g => g.gpio !== gpioNum);
    config.gpio.control = (config.gpio.control || []).filter(g => g.gpio !== gpioNum);

    if (type === 'monitor') {
      const monitorConfig = { gpio: gpioNum, name: name || `GPIO${gpioNum}` };
      config.gpio.monitor.push(monitorConfig);
    } else if (type === 'control') {
      config.gpio.control.push({ gpio: gpioNum, name: name || `GPIO${gpioNum}` });
    }

    try {
      saveConfigToFile();
      gpio.restartMonitoring();
      if (isMqttConnected()) gpio.subscribeControlTopics();
    } catch (err) {
      console.error('Error saving config:', err.message);
      return res.status(500).json({ error: 'Błąd zapisu konfiguracji' });
    }

    res.json({ success: true });
  });

  app.post('/api/gpio/control', auth.requireAuthApi, express.json(), (req, res) => {
    const { gpio: gpioNum, power, duration = 0 } = req.body;

    const gpioConfig = config.gpio.control.find(g => g.gpio === gpioNum);
    if (!gpioConfig) {
      return res.status(404).json({ error: 'GPIO not configured for control' });
    }

    gpio.controlRelay(gpioConfig, power, duration);
    res.json({ success: true });
  });

  app.get('/api/gpio/:id/history', auth.requireAuthApi, (req, res) => {
    const key = `GPIO${req.params.id}`;
    res.json(gpio.gpioHistory[key] || []);
  });

  app.get('/api/uart/history', auth.requireAuthApi, (req, res) => {
    res.json(uartHistory || []);
  });

  // --- SSE streams ---

  app.get('/api/gpio/stream', auth.requireAuth, (req, res) => {
    res.setHeader('Content-Type', 'text/event-stream');
    res.setHeader('Cache-Control', 'no-cache');
    res.setHeader('Connection', 'keep-alive');

    const initialData = [];
    for (let i = 0; i <= 27; i++) {
      const monitorConfig = config.gpio.monitor.find(g => g.gpio === i);
      const controlConfig = config.gpio.control.find(g => g.gpio === i);

      if (monitorConfig) {
        const gpioId = `GPIO${i}`;
        const state = gpio.gpioStates[gpioId];
        initialData.push({ gpio: i, type: 'monitor', state: state?.lastConnected === true });
      }

      if (controlConfig) {
        const gpioId = `GPIO${i}`;
        const state = gpio.gpioOutputStates[gpioId];
        initialData.push({ gpio: i, type: 'control', state: state?.power ? 'on' : 'off' });
      }
    }

    res.write(`data: ${JSON.stringify({ type: 'init', data: initialData })}\n\n`);

    gpio.sseClients.push(res);

    const heartbeat = setInterval(() => {
      res.write(': heartbeat\n\n');
    }, 30000);

    req.on('close', () => {
      clearInterval(heartbeat);
      const index = gpio.sseClients.indexOf(res);
      if (index !== -1) gpio.sseClients.splice(index, 1);
    });
  });

  app.get('/api/uart/stream', auth.requireAuth, (req, res) => {
    res.setHeader('Content-Type', 'text/event-stream');
    res.setHeader('Cache-Control', 'no-cache');
    res.setHeader('Connection', 'keep-alive');

    uartSseClients.push(res);

    const heartbeat = setInterval(() => {
      res.write(': heartbeat\n\n');
    }, 30000);

    req.on('close', () => {
      clearInterval(heartbeat);
      const idx = uartSseClients.indexOf(res);
      if (idx !== -1) uartSseClients.splice(idx, 1);
    });
  });

  // --- Restart ---

  app.post('/restart', auth.requireAuth, (req, res) => {
    res.status(200).send('Restarting...');
    setTimeout(() => {
      if (shutdownFn) shutdownFn('restart request');
      else process.exit(0);
    }, 300);
  });
}

module.exports = { setupRoutes, setShutdownFn };
