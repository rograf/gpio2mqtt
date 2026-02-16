const jwt = require('jsonwebtoken');
const { config } = require('./config');

// Rate limiting: track failed attempts per IP
const failedAttempts = new Map(); // ip -> { count, firstAttemptAt, blockedUntil }

function isBlocked(ip) {
  const entry = failedAttempts.get(ip);
  if (!entry) return false;
  if (entry.blockedUntil && entry.blockedUntil > Date.now()) return true;
  if (entry.blockedUntil && entry.blockedUntil <= Date.now()) {
    failedAttempts.delete(ip);
    return false;
  }
  return false;
}

function recordFailedAttempt(ip) {
  const now = Date.now();
  const windowMs = config.auth.failedAttemptWindowSeconds * 1000;
  let entry = failedAttempts.get(ip);

  if (!entry || (now - entry.firstAttemptAt) > windowMs) {
    entry = { count: 1, firstAttemptAt: now, blockedUntil: null };
  } else {
    entry.count++;
  }

  if (entry.count >= config.auth.maxFailedAttempts) {
    entry.blockedUntil = now + config.auth.blockDurationMinutes * 60 * 1000;
  }

  failedAttempts.set(ip, entry);
}

function generateAccessToken() {
  return jwt.sign({}, config.auth.secret, {
    expiresIn: config.auth.accessTokenDays * 24 * 60 * 60,
  });
}

function verifyAccessToken(token) {
  try {
    return jwt.verify(token, config.auth.secret);
  } catch {
    return null;
  }
}

const TOKEN_MAX_AGE_MS = () => config.auth.accessTokenDays * 24 * 60 * 60 * 1000;

function setAuthCookie(res, accessToken) {
  res.cookie('accessToken', accessToken, {
    httpOnly: true,
    sameSite: 'strict',
    maxAge: TOKEN_MAX_AGE_MS(),
  });
}

function clearAuthCookie(res) {
  res.clearCookie('accessToken');
}

// Try to authenticate: returns true if valid (may auto-refresh token if older than tokenRefreshDays)
function tryAuth(req, res) {
  const accessToken = req.cookies && req.cookies.accessToken;
  const decoded = accessToken && verifyAccessToken(accessToken);
  if (!decoded) return false;

  // Auto-refresh: if token is older than tokenRefreshDays, issue a new one
  const tokenAgeMs = Date.now() - (decoded.iat * 1000);
  const refreshThresholdMs = config.auth.tokenRefreshDays * 24 * 60 * 60 * 1000;
  if (tokenAgeMs > refreshThresholdMs) {
    const newToken = generateAccessToken();
    setAuthCookie(res, newToken);
  }

  return true;
}

// Middleware for page routes (redirects to /login)
function requireAuth(req, res, next) {
  const ip = req.ip;
  if (isBlocked(ip)) {
    clearAuthCookie(res);
    return res.redirect('/login?error=' + encodeURIComponent('Too many failed attempts. Try again later.'));
  }
  if (tryAuth(req, res)) {
    res.locals.loggedIn = true;
    return next();
  }
  recordFailedAttempt(ip);
  clearAuthCookie(res);
  res.redirect('/login');
}

// Middleware for API routes (returns 401 JSON)
function requireAuthApi(req, res, next) {
  const ip = req.ip;
  if (isBlocked(ip)) {
    clearAuthCookie(res);
    return res.status(429).json({ error: 'Too many failed attempts. Try again later.' });
  }
  if (tryAuth(req, res)) {
    return next();
  }
  recordFailedAttempt(ip);
  clearAuthCookie(res);
  res.status(401).json({ error: 'Unauthorized' });
}

module.exports = {
  generateAccessToken,
  verifyAccessToken,
  setAuthCookie,
  clearAuthCookie,
  requireAuth,
  requireAuthApi,
  isBlocked,
  recordFailedAttempt,
};
