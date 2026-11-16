/* ============================================================================
   SONAR RADAR  ·  display + sound  v2
   IB MYP Design — Unit 2: Interactive display and sensors

   Reads "<angle>,<distance>" lines from the Arduino, draws the sweep, groups
   the echoes into numbered tracks, and plays a submarine sonar ping through
   the laptop speakers.

   The ping never stops. Empty water gets a slow, low ping. As soon as there is
   an object the pings come faster and brighter, and faster again the closer it
   gets. The ping is also panned left or right to match where the object is.

   Needs the Sound library:  Sketch > Import Library > Manage Libraries > "Sound"
   The three ping_*.wav files live in this sketch's data folder.

   PORT: leave PORT_NAME empty and the sketch looks for an Arduino by itself.
   If it picks the wrong one, type the name here. With no board connected the
   sketch runs in DEMO mode with simulated objects.

   KEYS   SPACE pause sweep   1 2 3 sweep speed   R range   S sound   D demo   C clear
   ========================================================================== */

import processing.serial.*;
import processing.sound.*;

/* ---- things you may want to change --------------------------------------- */
String PORT_NAME = "";            // e.g. "/dev/cu.usbmodem1101" or "COM3"
final int   BAUD       = 115200;
final int[] RANGES    = {50, 100, 150, 200};   // R key steps through these
int         maxRange   = 200;     // cm shown on the scope; lower it in a crowded room
final int   CLOSE_CM   = 45;      // inside this range the alarm state starts
final int   BLIP_MIN   = 3400;    // ms an echo glows at the very least
final float MASTER_VOL = 0.8;
final int   VOICES     = 6;       // copies of each ping, so tails can overlap

/* the alarm zone shrinks with the scope so it never fills the screen */
float closeCm() { return min(CLOSE_CM, maxRange * 0.35); }

/* ---- colours ------------------------------------------------------------- */
final color GREEN = #2CFF9B;
final color DIM   = #12935A;
final color FAINT = #0B4E31;
final color AMBER = #FFC857;
final color RED   = #FF5065;
final color INK   = #030D09;
final color PANEL = #061710;
final color WHITE = #E4FFF1;

/* ---- scope geometry ------------------------------------------------------ */
final int CX = 640, CY = 678, R = 500;

/* ---- link ---------------------------------------------------------------- */
Serial  port;
boolean linked = false, demo = false, paused = false, muted = false;
String  portLabel = "";
int     lastLineMs = 0, speed = 2;

/* ---- sweep state --------------------------------------------------------- */
int   angle = 15, sweepDir = 1, dist = -1, stepDeg = 1;
float beam  = 15;                 // eased copy of angle, for a smooth beam
int   sweeps = 0, sweepStartMs = 0, startMs = 0;
float sweepSecs = 0;
float[] profile = new float[181]; // last range seen at every bearing

ArrayList<Blip>   blips    = new ArrayList<Blip>();
ArrayList<Track>  tracks   = new ArrayList<Track>();
ArrayList<String> logLines = new ArrayList<String>();
int nextId = 1;

/* a "run" is a streak of neighbouring echoes at about the same range */
int   runCount = 0, runStartA = 0, runLastA = 0;
float runSum = 0, runLastD = 0;

/* ---- sound --------------------------------------------------------------- */
SoundFile[] idleBank = new SoundFile[VOICES];
SoundFile[] contactBank = new SoundFile[VOICES];
SoundFile[] closeBank = new SoundFile[VOICES];
int[]  voice = new int[3];
int    lastPing = 0, pingAt = -9999, pingInterval = 1250;
String mode = "SEARCH";

PFont fSmall, fMono, fBig, fTitle;
PGraphics gridLayer, overlay;
int blipLife = BLIP_MIN;
int glowLayers = 22;

void setup() {
  size(1280, 760);
  frameRate(60);

  String[] fonts = PFont.list();
  fSmall = pickFont(fonts, 11, false);
  fMono  = pickFont(fonts, 14, false);
  fBig   = pickFont(fonts, 22, true);
  fTitle = pickFont(fonts, 16, true);

  for (int i = 0; i < profile.length; i++) profile[i] = -1;
  for (int i = 0; i < VOICES; i++) {
    idleBank[i]    = new SoundFile(this, "ping_idle.wav");
    contactBank[i] = new SoundFile(this, "ping_contact.wav");
    closeBank[i]   = new SoundFile(this, "ping_close.wav");
  }

  buildGrid();
  buildOverlay();
  startMs = millis();
  openPort();
}

PFont pickFont(String[] have, float px, boolean bold) {
  String[] want = bold
    ? new String[] {"Menlo-Bold", "Consolas Bold", "DejaVu Sans Mono Bold", "Courier New Bold"}
    : new String[] {"Menlo-Regular", "Consolas", "DejaVu Sans Mono", "Courier New"};
  for (String w : want)
    for (String h : have)
      if (h.equals(w)) return createFont(w, px, true);
  return createFont("Monospaced", px, true);
}

/* Look for something that smells like an Arduino, newest port first. */
void openPort() {
  String[] list = Serial.list();
  String pick = PORT_NAME;
  String[] hints = {"cu.usbmodem", "cu.usbserial", "cu.wchusb", "usbmodem", "usbserial", "ttyACM", "ttyUSB", "COM"};
  for (String h : hints)
    for (int i = list.length - 1; i >= 0; i--)
      if (pick.length() == 0 && list[i].indexOf(h) >= 0) pick = list[i];

  if (pick.length() == 0) {
    demo = true;
    note("no board found: demo mode");
    return;
  }
  try {
    port = new Serial(this, pick, BAUD);
    port.bufferUntil('\n');
    linked = true;
    portLabel = pick.substring(pick.lastIndexOf('/') + 1);
    lastLineMs = millis();
    note("link " + portLabel);
  } catch (Exception e) {
    demo = true;
    note("port busy: demo mode");
  }
}

/* ================================================================== input == */
void serialEvent(Serial p) {
  try {                                   // an exception here would kill serial
    String line = p.readStringUntil('\n');
    if (line == null) return;
    line = trim(line);
    if (line.length() == 0) return;
    lastLineMs = millis();
    if (line.charAt(0) == '#') { note(trim(line.substring(1))); return; }
    if (demo) return;

    String[] f = split(line, ',');
    if (f.length != 2) return;
    int a = Integer.parseInt(trim(f[0]));
    int d = Integer.parseInt(trim(f[1]));
    if (a < 0 || a > 180) return;
    takeReading(a, d);
  } catch (Exception e) {
    // half a line during reset: ignore it
  }
}

void takeReading(int a, int d) {
  int now = millis();
  int prevDir = sweepDir;
  if (a > angle) sweepDir = 1;
  if (a < angle) sweepDir = -1;
  if (a != angle) stepDeg = constrain(abs(a - angle), 1, 4);

  if (sweepDir != prevDir) {              // the head just turned around
    closeRun();
    sweeps++;
    if (sweepStartMs > 0) sweepSecs = (now - sweepStartMs) / 1000.0;
    sweepStartMs = now;
  }

  boolean hit = d > 0 && d <= maxRange;
  for (int i = min(a, angle); i <= max(a, angle); i++)
    if (i != angle || a == angle) profile[i] = hit ? d : -1;
  angle = a;
  dist  = hit ? d : -1;

  if (!hit) { closeRun(); return; }

  blips.add(new Blip(a, d, stepDeg, now));
  if (blips.size() > 1200) blips.remove(0);

  if (runCount > 0 && abs(d - runLastD) > 14) closeRun();   // a different object
  if (runCount == 0) runStartA = a;
  runCount++;
  runSum  += d;
  runLastA = a;
  runLastD = d;
}

/* A streak just ended. Three or more echoes in a row count as an object. */
void closeRun() {
  if (runCount >= 3) {
    float brg  = (runStartA + runLastA) / 2.0;
    float rng  = runSum / runCount;
    float span = abs(runLastA - runStartA) + stepDeg;
    updateTrack(brg, rng, span);
  }
  runCount = 0;
  runSum = 0;
}

void updateTrack(float brg, float rng, float span) {
  int now = millis();
  Track best = null;
  float bestScore = 999999;
  for (Track t : tracks) {
    float dt   = (now - t.seen) / 1000.0;
    float pred = t.hits >= 2 ? t.rng + t.rate * dt : t.rng;   // where it should be by now
    float db   = abs(t.brg - brg), dr = abs(pred - rng);
    float gate = 30 + 9 * dt;                                 // the longer unseen, the looser
    if (db < 14 && dr < gate && db + dr * 0.4 < bestScore) { best = t; bestScore = db + dr * 0.4; }
  }
  if (best == null) {
    best = new Track(nextId++, brg, rng, span, now);
    tracks.add(best);
    note("T" + nf(best.id, 2) + " acquired " + nf(round(brg), 3) + "° " + round(rng) + "cm");
  } else {
    float dt = (now - best.seen) / 1000.0;
    if (dt > 0.5) best.rate = lerp(best.rate, (rng - best.rng) / dt, best.hits < 2 ? 1.0 : 0.5);
    best.brg  = lerp(best.brg, brg, 0.7);
    best.rng  = rng;
    best.span = span;
    best.seen = now;
    best.misses = 0;
    best.hits++;
  }

  /* The sensor only hears the first surface on a bearing. If we now see
     something farther away on this bearing, a nearer track cannot still be there. */
  for (int i = tracks.size() - 1; i >= 0; i--) {
    Track t = tracks.get(i);
    if (t != best && abs(t.brg - best.brg) < 8 && t.rng < best.rng - 10) {
      note("T" + nf(t.id, 2) + " lost");
      tracks.remove(i);
    }
  }

  if (best.rng < closeCm() && !best.warned) {
    best.warned = true;
    note("T" + nf(best.id, 2) + " CLOSE " + round(best.rng) + "cm");
  }
  if (best.rng >= closeCm() + 10) best.warned = false;
}

/* Tracks are dropped when the beam has swept past them twice and found
   nothing, or when they have gone unseen for more than two whole sweeps. */
void expireTracks() {
  int life = sweepSecs > 0 ? max(5000, (int)(sweepSecs * 2400)) : 12000;
  int now = millis();
  for (int i = tracks.size() - 1; i >= 0; i--) {
    Track t = tracks.get(i);
    float margin = t.span / 2 + 4;
    boolean passed = (sweepDir > 0 && angle > t.brg + margin) || (sweepDir < 0 && angle < t.brg - margin);
    if (passed && t.checkedSweep != sweeps) {
      t.checkedSweep = sweeps;
      if (now - t.seen > 1500) t.misses++;
    }
    if (t.misses >= 2 || now - t.seen > life) {
      note("T" + nf(t.id, 2) + " lost");
      tracks.remove(i);
    }
  }
}

/* =================================================================== draw == */
void draw() {
  if (demo && !paused) demoStep();
  beam += (angle - beam) * 0.3;
  blipLife = max(BLIP_MIN, (int)(sweepSecs * 1500));   // about a sweep and a half
  expireTracks();

  background(INK);
  image(gridLayer, 0, 0);
  drawProfile();
  drawBlips();
  drawBeam();
  drawPingRing();
  drawTracks();
  sonar();
  drawSensorPanel();
  drawTrackPanel();
  drawBars();
  image(overlay, 0, 0);
  drawAlarm();
}

/* Rings, spokes and labels never change, so they are drawn once. */
void buildGrid() {
  gridLayer = createGraphics(width, height);
  PGraphics pg = gridLayer;
  pg.beginDraw();
  pg.smooth();
  pg.textFont(fSmall);

  pg.noStroke();
  pg.fill(GREEN, 9);
  pg.arc(CX, CY, R * 2, R * 2, PI, TWO_PI, PIE);

  pg.noFill();
  for (int i = 1; i <= 8; i++) {                      // eight rings, every 2nd is major
    float r = R * i / 8.0;
    boolean major = i % 2 == 0;
    pg.stroke(major ? DIM : FAINT, major ? 150 : 120);
    pg.strokeWeight(i == 8 ? 2 : 1);
    pg.arc(CX, CY, r * 2, r * 2, PI, TWO_PI);
  }

  float rc = R * closeCm() / (float) maxRange;        // alarm zone
  pg.stroke(RED, 110);
  pg.strokeWeight(1);
  for (int a = 0; a < 180; a += 4)
    pg.arc(CX, CY, rc * 2, rc * 2, PI + radians(a), PI + radians(a + 2));

  for (int a = 0; a <= 180; a += 5) {
    float ca = cos(radians(180 - a)), sa = sin(radians(180 - a));
    boolean spoke = a % 30 == 0;
    float r0 = spoke ? R * 0.05 : (a % 15 == 0 ? R * 0.96 : R * 0.98);
    pg.stroke(spoke ? FAINT : DIM, spoke ? 170 : 190);
    pg.line(CX + ca * r0, CY - sa * r0, CX + ca * R, CY - sa * R);
    if (a % 15 == 0) {
      pg.fill(DIM);
      pg.noStroke();
      pg.textAlign(CENTER, CENTER);
      pg.text(nf(a, 3) + "°", CX + ca * (R + 22), CY - sa * (R + 14) - (a % 180 == 0 ? 9 : 0));
      pg.noFill();
    }
  }

  pg.stroke(DIM, 200);
  pg.strokeWeight(1.5);
  pg.line(CX - R, CY, CX + R, CY);

  pg.fill(DIM);
  pg.noStroke();
  pg.textAlign(CENTER, TOP);
  for (int i = 2; i <= 8; i += 2) {
    float r = R * i / 8.0;
    String lab = round(maxRange * i / 8.0) + (i == 8 ? " cm" : "");
    pg.text(lab, CX + r, CY + 5);
    pg.text(lab, CX - r, CY + 5);
  }
  pg.endDraw();
}

/* Scanlines and dark corners, also drawn once and laid over everything. */
void buildOverlay() {
  overlay = createGraphics(width, height);
  overlay.beginDraw();
  overlay.loadPixels();
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      float dx = (x - width / 2.0) / (width / 2.0);
      float dy = (y - height / 2.0) / (height / 2.0);
      float v = constrain((dx * dx + dy * dy - 0.6) * 0.8, 0, 1);
      float a = v * 170 + (y % 3 == 0 ? 30 : 0);
      overlay.pixels[y * width + x] = color(0, 0, 0, constrain(a, 0, 255));
    }
  }
  overlay.updatePixels();
  overlay.endDraw();
}

/* Echoes: short arcs at their range, so a flat object draws as one curve. */
void drawBlips() {
  int now = millis();
  noFill();
  strokeCap(SQUARE);
  for (int i = blips.size() - 1; i >= 0; i--) {
    Blip b = blips.get(i);
    float age = (now - b.t) / (float) blipLife;
    if (age >= 1) { blips.remove(i); continue; }

    float fade = pow(1 - age, 1.35);
    float r    = R * b.dist / maxRange;
    float s    = -radians(180 - b.ang);
    float half = radians(b.span * 0.5 + 0.3);
    color c    = b.dist < closeCm() ? RED : GREEN;
    c = lerpColor(c, WHITE, 0.75 * constrain(1 - (now - b.t) / 220.0, 0, 1));

    stroke(c, 55 * fade);                 // glow
    strokeWeight(11);
    arc(CX, CY, r * 2, r * 2, s - half, s + half);
    stroke(c, 255 * fade);                // the echo itself
    strokeWeight(3.2);
    arc(CX, CY, r * 2, r * 2, s - half, s + half);

    if (age < 0.25) {                     // shadow: we cannot see behind it
      stroke(c, 26 * (1 - age * 4));
      strokeWeight(1);
      line(CX + cos(s) * (r + 8), CY + sin(s) * (r + 8), CX + cos(s) * R, CY + sin(s) * R);
    }
  }
  strokeCap(ROUND);
}

/* The beam and its afterglow. The glow is a stack of very faint wedges that
   all start at the beam and reach back different distances. Where many of
   them overlap it is bright, where few do it is dim, and no single edge is
   strong enough to show as a line. */
void drawBeam() {
  float s = -radians(180 - beam);
  float spread = radians(44);
  int layers = glowLayers;
  if (frameCount % 90 == 0 && frameRate < 42 && glowLayers > 10) glowLayers -= 4;   // slow laptop
  noStroke();
  fill(GREEN, 110.0 / layers);
  for (int j = 1; j <= layers; j++) {
    float reach = spread * (1 - sqrt(j / (float) layers)) + radians(0.6);
    float far = constrain(s - sweepDir * reach, -PI, 0);
    if (abs(far - s) < 0.002) continue;
    arc(CX, CY, R * 2, R * 2, min(s, far), max(s, far), PIE);
  }

  float x = CX + cos(s) * R, y = CY + sin(s) * R;
  stroke(GREEN, 70);
  strokeWeight(6);
  line(CX, CY, x, y);
  stroke(WHITE, 240);
  strokeWeight(1.6);
  line(CX, CY, x, y);

  noStroke();
  fill(GREEN, 40);
  ellipse(CX, CY, 30, 30);
  fill(WHITE);
  ellipse(CX, CY, 7, 7);
}

/* A ring leaves the dish every time you hear a ping. */
void drawPingRing() {
  float age = (millis() - pingAt) / 1100.0;
  if (age < 0 || age > 1) return;
  float r = R * (1 - pow(1 - age, 2));
  noFill();
  stroke(mode.equals("CLOSE") ? RED : GREEN, 120 * pow(1 - age, 2));
  strokeWeight(1 + 2 * (1 - age));
  arc(CX, CY, r * 2, r * 2, PI, TWO_PI);
}

/* Bracket, ID and range for every tracked object. */
void drawTracks() {
  int now = millis();
  textFont(fSmall);
  for (Track t : tracks) {
    float s = -radians(180 - t.brg);
    float r = R * t.rng / maxRange;
    float x = CX + cos(s) * r, y = CY + sin(s) * r;
    boolean near = t.rng < closeCm();
    color c = near ? RED : AMBER;
    float fresh = constrain(1 - (now - t.seen) / 9000.0, 0.35, 1);
    float k = 15 + (near ? 3 * sin(now / 90.0) : 0);

    noFill();
    stroke(c, 255 * fresh);
    strokeWeight(1.6);
    for (int sx = -1; sx <= 1; sx += 2) {
      for (int sy = -1; sy <= 1; sy += 2) {
        line(x + sx * k, y + sy * k, x + sx * (k - 6), y + sy * k);
        line(x + sx * k, y + sy * k, x + sx * k, y + sy * (k - 6));
      }
    }
    if (abs(t.rate) >= 2) {               // motion: toward or away from the dish
      float len = constrain(abs(t.rate) * 1.6, 8, 46);
      float dirSign = t.rate < 0 ? -1 : 1;
      float ex = x + cos(s) * len * dirSign, ey = y + sin(s) * len * dirSign;
      line(x, y, ex, ey);
      noStroke();
      fill(c, 255 * fresh);
      ellipse(ex, ey, 5, 5);
    }

    boolean flip = x > CX + R * 0.62;
    noStroke();
    fill(c, 255 * fresh);
    textAlign(flip ? RIGHT : LEFT, BASELINE);
    float lx = x + (flip ? -k - 7 : k + 7);
    text("T" + nf(t.id, 2), lx, y - 3);
    fill(WHITE, 230 * fresh);
    text(round(t.rng) + " cm", lx, y + 11);
  }
}

/* Range at every bearing from the latest pass. Taller bar = closer. */
void drawProfile() {
  float x0 = CX - R, w = R * 2.0, y1 = 738, h = 30;
  noStroke();
  fill(PANEL);
  rect(x0, y1 - h - 2, w, h + 2);
  stroke(FAINT);
  strokeWeight(1);
  line(x0, y1, x0 + w, y1);
  noStroke();
  for (int a = 0; a <= 180; a++) {
    if (profile[a] <= 0) continue;
    float bar = 3 + (h - 3) * (1 - profile[a] / maxRange);
    fill(profile[a] < closeCm() ? RED : GREEN, 190);
    rect(x0 + w * a / 181.0, y1 - bar, w / 181.0 - 0.6, bar);
  }
  stroke(WHITE, 200);
  float bx = x0 + w * (beam + 0.5) / 181.0;
  line(bx, y1 - h - 2, bx, y1);

  fill(DIM);
  noStroke();
  textFont(fSmall);
  textAlign(RIGHT, BASELINE);
  text("RANGE", x0 - 12, y1 - 16);
  text("PROFILE", x0 - 12, y1 - 3);
}

/* ================================================================== sound == */
void sonar() {
  int now = millis();

  /* what are we pinging at? nearest track, or a fresh echo if it is closer */
  float target = -1, bearing = 90;
  for (Track t : tracks)
    if (target < 0 || t.rng < target) { target = t.rng; bearing = t.brg; }

  int recent = 0;
  Blip nb = null;
  for (int i = blips.size() - 1; i >= 0; i--) {
    Blip b = blips.get(i);
    if (now - b.t > 600) break;
    recent++;
    if (nb == null || b.dist < nb.dist) nb = b;
  }
  if (recent >= 3 && (target < 0 || nb.dist < target)) { target = nb.dist; bearing = nb.ang; }

  float amp = 0.42, rate = 1.0, pan = 0;
  SoundFile[] bank = idleBank;
  int slot = 0;

  if (target < 0) {
    mode = "SEARCH";
    pingInterval = 1250;
  } else {
    float f = constrain(norm(target, maxRange * 0.1, maxRange), 0, 1);   // 0 = on top of us
    pingInterval = (int) lerp(170, 820, f);
    amp  = lerp(1.0, 0.6, f);
    rate = lerp(1.12, 0.98, f);
    pan  = map(bearing, 0, 180, -0.7, 0.7);
    if (target < closeCm()) { mode = "CLOSE";   bank = closeBank;   slot = 2; }
    else                   { mode = "CONTACT"; bank = contactBank; slot = 1; }
  }

  if (now - lastPing >= pingInterval) {
    lastPing = now;
    pingAt = now;
    if (!muted) {
      SoundFile s = bank[voice[slot]];
      voice[slot] = (voice[slot] + 1) % VOICES;
      s.play(rate, pan, amp * MASTER_VOL);
    }
  }
}

/* ==================================================================== hud == */
void panel(float x, float y, float w, float h, String title) {
  noStroke();
  fill(PANEL, 225);
  rect(x, y, w, h);
  noFill();
  stroke(FAINT);
  strokeWeight(1);
  rect(x, y, w, h);
  stroke(GREEN, 200);
  strokeWeight(1.6);
  float c = 9;
  line(x, y, x + c, y);          line(x, y, x, y + c);
  line(x + w, y, x + w - c, y);  line(x + w, y, x + w, y + c);
  line(x, y + h, x + c, y + h);  line(x, y + h, x, y + h - c);
  line(x + w, y + h, x + w - c, y + h);  line(x + w, y + h, x + w, y + h - c);
  noStroke();
  fill(DIM);
  textFont(fSmall);
  textAlign(LEFT, BASELINE);
  text(title, x + 12, y + 19);
  stroke(FAINT);
  strokeWeight(1);
  line(x + 12, y + 27, x + w - 12, y + 27);
}

void drawSensorPanel() {
  float x = 24, y = 90, w = 216, h = 240;
  panel(x, y, w, h, "SENSOR HEAD");

  Track near = null;
  for (Track t : tracks) if (near == null || t.rng < near.rng) near = t;

  String[] labels = {"BEARING", "RANGE", "NEAREST TRACK", "SWEEP"};
  String[] values = {
    nf(angle, 3) + "°",
    dist > 0 ? nf(dist, 3) + " cm" : "no echo",
    near != null ? round(near.rng) + " cm  T" + nf(near.id, 2) : "none",
    (sweepSecs > 0 ? nf(sweepSecs, 1, 1) + " s" : "timing") + "  #" + sweeps
  };
  float yy = y + 48;
  for (int i = 0; i < labels.length; i++) {
    noStroke();
    textAlign(LEFT, BASELINE);
    textFont(fSmall);
    fill(DIM);
    text(labels[i], x + 12, yy);
    textFont(i < 2 ? fBig : fMono);
    fill(i == 1 && dist > 0 && dist < closeCm() ? RED
       : i == 2 && near != null && near.rng < closeCm() ? RED : GREEN);
    text(values[i], x + 12, yy + (i < 2 ? 25 : 19));
    yy += i < 2 ? 54 : 44;
    if (i == 1) {                        // range bar
      fill(FAINT);
      rect(x + 12, yy - 16, w - 24, 5);
      if (dist > 0) {
        fill(dist < closeCm() ? RED : GREEN);
        rect(x + 12, yy - 16, (w - 24) * (1 - dist / (float) maxRange), 5);
      }
      yy += 6;
    }
  }
}

void drawTrackPanel() {
  float x = 1040, y = 90, w = 216, h = 240;
  panel(x, y, w, h, "TRACKS  " + tracks.size());

  textFont(fSmall);
  noStroke();
  fill(DIM);
  textAlign(LEFT, BASELINE);
  text("ID", x + 12, y + 44);
  text("BRG", x + 46, y + 44);
  text("CM", x + 92, y + 44);
  text("SPAN", x + 126, y + 44);
  textAlign(RIGHT, BASELINE);
  text("CM/S", x + w - 12, y + 44);

  /* nearest first */
  ArrayList<Track> sorted = new ArrayList<Track>(tracks);
  for (int i = 1; i < sorted.size(); i++)
    for (int j = i; j > 0 && sorted.get(j).rng < sorted.get(j - 1).rng; j--) {
      Track tmp = sorted.get(j); sorted.set(j, sorted.get(j - 1)); sorted.set(j - 1, tmp);
    }

  float yy = y + 62;
  if (sorted.size() == 0) {
    fill(FAINT);
    textAlign(LEFT, BASELINE);
    text("water clear", x + 12, yy);
  }
  for (int i = 0; i < min(4, sorted.size()); i++) {
    Track t = sorted.get(i);
    fill(t.rng < closeCm() ? RED : AMBER);
    textAlign(LEFT, BASELINE);
    text("T" + nf(t.id, 2), x + 12, yy);
    fill(WHITE);
    text(nf(round(t.brg), 3) + "°", x + 46, yy);
    text(nf(round(t.rng), 3), x + 92, yy);
    text(round(t.span) + "°", x + 126, yy);
    textAlign(RIGHT, BASELINE);
    if (abs(t.rate) < 2) { fill(DIM); text("steady", x + w - 12, yy); }
    else {
      fill(t.rate < 0 ? RED : GREEN);
      text((t.rate < 0 ? "in " : "out ") + round(abs(t.rate)), x + w - 12, yy);
    }
    yy += 17;
  }

  /* event log */
  float ly = y + 140;
  stroke(FAINT);
  line(x + 12, ly - 14, x + w - 12, ly - 14);
  noStroke();
  fill(DIM);
  textAlign(LEFT, BASELINE);
  text("EVENT LOG", x + 12, ly);
  int rows = 5, first = max(0, logLines.size() - rows);
  for (int i = first; i < logLines.size(); i++) {
    String s = logLines.get(i);
    if (s.length() > 29) s = s.substring(0, 28) + "…";
    fill(s.indexOf("CLOSE") >= 0 ? RED : GREEN, i == logLines.size() - 1 ? 255 : 150);
    ly += 16;
    text(s, x + 12, ly);
  }
}

void drawBars() {
  int now = millis();
  boolean stale = linked && !demo && !paused && now - lastLineMs > 2500;

  /* title bar */
  noStroke();
  fill(GREEN, 16);
  rect(0, 0, width, 44);
  fill(GREEN);
  textFont(fTitle);
  textAlign(LEFT, BASELINE);
  text("ACTIVE SONAR", 24, 28);
  float tw = textWidth("ACTIVE SONAR");
  textFont(fSmall);
  fill(DIM);
  text("ultrasonic scanning array   ·   HC-SR04 on servo   ·   " + maxRange + " cm scale", 24 + tw + 18, 27);

  int secs = (now - startMs) / 1000;
  textAlign(RIGHT, BASELINE);
  textFont(fMono);
  fill(demo ? AMBER : GREEN);
  text((demo ? "DEMO" : "LINK " + portLabel) + "    T+" + nf(secs / 60, 2) + ":" + nf(secs % 60, 2), width - 24, 28);

  /* status strip */
  color c = mode.equals("CLOSE") ? RED : mode.equals("CONTACT") ? AMBER : GREEN;
  String msg = mode.equals("CLOSE") ? "CONTACT CLOSE  ·  inside " + round(closeCm()) + " cm"
             : mode.equals("CONTACT") ? "CONTACT  ·  holding track"
             : "WATER CLEAR  ·  listening";
  if (paused) { c = AMBER; msg = "SWEEP PAUSED  ·  press SPACE"; }
  if (stale)  { c = AMBER; msg = "NO DATA FROM BOARD  ·  check the port, close the Serial Monitor"; }
  fill(c, mode.equals("CLOSE") && !paused ? 40 + 30 * sin(now / 90.0) : 30);
  rect(0, 44, width, 28);
  fill(c);
  textFont(fMono);
  textAlign(LEFT, BASELINE);
  text(msg, 24, 63);
  textAlign(RIGHT, BASELINE);
  text("ping " + pingInterval + " ms   ·   sound " + (muted ? "off" : "on") + "   ·   speed " + speed, width - 24, 63);

  /* key hints */
  fill(DIM);
  textFont(fSmall);
  textAlign(CENTER, BASELINE);
  text("SPACE pause     1 2 3 speed     R range     S sound     D demo     C clear", width / 2, height - 6);
}

/* red frame that throbs while something is inside the alarm zone */
void drawAlarm() {
  if (!mode.equals("CLOSE")) return;
  noFill();
  stroke(RED, 90 + 80 * sin(millis() / 90.0));
  strokeWeight(5);
  rect(2, 2, width - 5, height - 5);
}

void note(String s) {
  logLines.add(s);
  if (logLines.size() > 200) logLines.remove(0);
}

/* =================================================================== demo == */
int demoLast = 0, demoDir = 1;
void demoStep() {
  int now = millis();
  int stepMs = speed == 1 ? 60 : 35;
  if (now - demoLast > 400) demoLast = now - stepMs;
  for (int n = 0; n < 3 && now - demoLast >= stepMs; n++) {
    demoLast += stepMs;
    demoOne(now);
  }
}

void demoOne(int now) {
  int a = angle + demoDir * (speed == 3 ? 2 : 1);
  if (a >= 165) { a = 165; demoDir = -1; }
  if (a <= 15)  { a = 15;  demoDir = 1; }
  int d = -1;
  float t = now / 1000.0;

  /* a wall-ish thing on the left that never moves */
  if (a >= 36 && a <= 54) d = 150 + round(4 * sin(a * 0.4));
  /* something that walks in to 30 cm and back out again */
  if (abs(a - 100) <= 6) d = round(82 + 52 * cos(t * TWO_PI / 34.0));
  /* a small thing on the right that comes and goes */
  if (abs(a - 141) <= 3 && (t % 30) < 19) d = 96;

  if (d > 0) d += round(random(-1.2, 1.2));
  takeReading(a, d);
}

/* =================================================================== keys == */
void keyPressed() {
  if (key == ' ') {
    paused = !paused;
    if (linked && !demo) port.write('p');
    else note(paused ? "sweep paused" : "sweep resumed");
  }
  if (key >= '1' && key <= '3') {
    speed = key - '0';
    if (linked && !demo) port.write(key);
    else note("speed " + speed);
  }
  if (key == 's' || key == 'S') {
    muted = !muted;
    note(muted ? "sound off" : "sound on");
  }
  if (key == 'd' || key == 'D') {
    if (port == null) { note("no board: demo only"); return; }
    demo = !demo;
    clearAll();
    note(demo ? "demo mode" : "live");
  }
  if (key == 'c' || key == 'C') {
    clearAll();
    note("screen cleared");
  }
  if (key == 'r' || key == 'R') {
    int i = 0;
    while (i < RANGES.length - 1 && RANGES[i] != maxRange) i++;
    maxRange = RANGES[(i + 1) % RANGES.length];
    clearAll();
    buildGrid();
    note("range " + maxRange + " cm");
  }
}

void clearAll() {
  blips.clear();
  tracks.clear();
  runCount = 0;
  runSum = 0;
  for (int i = 0; i < profile.length; i++) profile[i] = -1;
}

/* ================================================================== types == */
class Blip {
  float ang, dist, span;
  int t;
  Blip(float a, float d, float sp, int tt) { ang = a; dist = d; span = sp; t = tt; }
}

class Track {
  int id, born, seen, hits = 1, misses = 0, checkedSweep = -1;
  float brg, rng, span, rate = 0;
  boolean warned = false;
  Track(int i, float b, float r, float sp, int now) {
    id = i; brg = b; rng = r; span = sp; born = now; seen = now;
  }
}
