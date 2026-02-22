#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SparkFun_SCD4x_Arduino_Library.h>

#include <qrcode.h>

#include <esp_task_wdt.h>

#include "EPD_4in2.h"
#include "GUI_Paint.h"
#include "config.h"
#include "hiki_bitmaps.h"

// I2C pins (ESP32-C6 Insight board)
#define SDA_PIN 19
#define SCL_PIN 18

// BOOT button (GPIO9, active LOW, internal pull-up)
#define BTN_PIN       9
#define DEBOUNCE_MS   50

// Display
#define DISPLAY_W EPD_4IN2_V2_WIDTH   // 400
#define DISPLAY_H EPD_4IN2_V2_HEIGHT  // 300

static UBYTE *framebuffer = nullptr;

// ─── State structs ────────────────────────────────────────────

struct SensorData {
    bool  ok   = false;
    float co2  = 0, temp = 0, hum = 0;
};

struct HealthState {
    bool received = false;
    bool ha = false, gw = false, inet = false, ha_api = false;
    int  ha_ms = 0, gw_ms = 0, inet_ms = 0;
    int  mem = 0, disk = 0, msgs_24h = 0;
    char up[16] = "", model[24] = "";
};

struct KillswitchState {
    bool received     = false;
    char state[16]    = "unknown";
    char address[64]  = "";
    bool ws_connected = false;
    char isolated_at[24] = "";
    int  block_number = 0;
};

struct GatewayHealth {
    bool received     = false;
    int  ha_errors    = 0;
    bool ha_reachable = false;
};

struct AppState {
    int  current_screen = 0;
    int  cycle_count    = 0;
};

static SCD4x           scd4x_driver;
static SensorData      sensor;
static HealthState     health;
static KillswitchState killswitch;
static GatewayHealth   gw_health;
static AppState        app;

// ─── Layout constants ─────────────────────────────────────────

namespace Layout {
    constexpr int MARGIN_L   = 12;
    constexpr int MARGIN_R   = 388;
    constexpr int FONT16_W   = 11;
    constexpr int FONT20_W   = 14;
    constexpr int FONT24_W   = 17;
    constexpr int CO2_MAX    = 2000;
}

// Network
static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);

// MQTT topics
static const char * const TOPIC_CO2  = DEVICE_ID "/sensor/co2";
static const char * const TOPIC_TEMP = DEVICE_ID "/sensor/temperature";
static const char * const TOPIC_HUM  = DEVICE_ID "/sensor/humidity";
static const char * const TOPIC_HEALTH = "hiki/health";
static const char * const TOPIC_KILLSWITCH = "hiki/killswitch/status";
static const char * const TOPIC_GW_HEALTH = "hiki/gateway/health";

// Screen cycling
#define SCREEN_COUNT      4
#define CYCLE_MS          20000       // switch screens every 20s
#define SENSOR_EVERY_N    6           // read+publish sensors every 6 cycles (60s)
#define FULL_REFRESH_EVERY_N 3        // full e-ink refresh every 3 cycles (~60s)

// ─── JSON helpers ──────────────────────────────────────────────

static int jsonInt(const char *json, const char *key) {
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    return atoi(p);
}

static void jsonStr(const char *json, const char *key, char *out, size_t out_sz) {
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) { out[0] = '\0'; return; }
    p += strlen(search);
    while (*p == ' ') p++;  // skip whitespace after colon
    if (*p != '"') { out[0] = '\0'; return; }  // null or non-string
    p++;  // skip opening quote
    const char *end = strchr(p, '"');
    if (!end) { out[0] = '\0'; return; }
    size_t len = end - p;
    if (len >= out_sz) len = out_sz - 1;
    memcpy(out, p, len);
    out[len] = '\0';
}

static bool jsonBool(const char *json, const char *key) {
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    return strncmp(p, "true", 4) == 0;
}

// ─── MQTT ──────────────────────────────────────────────────────

static void mqttCallback(char *topic, byte *payload, unsigned int length) {
    Serial.printf("MQTT msg [%s]: %.*s\n", topic, length, (char *)payload);

    if (length >= 512) {
        Serial.println("MQTT: message too large, dropped");
        return;
    }

    char buf[512];
    memcpy(buf, payload, length);
    buf[length] = '\0';

    if (strcmp(topic, TOPIC_HEALTH) == 0) {
        health.ha = jsonInt(buf, "ha") != 0;
        health.gw = jsonInt(buf, "gw") != 0;
        health.inet = jsonInt(buf, "inet") != 0;
        health.ha_api = jsonInt(buf, "ha_api") != 0;
        health.ha_ms = jsonInt(buf, "ha_ms");
        health.gw_ms = jsonInt(buf, "gw_ms");
        health.inet_ms = jsonInt(buf, "inet_ms");
        health.mem = jsonInt(buf, "mem");
        health.disk = jsonInt(buf, "disk");
        health.msgs_24h = jsonInt(buf, "msgs_24h");
        jsonStr(buf, "up", health.up, sizeof(health.up));
        jsonStr(buf, "model", health.model, sizeof(health.model));
        health.received = true;
        Serial.println("Health data parsed OK");
    } else if (strcmp(topic, TOPIC_KILLSWITCH) == 0) {
        jsonStr(buf, "state", killswitch.state, sizeof(killswitch.state));
        jsonStr(buf, "address", killswitch.address, sizeof(killswitch.address));
        killswitch.ws_connected = jsonBool(buf, "ws_connected");
        jsonStr(buf, "isolated_at", killswitch.isolated_at, sizeof(killswitch.isolated_at));
        killswitch.block_number = jsonInt(buf, "block_number");
        killswitch.received = true;
        Serial.printf("Killswitch: state=%s ws=%d addr=%s\n",
                       killswitch.state, killswitch.ws_connected, killswitch.address);
    } else if (strcmp(topic, TOPIC_GW_HEALTH) == 0) {
        gw_health.ha_errors = jsonInt(buf, "ha_errors");
        gw_health.ha_reachable = jsonBool(buf, "ha_reachable");
        gw_health.received = true;
        Serial.printf("GW health: errors=%d reachable=%d\n",
                       gw_health.ha_errors, gw_health.ha_reachable);
    }
}

static void publishSensorDiscovery(const char *name, const char *dev_class,
                                   const char *suffix, const char *unit) {
    char cfg[300], topic[80];
    snprintf(cfg, sizeof(cfg),
        "{\"name\":\"%s\","
        "\"device_class\":\"%s\","
        "\"state_topic\":\"" DEVICE_ID "/sensor/%s\","
        "\"unit_of_measurement\":\"%s\","
        "\"unique_id\":\"torii_ink_%s\","
        "\"device\":{\"identifiers\":[\"torii_ink\"],"
        "\"name\":\"Torii Ink\",\"model\":\"ESP32-C6 e-ink\","
        "\"manufacturer\":\"Hiki\"}}",
        name, dev_class, suffix, unit, suffix);
    snprintf(topic, sizeof(topic),
        "homeassistant/sensor/torii_ink_%s/config", suffix);
    mqtt.publish(topic, cfg, true);
}

static void publishDiscovery() {
    publishSensorDiscovery("CO2",         "carbon_dioxide", "co2",         "ppm");
    publishSensorDiscovery("Temperature", "temperature",    "temperature", "\u00b0C");
    publishSensorDiscovery("Humidity",    "humidity",       "humidity",    "%");
    Serial.println("MQTT: HA discovery configs published");
}

static void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqtt.connected()) return;

    Serial.print("MQTT: connecting... ");
    if (mqtt.connect(DEVICE_ID)) {
        Serial.println("connected");
        mqtt.subscribe(TOPIC_HEALTH);
        mqtt.subscribe(TOPIC_KILLSWITCH);
        mqtt.subscribe(TOPIC_GW_HEALTH);
        for (int i = 0; i < 5; i++) { delay(100); mqtt.loop(); }
        publishDiscovery();
    } else {
        Serial.printf("failed (rc=%d)\n", mqtt.state());
    }
}

static void publishSensors() {
    if (!mqtt.connected()) return;

    char val[16];
    if (sensor.ok) {
        snprintf(val, sizeof(val), "%.0f", sensor.co2);
        mqtt.publish(TOPIC_CO2, val);
        snprintf(val, sizeof(val), "%.1f", sensor.temp);
        mqtt.publish(TOPIC_TEMP, val);
        snprintf(val, sizeof(val), "%.0f", sensor.hum);
        mqtt.publish(TOPIC_HUM, val);
    }
    Serial.println("MQTT: sensors published");
}

// ─── Hardware init ─────────────────────────────────────────────

static void initDisplay() {
    DEV_Module_Init();
    EPD_4IN2_V2_Init();
    EPD_4IN2_V2_Clear();
    EPD_4IN2_V2_Init_Fast(Seconds_1_5S);

    uint32_t imageSize = ((DISPLAY_W % 8 == 0) ? (DISPLAY_W / 8) : (DISPLAY_W / 8 + 1)) * DISPLAY_H;
    framebuffer = (UBYTE *)malloc(imageSize);
    if (!framebuffer) {
        Serial.println("Failed to allocate framebuffer!");
        return;
    }
    Paint_NewImage(framebuffer, DISPLAY_W, DISPLAY_H, ROTATE_0, WHITE);
    Paint_SelectImage(framebuffer);
    Paint_Clear(WHITE);
}

static void initSensors() {
    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    if (scd4x_driver.begin(Wire, false, false, false)) {
        sensor.ok = true;
        Serial.println("SCD4x: detected, starting periodic measurement...");
        scd4x_driver.startPeriodicMeasurement();
    } else {
        Serial.println("SCD4x: not found");
    }
}

static void initWiFi() {
    Serial.printf("WiFi: connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("WiFi: connected, IP=%s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("WiFi: connection failed, will retry");
    }
}

static bool readSCD4x() {
    if (!sensor.ok) return false;
    if (!scd4x_driver.getDataReadyStatus()) return false;
    if (scd4x_driver.readMeasurement()) {
        sensor.co2 = scd4x_driver.getCO2();
        sensor.temp = scd4x_driver.getTemperature();
        sensor.hum = scd4x_driver.getHumidity();
        Serial.printf("SCD4x: CO2=%.0f ppm, T=%.1f C, H=%.0f%%\n", sensor.co2, sensor.temp, sensor.hum);
        return true;
    }
    return false;
}

static void readSensors() {
    readSCD4x();
}

// ─── State evaluation ─────────────────────────────────────────

static int clampedCO2() {
    int v = (int)sensor.co2;
    return (v > Layout::CO2_MAX) ? Layout::CO2_MAX : v;
}

static const char *getCO2Label() {
    if (sensor.co2 < 600) return "Excellent";
    if (sensor.co2 < 1000) return "Good";
    if (sensor.co2 < 1500) return "Stuffy";
    return "Ventilate!";
}

static bool hasAnyProblem() {
    bool isolated = (strcmp(killswitch.state, "isolated") == 0);
    bool node_down = health.received && (!health.ha || !health.gw || !health.inet);
    bool co2_high = sensor.ok && sensor.co2 > 1000;
    return isolated || node_down || co2_high;
}

static const char *getPersonalityMessage() {
    bool isolated = (strcmp(killswitch.state, "isolated") == 0);
    if (isolated) return "Cut off from world";
    if (health.received && (!health.ha || !health.gw || !health.inet)) return "Something is off...";
    if (sensor.ok && sensor.co2 > 1500) return "Open a window pls?";
    if (sensor.ok && sensor.co2 > 1000) return "Air getting stuffy.";
    if (health.received && health.msgs_24h == 0) return "It's quiet today.";
    if (health.received && health.msgs_24h > 10) return "Busy day!";
    if (health.up[0] == '0') return "Just woke up...";
    return "All systems nominal.";
}

// ─── Drawing helpers ───────────────────────────────────────────

// Corner brackets on 4 corners of display — cyberpunk "targeting" frame
static void drawCornerBrackets(int arm = 15, int margin = 2) {
    // Top-left
    Paint_DrawLine(margin, margin, margin + arm, margin, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(margin, margin, margin, margin + arm, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    // Top-right
    Paint_DrawLine(DISPLAY_W - margin - arm, margin, DISPLAY_W - margin, margin, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(DISPLAY_W - margin, margin, DISPLAY_W - margin, margin + arm, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    // Bottom-left
    Paint_DrawLine(margin, DISPLAY_H - margin, margin + arm, DISPLAY_H - margin, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(margin, DISPLAY_H - margin - arm, margin, DISPLAY_H - margin, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    // Bottom-right
    Paint_DrawLine(DISPLAY_W - margin - arm, DISPLAY_H - margin, DISPLAY_W - margin, DISPLAY_H - margin, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(DISPLAY_W - margin, DISPLAY_H - margin - arm, DISPLAY_W - margin, DISPLAY_H - margin, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
}

// Cyber header: ">> LABEL <<" + solid line below
static void drawCyberHeader(int y, const char *label) {
    char buf[40];
    snprintf(buf, sizeof(buf), ">> %s <<", label);
    Paint_DrawString_EN(Layout::MARGIN_L, y, buf, &Font20, WHITE, BLACK);
    Paint_DrawLine(Layout::MARGIN_L, y + 22, Layout::MARGIN_R, y + 22, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

// Full-width dotted separator
static void drawDottedLine(int y) {
    Paint_DrawLine(Layout::MARGIN_L, y, Layout::MARGIN_R, y,
                   BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
}

// Double horizontal line — "data bus" separator
static void drawDoubleLine(int y, int x1 = 8, int x2 = 392) {
    Paint_DrawLine(x1, y, x2, y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x1, y + 3, x2, y + 3, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

// Draw truncated SS58 address: "5DcXdyxU...yc5H"
static void drawAddress(int x, int y, const char *addr, sFONT *font) {
    if (!addr || !addr[0]) {
        Paint_DrawString_EN(x, y, "---", font, WHITE, BLACK);
        return;
    }
    int len = strlen(addr);
    char short_addr[20];
    if (len > 12) {
        snprintf(short_addr, sizeof(short_addr), "%.8s...%.4s", addr, addr + len - 4);
    } else {
        snprintf(short_addr, sizeof(short_addr), "%s", addr);
    }
    Paint_DrawString_EN(x, y, short_addr, font, WHITE, BLACK);
}

static void drawNodeCircle(int x, int y, bool online, const char *label) {
    Paint_DrawCircle(x, y, 4, BLACK, DOT_PIXEL_1X1,
                     online ? DRAW_FILL_FULL : DRAW_FILL_EMPTY);
    Paint_DrawString_EN(x + 8, y - 6, label, &Font16, WHITE, BLACK);
}

static void drawProgressBar(int x, int y, int w, int h, int value, int max_value) {
    Paint_DrawRectangle(x, y, x + w, y + h, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    int fill_w = 0;
    if (max_value > 0 && value > 0) {
        fill_w = (value * (w - 2)) / max_value;
        if (fill_w > w - 2) fill_w = w - 2;
    }
    if (fill_w > 0)
        Paint_DrawRectangle(x + 1, y + 1, x + 1 + fill_w, y + h - 1,
                            BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

// ─── Icon helpers (primitives only) ───────────────────────────

// Thermometer: stem + bulb, ~10x16 at (x,y)
static void drawIconThermo(int x, int y) {
    Paint_DrawRectangle(x + 3, y, x + 7, y + 9, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(x + 4, y + 4, x + 6, y + 9, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawCircle(x + 5, y + 13, 3, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

// Water droplet: teardrop shape, ~10x16 at (x,y)
static void drawIconDrop(int x, int y) {
    Paint_DrawLine(x + 5, y, x + 1, y + 8, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x + 5, y, x + 9, y + 8, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawCircle(x + 5, y + 10, 4, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
}

// Clock: circle + two hands, 14x14 centered at (x+7,y+7)
static void drawIconClock(int x, int y) {
    int cx = x + 7, cy = y + 7;
    Paint_DrawCircle(cx, cy, 6, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawLine(cx, cy, cx + 3, cy - 4, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(cx, cy, cx, cy - 5, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawCircle(cx, cy, 1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

// WiFi signal bars: 4 bars of increasing height, bottom-aligned
static void drawSignalBars(int x, int y, int rssi) {
    int bars = (rssi > -50) ? 4 : (rssi > -60) ? 3 : (rssi > -70) ? 2 : (rssi > -80) ? 1 : 0;
    for (int i = 0; i < 4; i++) {
        int bx = x + i * 5;
        int bh = 4 + i * 3;
        int by = y + 14 - bh;
        Paint_DrawRectangle(bx, by, bx + 3, y + 14, BLACK, DOT_PIXEL_1X1,
                            (i < bars) ? DRAW_FILL_FULL : DRAW_FILL_EMPTY);
    }
}

// Inverted badge: black background, white text (clamped to display width)
static void drawBadge(int x, int y, const char *text, sFONT *font) {
    int w = strlen(text) * font->Width + 8;
    int h = font->Height + 2;
    if (x + w > DISPLAY_W) w = DISPLAY_W - x;
    Paint_DrawRectangle(x, y, x + w, y + h, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawString_EN(x + 4, y + 1, text, font, BLACK, WHITE);
}

// WiFi signal bars + RSSI text (fits within display at x >= 310)
static void drawWiFiStatus(int x, int y) {
    int rssi = WiFi.RSSI();
    char buf[12];
    snprintf(buf, sizeof(buf), "%ddB", rssi);
    drawSignalBars(x, y, rssi);
    Paint_DrawString_EN(x + 22, y + 2, buf, &Font16, WHITE, BLACK);
}

// Node status line: "HA:ok  GW:!  NET:--"
static void drawNodeStatusLine(int x, int y) {
    char buf[48];
    const char *ha_s  = health.received ? (health.ha  ? "ok" : "!") : "--";
    const char *gw_s  = health.received ? (health.gw  ? "ok" : "!") : "--";
    const char *net_s = health.received ? (health.inet ? "ok" : "!") : "--";
    snprintf(buf, sizeof(buf), "HA:%s  GW:%s  NET:%s", ha_s, gw_s, net_s);
    Paint_DrawString_EN(x, y, buf, &Font16, WHITE, BLACK);
}

// Labeled data panel: bordered box with label on top edge, icon, and value
typedef void (*IconDrawFn)(int x, int y);
static void drawLabeledPanel(int x, int y, int w, int h,
                             const char *label, IconDrawFn icon,
                             const char *value) {
    Paint_DrawRectangle(x, y, x + w, y + h, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    int lbl_w = strlen(label) * Layout::FONT16_W + 4;
    Paint_DrawRectangle(x + 4, y - 2, x + lbl_w, y + 2, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawString_EN(x + 6, y - 7, label, &Font16, WHITE, BLACK);
    if (icon) icon(x + 10, y + 10);
    Paint_DrawString_EN(x + 28, y + 12, value, &Font20, WHITE, BLACK);
}

// Speech bubble: rectangle with pointer toward mascot (left side)
static void drawSpeechBubble(int x, int y, int w, int h) {
    Paint_DrawRectangle(x, y, x + w, y + h, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    // Triangle pointer on left edge
    int py = y + h / 2;
    Paint_DrawLine(x, py - 3, x - 6, py, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x - 6, py, x, py + 3, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    // Erase wall between pointer roots
    Paint_DrawLine(x, py - 2, x, py + 2, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

// Block number display
static void drawBlockNumber(int x, int y, sFONT *font) {
    if (killswitch.block_number <= 0) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "Block: #%d", killswitch.block_number);
    Paint_DrawString_EN(x, y, buf, font, WHITE, BLACK);
}

// QR code helper: draw killswitch.address at position with px_size per module
static void drawQR(int qr_x, int qr_y, int px_sz) {
    if (!killswitch.address[0]) return;
    QRCode qrcode;
    uint8_t qrcodeData[qrcode_getBufferSize(6)];
    qrcode_initText(&qrcode, qrcodeData, 6, ECC_LOW, killswitch.address);
    int qr_size = qrcode.size;
    int qr_px = qr_size * px_sz;
    Paint_DrawRectangle(qr_x - 2, qr_y - 2, qr_x + qr_px + 2, qr_y + qr_px + 2,
                        WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    for (int y = 0; y < qr_size; y++)
        for (int x = 0; x < qr_size; x++)
            if (qrcode_getModule(&qrcode, x, y))
                Paint_DrawRectangle(qr_x + x * px_sz, qr_y + y * px_sz,
                                    qr_x + (x + 1) * px_sz - 1, qr_y + (y + 1) * px_sz - 1,
                                    BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

// ─── Screen 0: SOUL (identity + presence) ─────────────────────

static void renderSoulPage() {
    char buf[48];

    // Mascot bitmap — left column
    const unsigned char *mascot = hasAnyProblem() ? hiki_worried : hiki_normal;
    Paint_DrawImage(mascot, 0, 8, MASCOT_W, MASCOT_H);

    // Name badge above mascot area
    Paint_DrawString_EN(10, 10, "<HIKI>", &Font16, WHITE, BLACK);

    // Vertical dotted separator between mascot and data
    for (int y = 8; y < 200; y += 3)
        Paint_SetPixel(155, y, BLACK);

    // Speech bubble — right of mascot, top (max 19 chars fit)
    const char *msg = getPersonalityMessage();
    int msg_len = strlen(msg);
    if (msg_len > 19) msg_len = 19;
    int bw = msg_len * Layout::FONT16_W + 12;
    if (bw < 100) bw = 100;
    drawSpeechBubble(165, 8, bw, 24);
    Paint_DrawString_EN(171, 12, msg, &Font16, WHITE, BLACK);

    // T/H in dotted frame
    int tx = 165, ty = 42;
    Paint_DrawLine(tx, ty, tx + 220, ty, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawLine(tx, ty + 20, tx + 220, ty + 20, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    if (sensor.ok) {
        drawIconThermo(tx + 4, ty + 2);
        snprintf(buf, sizeof(buf), "%.1fC", sensor.temp);
        Paint_DrawString_EN(tx + 18, ty + 3, buf, &Font16, WHITE, BLACK);
        drawIconDrop(tx + 100, ty + 2);
        snprintf(buf, sizeof(buf), "%.0f%%", sensor.hum);
        Paint_DrawString_EN(tx + 114, ty + 3, buf, &Font16, WHITE, BLACK);
    } else {
        Paint_DrawString_EN(tx + 4, ty + 3, "Sensors: --", &Font16, WHITE, BLACK);
    }

    // Model badge (max 18 chars to fit within MARGIN_R)
    char model_buf[20];
    if (health.model[0]) {
        snprintf(model_buf, sizeof(model_buf), "%.18s", health.model);
    } else {
        strcpy(model_buf, "---");
    }
    drawBadge(165, 68, model_buf, &Font16);

    // Uptime + memory
    int sy = 90;
    snprintf(buf, sizeof(buf), "up:%s  mem:%dM",
             health.received && health.up[0] ? health.up : "--",
             health.received ? health.mem : 0);
    Paint_DrawString_EN(165, sy, buf, &Font16, WHITE, BLACK);

    // Messages
    if (health.received)
        snprintf(buf, sizeof(buf), "%d msg/24h", health.msgs_24h);
    else
        snprintf(buf, sizeof(buf), "-- msg/24h");
    Paint_DrawString_EN(165, sy + 18, buf, &Font16, WHITE, BLACK);

    // Node status circles
    int ny = sy + 40;
    drawNodeCircle(170, ny, health.received && health.gw,   "GW");
    drawNodeCircle(220, ny, health.received,            "AI");
    drawNodeCircle(270, ny, health.received && health.ha,    "HA");
    drawNodeCircle(325, ny, health.received && health.inet,  "NET");

    // ══════ Robonomics address between double lines ══════
    int addr_y = 206;
    drawDoubleLine(addr_y);
    drawAddress(12, addr_y + 8, killswitch.address, &Font16);
    Paint_DrawString_EN(210, addr_y + 8, "ROBONOMICS ID", &Font16, WHITE, BLACK);
    drawDoubleLine(addr_y + 26);

    // CO2 bar + status bottom row
    int bot_y = 240;
    if (sensor.ok) {
        snprintf(buf, sizeof(buf), "CO2: %.0f ppm", sensor.co2);
        Paint_DrawString_EN(12, bot_y, buf, &Font16, WHITE, BLACK);
        int co2v = clampedCO2();
        drawProgressBar(170, bot_y + 2, 100, 12, co2v, Layout::CO2_MAX);
        Paint_DrawString_EN(276, bot_y, getCO2Label(), &Font16, WHITE, BLACK);
    }

    // Bottom status line
    int boty2 = 260;
    snprintf(buf, sizeof(buf), "HA:%s  KS:%s",
             health.received ? (health.ha_api ? "ok" : "!") : "--",
             killswitch.received ? killswitch.state : "--");
    Paint_DrawString_EN(12, boty2, buf, &Font16, WHITE, BLACK);

    drawWiFiStatus(316, boty2);
}

// ─── Screen 1: BREATH (environment + senses) ──────────────────

static void renderBreathPage() {
    char buf[48];

    drawCyberHeader(8, "ENVIRONMENT SCAN");
    drawDoubleLine(32);

    if (sensor.ok) {
        // CO2 hero number — large, centered
        snprintf(buf, sizeof(buf), "CO2  %.0f  ppm", sensor.co2);
        // Center the text
        int tw = strlen(buf) * Layout::FONT24_W;
        Paint_DrawString_EN((DISPLAY_W - tw) / 2, 42, buf, &Font24, WHITE, BLACK);

        // Triple-frame progress bar
        int bx = 20, by = 72, bw = 360, bh = 18;
        Paint_DrawRectangle(bx, by, bx + bw, by + bh, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        Paint_DrawRectangle(bx + 2, by + 2, bx + bw - 2, by + bh - 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        // Fill
        int co2v = clampedCO2();
        int fill_w = (co2v * (bw - 6)) / Layout::CO2_MAX;
        if (fill_w > 0)
            Paint_DrawRectangle(bx + 3, by + 3, bx + 3 + fill_w, by + bh - 3,
                                BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        // Quality label — right-aligned inverted badge
        const char *label = getCO2Label();
        int lbx = Layout::MARGIN_R - (int)strlen(label) * Layout::FONT16_W - 8;
        drawBadge(lbx, by + bh + 4, label, &Font16);

        // Dotted separator (below badge bottom at y~112)
        drawDottedLine(114);

        // Thermal + Moisture panels
        snprintf(buf, sizeof(buf), "%.1f C", sensor.temp);
        drawLabeledPanel(20, 120, 170, 40, "THERMAL", drawIconThermo, buf);
        snprintf(buf, sizeof(buf), "%.0f %%", sensor.hum);
        drawLabeledPanel(210, 120, 170, 40, "MOISTURE", drawIconDrop, buf);

        // Dotted separator
        drawDottedLine(168);
    } else {
        Paint_DrawString_EN(100, 60, "Sensors: offline", &Font20, WHITE, BLACK);
        drawDottedLine(168);
    }

    // ── SYSTEM VITALS section ──
    drawCyberHeader(174, "SYSTEM VITALS");
    drawDoubleLine(198);

    // Uptime, memory, disk with icon labels
    int vy = 206;
    drawIconClock(12, vy);
    Paint_DrawString_EN(28, vy + 2, health.received && health.up[0] ? health.up : "--", &Font16, WHITE, BLACK);

    if (health.received) {
        snprintf(buf, sizeof(buf), "[mem] %dM", health.mem);
        Paint_DrawString_EN(130, vy + 2, buf, &Font16, WHITE, BLACK);
        snprintf(buf, sizeof(buf), "[dsk] %d%%", health.disk);
        Paint_DrawString_EN(270, vy + 2, buf, &Font16, WHITE, BLACK);
    }

    // AI status line
    int ay = vy + 20;
    bool ai_isolated = (strcmp(killswitch.state, "isolated") == 0);
    if (ai_isolated) {
        drawBadge(Layout::MARGIN_L, ay, "AI:ISOLATED", &Font16);
    } else if (health.received) {
        snprintf(buf, sizeof(buf), "AI:ok %dmsg %.10s", health.msgs_24h, health.model[0] ? health.model : "");
        Paint_DrawString_EN(12, ay + 1, buf, &Font16, WHITE, BLACK);
    } else {
        Paint_DrawString_EN(12, ay + 1, "AI: --", &Font16, WHITE, BLACK);
    }

    // HA, GW, NET status
    int sy = ay + 20;
    drawNodeStatusLine(Layout::MARGIN_L, sy);

    // Bottom double line + Web3/KS/WiFi
    drawDoubleLine(sy + 20);
    int bly = sy + 28;
    snprintf(buf, sizeof(buf), "Web3:%s  KS:%s",
             killswitch.ws_connected ? "ok" : "--",
             killswitch.received ? killswitch.state : "--");
    Paint_DrawString_EN(12, bly, buf, &Font16, WHITE, BLACK);

    int rssi = WiFi.RSSI();
    snprintf(buf, sizeof(buf), "WiFi:%ddB", rssi);
    Paint_DrawString_EN(290, bly, buf, &Font16, WHITE, BLACK);
}

// ─── Screen 2: NERVES (network topology) ──────────────────────

// Node box: double border when online, single when offline
static void drawNodeBox(int x, int y, int w, int h, bool online) {
    Paint_DrawRectangle(x, y, x + w, y + h, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    if (online)
        Paint_DrawRectangle(x + 2, y + 2, x + w - 2, y + h - 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
}

// 3-line node card: box with 3 text lines inside
static void drawNodeCard(int cx, int y, int w, int h, bool online,
                         const char *line1, const char *line2, const char *line3) {
    int x = cx - w / 2;
    drawNodeBox(x, y, w, h, online);
    Paint_DrawString_EN(x + 6, y + 4,  line1, &Font16, WHITE, BLACK);
    Paint_DrawString_EN(x + 6, y + 20, line2, &Font16, WHITE, BLACK);
    Paint_DrawString_EN(x + 6, y + 34, line3, &Font16, WHITE, BLACK);
}

// Connection line with junction dot
static void drawLink(int x1, int y1, int x2, int y2, bool healthy) {
    Paint_DrawLine(x1, y1, x2, y2, BLACK,
                   healthy ? DOT_PIXEL_2X2 : DOT_PIXEL_1X1,
                   healthy ? LINE_STYLE_SOLID : LINE_STYLE_DOTTED);
}

static void renderNervePage() {
    char buf[48];

    // Header
    drawCyberHeader(8, "NERVE MAP");

    // WiFi RSSI in header right
    drawWiFiStatus(316, 10);

    drawDoubleLine(32);

    bool inet_ok = health.received && health.inet;
    bool gw_ok = health.received && health.gw;
    bool ha_ok = health.received && health.ha;

    // ── INTERNET node (top center) ──
    int inet_x = 140, inet_y = 40, inet_w = 120, inet_h = 28;
    drawNodeBox(inet_x, inet_y, inet_w, inet_h, inet_ok);
    Paint_DrawString_EN(inet_x + 10, inet_y + 6, "INTERNET", &Font16, WHITE, BLACK);
    if (health.received) {
        snprintf(buf, sizeof(buf), "%dms", health.inet_ms);
        Paint_DrawString_EN(inet_x + inet_w + 4, inet_y + 6, buf, &Font16, WHITE, BLACK);
    }

    // Connection: Internet → Gateway
    int cx = 200;
    int link1_top = inet_y + inet_h;
    int link1_bot = inet_y + inet_h + 16;
    drawLink(cx, link1_top, cx, link1_bot, inet_ok);
    // Junction dot
    Paint_DrawCircle(cx, link1_top, 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    // Latency label on link
    if (health.received) {
        snprintf(buf, sizeof(buf), "%dms", health.inet_ms > 0 ? health.inet_ms : 0);
    }

    // ── GATEWAY node ──
    int gw_w = 150, gw_h = 42;
    int gw_x = cx - gw_w / 2, gw_y = link1_bot;
    drawNodeBox(gw_x, gw_y, gw_w, gw_h, gw_ok);
    Paint_DrawString_EN(gw_x + 6, gw_y + 4, "GATEWAY", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(gw_x + 6, gw_y + 20, "10.0.0.1 OPi", &Font16, WHITE, BLACK);
    // Web3 status beside
    Paint_DrawString_EN(gw_x + gw_w + 6, gw_y + 12, killswitch.ws_connected ? "Web3:ok" : "Web3:--", &Font16, WHITE, BLACK);

    // ── Branch: GW → Agent and GW → HA ──
    int gw_bot = gw_y + gw_h;
    int agent_cx = 90, ha_cx = 310;
    int branch_y = gw_bot + 10;

    drawLink(cx, gw_bot, cx, branch_y - 4, gw_ok);
    Paint_DrawCircle(cx, gw_bot, 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    // Horizontal branch line
    Paint_DrawLine(agent_cx, branch_y, ha_cx, branch_y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    // Vertical drops to child nodes
    int child_y = branch_y + 14;
    drawLink(agent_cx, branch_y, agent_cx, child_y, gw_ok);
    drawLink(ha_cx, branch_y, ha_cx, child_y, ha_ok);

    // Latency labels
    if (health.received) {
        snprintf(buf, sizeof(buf), "%dms", health.gw_ms);
        Paint_DrawString_EN(agent_cx - 28, branch_y - 14, buf, &Font16, WHITE, BLACK);
        snprintf(buf, sizeof(buf), "%dms", health.ha_ms);
        Paint_DrawString_EN(ha_cx + 6, branch_y - 14, buf, &Font16, WHITE, BLACK);
    }

    // ── AI AGENT + SMART HOME nodes ──
    int ag_w = 120, ag_h = 48;
    char model_trunc[12];
    snprintf(model_trunc, sizeof(model_trunc), "%.9s", health.model[0] ? health.model : "---");
    drawNodeCard(agent_cx, child_y, ag_w, ag_h, health.received,
                 "AI AGENT", "10.0.0.2", model_trunc);
    drawNodeCard(ha_cx, child_y, ag_w, ag_h, ha_ok,
                 "SMART HOME", "10.0.0.3", "HA+MQTT");

    // ── Converge to TORII-INK ──
    int torii_cx = 200;
    int torii_y = child_y + ag_h + 16;
    // Diagonal links from agent and HA down to torii
    drawLink(agent_cx, child_y + ag_h, torii_cx - 30, torii_y, true);
    drawLink(ha_cx, child_y + ag_h, torii_cx + 30, torii_y, true);

    // TORII-INK self node
    int tw = 130, th = 28;
    drawNodeBox(torii_cx - tw / 2, torii_y, tw, th, true);
    Paint_DrawString_EN(torii_cx - tw / 2 + 6, torii_y + 6, "TORII-INK", &Font16, WHITE, BLACK);
    // "[this]" marker
    Paint_DrawString_EN(torii_cx + tw / 2 - 44, torii_y + 6, "MQTT", &Font16, WHITE, BLACK);
}

// ─── Screen 3: SHELL (killswitch + blockchain) ────────────────

// Shield icon: pointed bottom, filled or outline
static void drawShield(int cx, int cy, int s, bool filled) {
    int w = s * 3 / 4;
    int top = cy - s, mid = cy + s / 3, bot = cy + s;
    Paint_DrawLine(cx - w, top, cx + w, top, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(cx - w, top, cx - w, mid, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(cx + w, top, cx + w, mid, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(cx - w, mid, cx, bot, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(cx + w, mid, cx, bot, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    if (filled) {
        for (int y = top + 2; y < mid; y++)
            Paint_DrawLine(cx - w + 2, y, cx + w - 2, y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        for (int y = mid; y < bot; y++) {
            int narrow = w * (y - mid) / (bot - mid);
            int lx = cx - w + narrow + 2, rx = cx + w - narrow - 2;
            if (lx < rx) Paint_DrawLine(lx, y, rx, y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        }
    }
}

// White checkmark inside a filled shape
static void drawCheckWhite(int cx, int cy, int s) {
    Paint_DrawLine(cx - s, cy, cx - s / 3, cy + s / 2, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(cx - s / 3, cy + s / 2, cx + s, cy - s * 2 / 3, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
}

// Warning triangle with exclamation mark
static void drawWarning(int cx, int top_y, int h) {
    int w = h * 2 / 3;
    int bot = top_y + h;
    Paint_DrawLine(cx, top_y, cx - w, bot, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(cx, top_y, cx + w, bot, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(cx - w, bot, cx + w, bot, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawLine(cx, top_y + h / 3, cx, bot - h / 3, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawCircle(cx, bot - 4, 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

// Connection topology: [ROB]---[GW]---[Agent]---[HA], optional X marks
static void drawTopology(int y, bool broken) {
    int nodes[] = {50, 150, 260, 360};
    const char *labels[] = {"ROB", "GW", "Agent", "HA"};

    for (int i = 0; i < 4; i++) {
        Paint_DrawString_EN(nodes[i] - (int)strlen(labels[i]) * Layout::FONT16_W / 2, y - 20,
                            labels[i], &Font16, WHITE, BLACK);
        bool on = (i == 0) || !broken;
        Paint_DrawCircle(nodes[i], y, 5, BLACK, DOT_PIXEL_1X1,
                         on ? DRAW_FILL_FULL : DRAW_FILL_EMPTY);
    }

    for (int i = 0; i < 3; i++) {
        int x1 = nodes[i] + 7, x2 = nodes[i + 1] - 7;
        if (broken && i >= 1) {
            int mx = (x1 + x2) / 2;
            Paint_DrawLine(x1, y, mx - 8, y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
            Paint_DrawLine(mx - 5, y - 5, mx + 5, y + 5, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
            Paint_DrawLine(mx - 5, y + 5, mx + 5, y - 5, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
            Paint_DrawLine(mx + 8, y, x2, y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
        } else {
            Paint_DrawLine(x1, y, x2, y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        }
    }
}

static void renderShellPage() {
    if (!killswitch.received) {
        drawCyberHeader(8, "SHELL");
        Paint_DrawString_EN(100, 100, "Waiting for", &Font20, WHITE, BLACK);
        Paint_DrawString_EN(80, 130, "killswitch data...", &Font20, WHITE, BLACK);
        return;
    }

    bool connected = (strcmp(killswitch.state, "connected") == 0);

    if (connected) {
        // ── CONNECTED: calm, reassuring ──
        drawCyberHeader(8, "SHELL");
        drawShield(370, 18, 12, true);

        // QR on left
        drawQR(20, 38, 3);

        // Shield + checkmark — dominant visual
        drawShield(260, 72, 28, true);
        drawCheckWhite(260, 74, 10);

        // Status banner (inverted)
        Paint_DrawRectangle(170, 106, 380, 132, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawString_EN(188, 108, "CONNECTED", &Font20, BLACK, WHITE);

        Paint_DrawString_EN(170, 140, "Agent has full", &Font16, WHITE, BLACK);
        Paint_DrawString_EN(170, 158, "network access.", &Font16, WHITE, BLACK);

        // Block number + address
        drawDoubleLine(178);
        drawBlockNumber(20, 186, &Font16);
        drawAddress(200, 186, killswitch.address, &Font16);
        drawDoubleLine(204);

        // Topology
        Paint_DrawLine(20, 212, 380, 212, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
        drawTopology(236, false);

        // WS status
        char ws_buf[32];
        snprintf(ws_buf, sizeof(ws_buf), "WS: %s", killswitch.ws_connected ? "connected" : "disconnected");
        Paint_DrawString_EN(20, 260, ws_buf, &Font16, WHITE, BLACK);

    } else {
        // ── ISOLATED: dramatic, alarming ──
        // Full black banner header
        Paint_DrawRectangle(0, 0, 399, 50, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawRectangle(2, 2, 397, 48, WHITE, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);

        // Warning triangles in banner (white)
        Paint_DrawLine(24, 12, 10, 38, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
        Paint_DrawLine(24, 12, 38, 38, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
        Paint_DrawLine(10, 38, 38, 38, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);

        Paint_DrawLine(376, 12, 362, 38, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
        Paint_DrawLine(376, 12, 390, 38, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
        Paint_DrawLine(362, 38, 390, 38, WHITE, DOT_PIXEL_2X2, LINE_STYLE_SOLID);

        // "ISOLATED" white on black
        Paint_DrawString_EN(132, 14, "ISOLATED", &Font24, BLACK, WHITE);

        // QR on left
        drawQR(20, 60, 2);

        // Warning triangle + explanation
        drawWarning(170, 58, 30);
        Paint_DrawString_EN(200, 60, "AGENT CUT OFF", &Font20, WHITE, BLACK);

        // Traffic dropped banner
        drawBadge(200, 86, "Traffic: DROPPED", &Font16);

        // Details box
        Paint_DrawRectangle(20, 120, 380, 168, BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
        drawBlockNumber(30, 126, &Font20);
        if (killswitch.isolated_at[0]) {
            char iso_buf[40];
            snprintf(iso_buf, sizeof(iso_buf), "Isolated at: %s", killswitch.isolated_at);
            Paint_DrawString_EN(30, 148, iso_buf, &Font16, WHITE, BLACK);
        }

        // Broken topology
        drawTopology(196, true);

        // Restore instructions
        Paint_DrawString_EN(20, 220, "To restore, send:", &Font16, WHITE, BLACK);
        drawBadge(20, 238, "Launch(param=true)", &Font16);

        // Inverted bottom bar
        Paint_DrawRectangle(0, 270, 399, 299, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawString_EN(72, 276, "KILLSWITCH ACTIVE", &Font24, BLACK, WHITE);
    }
}

// ─── Screen rendering ──────────────────────────────────────────

static void renderScreen(bool full_refresh) {
    if (!framebuffer) return;

    Paint_SelectImage(framebuffer);
    Paint_Clear(WHITE);
    drawCornerBrackets();

    switch (app.current_screen) {
        case 0: renderSoulPage();    break;
        case 1: renderBreathPage();  break;
        case 2: renderNervePage();   break;
        case 3: renderShellPage();   break;
    }

    if (full_refresh) {
        // Deep clean: full init + full waveform
        EPD_4IN2_V2_Init();
        EPD_4IN2_V2_Display(framebuffer);
        // Switch back to fast mode for subsequent updates
        EPD_4IN2_V2_Init_Fast(Seconds_1_5S);
    } else {
        // Fast refresh: writes both registers, no ghosting, minimal flicker
        EPD_4IN2_V2_Display_Fast(framebuffer);
    }
}

// ─── Main ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== TORII-INK ===");

    pinMode(BTN_PIN, INPUT_PULLUP);

    initDisplay();
    initSensors();
    initWiFi();

    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setBufferSize(512);
    mqtt.setCallback(mqttCallback);
    connectMQTT();

    // Watchdog: 120s covers worst-case e-ink refresh (6x ReadBusy 10s each)
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 120000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_reconfigure(&wdt_config);
    esp_task_wdt_add(NULL);

    Serial.println("Waiting for SCD4x first reading...");
    bool got_reading = false;
    for (int i = 0; i < 15; i++) {
        delay(1000);
        mqtt.loop();
        esp_task_wdt_reset();
        if (readSCD4x()) {
            got_reading = true;
            break;
        }
        Serial.printf("  ...waiting (%d/15)\n", i + 1);
    }
    if (!got_reading) {
        Serial.println("No SCD4x data yet, will retry in loop.");
    }

    readSensors();
    publishSensors();
    app.current_screen = 0;
    renderScreen(true);
    Serial.println("Display updated (full).");
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi: reconnecting...");
        if (mqtt.connected()) {
            mqtt.disconnect();
        }
        WiFi.disconnect(true);
        delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
            delay(500);
            esp_task_wdt_reset();
        }
    }
    connectMQTT();

    bool button_pressed = false;
    static bool btn_prev = HIGH;
    static unsigned long debounce_time = 0;

    for (unsigned long elapsed = 0; elapsed < CYCLE_MS; elapsed += 100) {
        mqtt.loop();
        delay(100);
        esp_task_wdt_reset();

        bool btn_now = digitalRead(BTN_PIN);
        if (btn_prev == HIGH && btn_now == LOW && (millis() - debounce_time) > DEBOUNCE_MS) {
            debounce_time = millis();
            button_pressed = true;
            break;
        }
        btn_prev = btn_now;
    }
    app.cycle_count++;

    if (app.cycle_count % SENSOR_EVERY_N == 0) {
        readSensors();
        publishSensors();
    }

    app.current_screen = (app.current_screen + 1) % SCREEN_COUNT;

    bool full = (app.cycle_count % FULL_REFRESH_EVERY_N == 0);
    renderScreen(full);
    Serial.printf("Screen %d (%s%s).\n", app.current_screen,
                  full ? "full" : "partial",
                  button_pressed ? ", btn" : "");
    esp_task_wdt_reset();
}
