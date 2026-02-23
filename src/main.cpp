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

// Hardware buttons (active LOW, external pull-up)
#define BTN_UP        10
#define BTN_SET       2
#define BTN_DOWN      3
#define DEBOUNCE_MS   50

// Display
#define DISPLAY_W EPD_4IN2_V2_WIDTH   // 400
#define DISPLAY_H EPD_4IN2_V2_HEIGHT  // 300

static UBYTE *framebuffer = nullptr;

// ─── Navigation ─────────────────────────────────────────────────

enum Screen { HOME, ISOLATED, ISOLATED_HOME, DETAIL_BREATH, DETAIL_NERVE };

#define DETAIL_TIMEOUT_MS   25000       // auto-return from detail screens
#define SENSOR_INTERVAL_MS  120000      // read+publish sensors
#define HOME_REFRESH_MS     60000       // re-render home with fresh data
#define FULL_REFRESH_EVERY  5           // full e-ink waveform every N transitions

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

struct NavState {
    Screen        screen            = HOME;
    unsigned long last_transition   = 0;
    unsigned long last_sensor       = 0;
    unsigned long last_home_refresh = 0;
    int           fast_count        = 0;
};

static SCD4x           scd4x_driver;
static SensorData      sensor;
static HealthState     health;
static KillswitchState killswitch;
static GatewayHealth   gw_health;
static NavState        nav;
static bool            ks_changed = false;

// ─── Layout constants ─────────────────────────────────────────

namespace Layout {
    constexpr int MARGIN_L   = 12;
    constexpr int MARGIN_R   = 388;
    constexpr int FONT16_W   = 11;
    constexpr int FONT20_W   = 14;
    constexpr int FONT24_W   = 17;
    constexpr int CO2_MAX    = 2000;
    constexpr int RIGHT_COL  = 160;
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
        ks_changed = true;
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

static bool isIsolated() {
    return strcmp(killswitch.state, "isolated") == 0;
}

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
    bool node_down = health.received && (!health.ha || !health.gw || !health.inet);
    bool co2_high = sensor.ok && sensor.co2 > 1000;
    return isIsolated() || node_down || co2_high;
}

static const char *getPersonalityMessage() {
    if (isIsolated()) return "Cut off from world";
    if (health.received && (!health.ha || !health.gw || !health.inet)) return "Something is off...";
    if (sensor.ok && sensor.co2 > 1500) return "Open a window pls?";
    if (sensor.ok && sensor.co2 > 1000) return "Air getting stuffy.";
    if (health.received && health.msgs_24h == 0) return "It's quiet today.";
    if (health.received && health.msgs_24h > 10) return "Busy day!";
    if (health.up[0] == '0') return "Just woke up...";
    return "All systems nominal.";
}

// ─── Drawing helpers ───────────────────────────────────────────

// Corner brackets on 4 corners of display
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

// Double horizontal line
static void drawDoubleLine(int y, int x1 = 8, int x2 = 392) {
    Paint_DrawLine(x1, y, x2, y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x1, y + 3, x2, y + 3, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

// Draw truncated SS58 address
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

// ─── Icon helpers ─────────────────────────────────────────────

static void drawIconThermo(int x, int y) {
    Paint_DrawRectangle(x + 3, y, x + 7, y + 9, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawRectangle(x + 4, y + 4, x + 6, y + 9, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawCircle(x + 5, y + 13, 3, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

static void drawIconDrop(int x, int y) {
    Paint_DrawLine(x + 5, y, x + 1, y + 8, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x + 5, y, x + 9, y + 8, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawCircle(x + 5, y + 10, 4, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
}

static void drawIconClock(int x, int y) {
    int cx = x + 7, cy = y + 7;
    Paint_DrawCircle(cx, cy, 6, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawLine(cx, cy, cx + 3, cy - 4, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(cx, cy, cx, cy - 5, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawCircle(cx, cy, 1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

// Chip icon for AI Agent (16×16)
static void drawIconAgent(int x, int y, bool online) {
    // Chip body
    Paint_DrawRectangle(x+4, y+2, x+11, y+12, BLACK, DOT_PIXEL_1X1,
                        online ? DRAW_FILL_FULL : DRAW_FILL_EMPTY);
    // Side pins (4 pairs)
    for (int i = 0; i < 4; i++) {
        int py = y + 4 + i * 2;
        Paint_DrawLine(x+2, py, x+4, py, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        Paint_DrawLine(x+11, py, x+13, py, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    }
    // Core
    if (online)
        Paint_DrawRectangle(x+6, y+6, x+9, y+8, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    else
        Paint_DrawCircle(x+7, y+7, 1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
}

// House icon for Smart Home (16×16)
static void drawIconHome(int x, int y, bool online) {
    int peak_x = x + 7, peak_y = y + 2;
    int roof_l = x + 2, roof_r = x + 12, roof_base = y + 7;
    // Roof lines
    Paint_DrawLine(peak_x, peak_y, roof_l, roof_base, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(peak_x, peak_y, roof_r, roof_base, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    // Walls
    Paint_DrawRectangle(roof_l, roof_base, roof_r, y+13, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    if (online) {
        // Fill roof
        for (int row = peak_y + 1; row < roof_base; row++) {
            int half_w = row - peak_y;
            Paint_DrawLine(peak_x - half_w, row, peak_x + half_w, row,
                           BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        }
        // Fill walls
        Paint_DrawRectangle(roof_l, roof_base, roof_r, y+13, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        // Door cutout
        Paint_DrawRectangle(x+5, y+10, x+8, y+13, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    } else {
        // Door outline
        Paint_DrawRectangle(x+5, y+10, x+7, y+13, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    }
}

// Router icon for Gateway (16×16)
static void drawIconGateway(int x, int y, bool online) {
    // Device body
    Paint_DrawRectangle(x+2, y+6, x+13, y+12, BLACK, DOT_PIXEL_1X1,
                        online ? DRAW_FILL_FULL : DRAW_FILL_EMPTY);
    // Antennas
    Paint_DrawLine(x+6, y+6, x+2, y+1, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x+9, y+6, x+13, y+1, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    // LED indicators
    UWORD led_color = online ? WHITE : BLACK;
    Paint_SetPixel(x+5, y+9, led_color);
    Paint_SetPixel(x+8, y+9, led_color);
    Paint_SetPixel(x+11, y+9, led_color);
}

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

// Inverted badge: black background, white text
static void drawBadge(int x, int y, const char *text, sFONT *font) {
    int w = strlen(text) * font->Width + 8;
    int h = font->Height + 2;
    if (x + w > DISPLAY_W) w = DISPLAY_W - x;
    Paint_DrawRectangle(x, y, x + w, y + h, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawString_EN(x + 4, y + 1, text, font, BLACK, WHITE);
}

static void drawWiFiStatus(int x, int y) {
    int rssi = WiFi.RSSI();
    char buf[12];
    snprintf(buf, sizeof(buf), "%ddB", rssi);
    drawSignalBars(x, y, rssi);
    Paint_DrawString_EN(x + 22, y + 2, buf, &Font16, WHITE, BLACK);
}

static void drawNodeStatusLine(int x, int y) {
    char buf[48];
    const char *ha_s  = health.received ? (health.ha  ? "ok" : "!") : "--";
    const char *gw_s  = health.received ? (health.gw  ? "ok" : "!") : "--";
    const char *net_s = health.received ? (health.inet ? "ok" : "!") : "--";
    snprintf(buf, sizeof(buf), "HA:%s  GW:%s  NET:%s", ha_s, gw_s, net_s);
    Paint_DrawString_EN(x, y, buf, &Font16, WHITE, BLACK);
}

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

// Speech bubble with pointer toward mascot
static void drawSpeechBubble(int x, int y, int w, int h) {
    Paint_DrawRectangle(x, y, x + w, y + h, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    int py = y + h / 2;
    Paint_DrawLine(x, py - 3, x - 6, py, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x - 6, py, x, py + 3, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x, py - 2, x, py + 2, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

static void drawBlockNumber(int x, int y, sFONT *font) {
    if (killswitch.block_number <= 0) return;
    char buf[32];
    snprintf(buf, sizeof(buf), "Block: #%d", killswitch.block_number);
    Paint_DrawString_EN(x, y, buf, font, WHITE, BLACK);
}

// QR code helper
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

// ─── Screen: HOME ──────────────────────────────────────────────

static void renderHomePage() {
    char buf[48];
    int rx = Layout::RIGHT_COL;  // 160

    // ── Left column: Mascot ──
    const unsigned char *mascot = hasAnyProblem() ? hiki_worried : hiki_normal;
    Paint_DrawImage(mascot, 0, 8, MASCOT_W, MASCOT_H);

    // Vertical dotted separator
    for (int y = 8; y < 198; y += 3)
        Paint_SetPixel(155, y, BLACK);

    // ── Right column: Device Identity ──

    // Speech bubble at top
    const char *msg = getPersonalityMessage();
    int msg_len = strlen(msg);
    if (msg_len > 19) msg_len = 19;
    int bw = msg_len * Layout::FONT16_W + 12;
    if (bw < 100) bw = 100;
    drawSpeechBubble(rx + 5, 8, bw, 24);
    Paint_DrawString_EN(rx + 11, 12, msg, &Font16, WHITE, BLACK);

    // QR code (Robonomics ID)
    drawQR(rx + 4, 36, 3);  // 164,36 — 123×123 (41 modules × 3px)

    // Address below QR
    drawAddress(rx + 4, 163, killswitch.address, &Font16);
    drawBlockNumber(rx + 4, 181, &Font16);

    // ── Separator ──
    drawDoubleLine(198);

    // ── Data section ──

    // Temperature (Font24, prominent)
    if (sensor.ok) {
        drawIconThermo(12, 205);
        snprintf(buf, sizeof(buf), "%.1f C", sensor.temp);
        Paint_DrawString_EN(30, 205, buf, &Font24, WHITE, BLACK);
    } else {
        Paint_DrawString_EN(12, 205, "Temp: --", &Font24, WHITE, BLACK);
    }

    // Connection status with icons: Agent → Home → Gateway
    int iy = 232;  // icon row top
    bool agent_ok = health.received;
    bool home_ok  = health.received && health.ha;
    bool gw_ok    = health.received && health.gw;

    drawIconAgent(12, iy, agent_ok);
    snprintf(buf, sizeof(buf), "Agent:%s", agent_ok ? "ok" : "offline");
    Paint_DrawString_EN(30, iy + 1, buf, &Font16, WHITE, BLACK);

    drawIconHome(140, iy, home_ok);
    snprintf(buf, sizeof(buf), "Home:%s", home_ok ? "ok" : "offline");
    Paint_DrawString_EN(158, iy + 1, buf, &Font16, WHITE, BLACK);

    drawIconGateway(268, iy, gw_ok);
    snprintf(buf, sizeof(buf), "GW:%s", gw_ok ? "ok" : "offline");
    Paint_DrawString_EN(286, iy + 1, buf, &Font16, WHITE, BLACK);

    drawDottedLine(250);

    // Killswitch state: badge only for alarm, plain text otherwise
    bool ks_isolated = isIsolated();
    snprintf(buf, sizeof(buf), "Killswitch: %s",
             killswitch.received ? killswitch.state : "---");
    if (ks_isolated) {
        drawBadge(12, 254, buf, &Font16);
    } else {
        Paint_DrawString_EN(12, 256, buf, &Font16, WHITE, BLACK);
    }
    drawSignalBars(365, 254, WiFi.RSSI());

    // Footer: Web3 chain + uptime + messages
    Paint_DrawString_EN(12, 274, killswitch.ws_connected ? "Web3 chain: ok" : "Web3 chain: --",
                        &Font16, WHITE, BLACK);
    snprintf(buf, sizeof(buf), "up: %.5s  %d msg",
             health.received && health.up[0] ? health.up : "--",
             health.received ? health.msgs_24h : 0);
    Paint_DrawString_EN(220, 274, buf, &Font16, WHITE, BLACK);

    drawDoubleLine(290);
}

// ─── Screen: BREATH (environment detail) ───────────────────────

static void renderBreathPage() {
    char buf[48];

    drawCyberHeader(8, "ENVIRONMENT SCAN");
    drawDoubleLine(32);

    if (sensor.ok) {
        // CO2 hero number
        snprintf(buf, sizeof(buf), "CO2  %.0f  ppm", sensor.co2);
        int tw = strlen(buf) * Layout::FONT24_W;
        Paint_DrawString_EN((DISPLAY_W - tw) / 2, 42, buf, &Font24, WHITE, BLACK);

        // Triple-frame progress bar
        int bx = 20, by = 72, bbar_w = 360, bh = 18;
        Paint_DrawRectangle(bx, by, bx + bbar_w, by + bh, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        Paint_DrawRectangle(bx + 2, by + 2, bx + bbar_w - 2, by + bh - 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        int co2v = clampedCO2();
        int fill_w = (co2v * (bbar_w - 6)) / Layout::CO2_MAX;
        if (fill_w > 0)
            Paint_DrawRectangle(bx + 3, by + 3, bx + 3 + fill_w, by + bh - 3,
                                BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        const char *label = getCO2Label();
        int lbx = Layout::MARGIN_R - (int)strlen(label) * Layout::FONT16_W - 8;
        drawBadge(lbx, by + bh + 4, label, &Font16);

        drawDottedLine(114);

        // Thermal + Moisture panels
        snprintf(buf, sizeof(buf), "%.1f C", sensor.temp);
        drawLabeledPanel(20, 120, 170, 40, "THERMAL", drawIconThermo, buf);
        snprintf(buf, sizeof(buf), "%.0f %%", sensor.hum);
        drawLabeledPanel(210, 120, 170, 40, "MOISTURE", drawIconDrop, buf);

        drawDottedLine(168);
    } else {
        Paint_DrawString_EN(100, 60, "Sensors: offline", &Font20, WHITE, BLACK);
        drawDottedLine(168);
    }

    // SYSTEM VITALS section
    drawCyberHeader(174, "SYSTEM VITALS");
    drawDoubleLine(198);

    int vy = 206;
    drawIconClock(12, vy);
    Paint_DrawString_EN(28, vy + 2, health.received && health.up[0] ? health.up : "--", &Font16, WHITE, BLACK);

    if (health.received) {
        snprintf(buf, sizeof(buf), "[mem] %dM", health.mem);
        Paint_DrawString_EN(130, vy + 2, buf, &Font16, WHITE, BLACK);
        snprintf(buf, sizeof(buf), "[dsk] %d%%", health.disk);
        Paint_DrawString_EN(270, vy + 2, buf, &Font16, WHITE, BLACK);
    }

    int ay = vy + 20;
    if (isIsolated()) {
        drawBadge(Layout::MARGIN_L, ay, "AI:ISOLATED", &Font16);
    } else if (health.received) {
        snprintf(buf, sizeof(buf), "AI:ok %dmsg %.10s", health.msgs_24h, health.model[0] ? health.model : "");
        Paint_DrawString_EN(12, ay + 1, buf, &Font16, WHITE, BLACK);
    } else {
        Paint_DrawString_EN(12, ay + 1, "AI: --", &Font16, WHITE, BLACK);
    }

    int sy = ay + 20;
    drawNodeStatusLine(Layout::MARGIN_L, sy);

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

// ─── Screen: NERVE (network topology detail) ───────────────────

// Node box: double border when online
static void drawNodeBox(int x, int y, int w, int h, bool online) {
    Paint_DrawRectangle(x, y, x + w, y + h, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    if (online)
        Paint_DrawRectangle(x + 2, y + 2, x + w - 2, y + h - 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
}

static void drawNodeCard(int cx, int y, int w, int h, bool online,
                         const char *line1, const char *line2, const char *line3) {
    int x = cx - w / 2;
    drawNodeBox(x, y, w, h, online);
    Paint_DrawString_EN(x + 6, y + 4,  line1, &Font16, WHITE, BLACK);
    Paint_DrawString_EN(x + 6, y + 20, line2, &Font16, WHITE, BLACK);
    Paint_DrawString_EN(x + 6, y + 34, line3, &Font16, WHITE, BLACK);
}

static void drawLink(int x1, int y1, int x2, int y2, bool healthy) {
    Paint_DrawLine(x1, y1, x2, y2, BLACK,
                   healthy ? DOT_PIXEL_2X2 : DOT_PIXEL_1X1,
                   healthy ? LINE_STYLE_SOLID : LINE_STYLE_DOTTED);
}

static void renderNervePage() {
    char buf[48];

    drawCyberHeader(8, "NERVE MAP");
    drawWiFiStatus(316, 10);
    drawDoubleLine(32);

    bool inet_ok = health.received && health.inet;
    bool gw_ok = health.received && health.gw;
    bool ha_ok = health.received && health.ha;

    // INTERNET node (top center)
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
    Paint_DrawCircle(cx, link1_top, 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    // GATEWAY node
    int gw_w = 150, gw_h = 42;
    int gw_x = cx - gw_w / 2, gw_y = link1_bot;
    drawNodeBox(gw_x, gw_y, gw_w, gw_h, gw_ok);
    Paint_DrawString_EN(gw_x + 6, gw_y + 4, "GATEWAY", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(gw_x + 6, gw_y + 20, "10.0.0.1 OPi", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(gw_x + gw_w + 6, gw_y + 12, killswitch.ws_connected ? "Web3:ok" : "Web3:--", &Font16, WHITE, BLACK);

    // Branch: GW → Agent and GW → HA
    int gw_bot = gw_y + gw_h;
    int agent_cx = 90, ha_cx = 310;
    int branch_y = gw_bot + 10;

    drawLink(cx, gw_bot, cx, branch_y - 4, gw_ok);
    Paint_DrawCircle(cx, gw_bot, 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

    Paint_DrawLine(agent_cx, branch_y, ha_cx, branch_y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    int child_y = branch_y + 14;
    drawLink(agent_cx, branch_y, agent_cx, child_y, gw_ok);
    drawLink(ha_cx, branch_y, ha_cx, child_y, ha_ok);

    if (health.received) {
        snprintf(buf, sizeof(buf), "%dms", health.gw_ms);
        Paint_DrawString_EN(agent_cx - 28, branch_y - 14, buf, &Font16, WHITE, BLACK);
        snprintf(buf, sizeof(buf), "%dms", health.ha_ms);
        Paint_DrawString_EN(ha_cx + 6, branch_y - 14, buf, &Font16, WHITE, BLACK);
    }

    // AI AGENT + SMART HOME nodes
    int ag_w = 120, ag_h = 48;
    char model_trunc[12];
    snprintf(model_trunc, sizeof(model_trunc), "%.9s", health.model[0] ? health.model : "---");
    drawNodeCard(agent_cx, child_y, ag_w, ag_h, health.received,
                 "AI AGENT", "10.0.0.2", model_trunc);
    drawNodeCard(ha_cx, child_y, ag_w, ag_h, ha_ok,
                 "SMART HOME", "10.0.0.3", "HA+MQTT");

    // Converge to TORII-INK
    int torii_cx = 200;
    int torii_y = child_y + ag_h + 16;
    drawLink(agent_cx, child_y + ag_h, torii_cx - 30, torii_y, true);
    drawLink(ha_cx, child_y + ag_h, torii_cx + 30, torii_y, true);

    int tbox_w = 130, tbox_h = 28;
    drawNodeBox(torii_cx - tbox_w / 2, torii_y, tbox_w, tbox_h, true);
    Paint_DrawString_EN(torii_cx - tbox_w / 2 + 6, torii_y + 6, "TORII-INK", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(torii_cx + tbox_w / 2 - 44, torii_y + 6, "MQTT", &Font16, WHITE, BLACK);
}

// ─── Screen: ISOLATED (killswitch active) ──────────────────────

// Shield icon
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

// Connection topology
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

static void renderIsolatedPage() {
    if (!killswitch.received) {
        Paint_DrawString_EN(100, 140, "Waiting for data...", &Font16, WHITE, BLACK);
        return;
    }

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

    // Traffic dropped badge
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

// ─── Navigation ────────────────────────────────────────────────

static void transitionTo(Screen to) {
    if (!framebuffer) return;

    Screen from = nav.screen;

    // Decide refresh type
    bool entering_isolation = (to == ISOLATED && from != ISOLATED && from != ISOLATED_HOME);
    bool leaving_isolation  = ((to == HOME) && (from == ISOLATED || from == ISOLATED_HOME));
    bool full = entering_isolation || leaving_isolation || (nav.fast_count % FULL_REFRESH_EVERY == 0);
    nav.fast_count++;

    // Render
    Paint_SelectImage(framebuffer);
    Paint_Clear(WHITE);
    drawCornerBrackets();

    switch (to) {
        case HOME:
        case ISOLATED_HOME:
            renderHomePage();
            break;
        case ISOLATED:
            renderIsolatedPage();
            break;
        case DETAIL_BREATH:
            renderBreathPage();
            break;
        case DETAIL_NERVE:
            renderNervePage();
            break;
    }

    // Refresh display
    if (full) {
        EPD_4IN2_V2_Init();
        EPD_4IN2_V2_Display(framebuffer);
        EPD_4IN2_V2_Init_Fast(Seconds_1_5S);
    } else {
        EPD_4IN2_V2_Display_Fast(framebuffer);
    }
    esp_task_wdt_reset();

    // Update state
    nav.screen = to;
    nav.last_transition = millis();
    if (to == HOME || to == ISOLATED_HOME)
        nav.last_home_refresh = millis();

    Serial.printf("NAV: %d -> %d (%s)\n", from, to, full ? "full" : "fast");
}

// ─── Main ──────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== TORII-INK ===");

    pinMode(BTN_UP,   INPUT_PULLUP);
    pinMode(BTN_SET,  INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);

    initDisplay();
    initSensors();
    initWiFi();

    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setBufferSize(512);
    mqtt.setCallback(mqttCallback);
    connectMQTT();

    // Watchdog: 120s covers worst-case e-ink refresh
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

    nav.last_sensor = millis();
    transitionTo(HOME);
    Serial.println("Setup complete.");
}

void loop() {
    esp_task_wdt_reset();

    // WiFi reconnect
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi: reconnecting...");
        if (mqtt.connected()) mqtt.disconnect();
        WiFi.disconnect(true);
        delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
            delay(500);
            esp_task_wdt_reset();
        }
    }
    connectMQTT();
    mqtt.loop();

    unsigned long now = millis();

    // Button edge detection (3 buttons)
    static bool prev_up = HIGH, prev_set = HIGH, prev_down = HIGH;
    static unsigned long db_up = 0, db_set = 0, db_down = 0;
    static unsigned long dbg_time = 0;
    bool btn_up_pressed = false, btn_set_pressed = false, btn_down_pressed = false;

    bool cur_up   = digitalRead(BTN_UP);
    bool cur_set  = digitalRead(BTN_SET);
    bool cur_down = digitalRead(BTN_DOWN);

    if (prev_up == HIGH && cur_up == LOW && (now - db_up) > DEBOUNCE_MS) {
        db_up = now;
        btn_up_pressed = true;
        Serial.println("BTN_UP: pressed");
    }
    if (prev_set == HIGH && cur_set == LOW && (now - db_set) > DEBOUNCE_MS) {
        db_set = now;
        btn_set_pressed = true;
        Serial.println("BTN_SET: pressed");
    }
    if (prev_down == HIGH && cur_down == LOW && (now - db_down) > DEBOUNCE_MS) {
        db_down = now;
        btn_down_pressed = true;
        Serial.println("BTN_DOWN: pressed");
    }
    prev_up = cur_up;
    prev_set = cur_set;
    prev_down = cur_down;
    (void)btn_set_pressed;  // reserved for future use

    // Debug: print button GPIO state every 3 seconds
    if (now - dbg_time >= 3000) {
        dbg_time = now;
        Serial.printf("DBG: UP=%d SET=%d DOWN=%d\n", cur_up, cur_set, cur_down);
    }

    // Periodic sensor read + MQTT publish
    if (now - nav.last_sensor >= SENSOR_INTERVAL_MS) {
        readSensors();
        publishSensors();
        nav.last_sensor = now;
    }

    // Check isolation state
    bool isolated = isIsolated();

    // Handle killswitch state change
    if (ks_changed) {
        ks_changed = false;
        if (isolated && nav.screen != ISOLATED && nav.screen != ISOLATED_HOME) {
            transitionTo(ISOLATED);
            return;
        }
        if (!isolated && (nav.screen == ISOLATED || nav.screen == ISOLATED_HOME)) {
            transitionTo(HOME);
            return;
        }
    }

    // Cyclic screen navigation
    // Normal:   HOME → DETAIL_BREATH → DETAIL_NERVE → (wrap) HOME
    // Isolated: ISOLATED → ISOLATED_HOME → DETAIL_BREATH → DETAIL_NERVE → (wrap) ISOLATED

    // Screen order tables
    static const Screen cycle_normal[]   = { HOME, DETAIL_BREATH, DETAIL_NERVE };
    static const int    cycle_normal_n   = 3;
    static const Screen cycle_isolated[] = { ISOLATED, ISOLATED_HOME, DETAIL_BREATH, DETAIL_NERVE };
    static const int    cycle_isolated_n = 4;

    auto cycleNext = [](const Screen *arr, int n, Screen cur) -> Screen {
        for (int i = 0; i < n; i++)
            if (arr[i] == cur) return arr[(i + 1) % n];
        return arr[0];
    };
    auto cyclePrev = [](const Screen *arr, int n, Screen cur) -> Screen {
        for (int i = 0; i < n; i++)
            if (arr[i] == cur) return arr[(i - 1 + n) % n];
        return arr[0];
    };

    const Screen *cycle = isolated ? cycle_isolated : cycle_normal;
    int cycle_n          = isolated ? cycle_isolated_n : cycle_normal_n;

    // State machine
    unsigned long elapsed = now - nav.last_transition;

    if (btn_up_pressed) {
        transitionTo(cycleNext(cycle, cycle_n, nav.screen));
    } else if (btn_down_pressed) {
        transitionTo(cyclePrev(cycle, cycle_n, nav.screen));
    } else {
        // Auto-behaviors (no button pressed)
        switch (nav.screen) {
            case HOME:
                if (now - nav.last_home_refresh >= HOME_REFRESH_MS) {
                    readSensors();
                    transitionTo(HOME);
                }
                break;
            case DETAIL_BREATH:
            case DETAIL_NERVE:
                if (elapsed >= DETAIL_TIMEOUT_MS) {
                    transitionTo(isolated ? ISOLATED : HOME);
                }
                break;
            case ISOLATED:
            case ISOLATED_HOME:
                // No auto-swap — only manual navigation
                break;
        }
    }

    delay(100);
}
