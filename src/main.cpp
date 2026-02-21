#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SparkFun_SCD4x_Arduino_Library.h>

#include <qrcode.h>

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

// Sensors
static SCD4x scd4x;
static bool scd4x_ok = false;

// Sensor values
static float co2 = 0;
static float scd_temp = 0;
static float scd_hum = 0;

// Network
static WiFiClient wifiClient;
static PubSubClient mqtt(wifiClient);
static bool mqtt_connected = false;

// MQTT topics
static const char *TOPIC_CO2  = DEVICE_ID "/sensor/co2";
static const char *TOPIC_TEMP = DEVICE_ID "/sensor/temperature";
static const char *TOPIC_HUM  = DEVICE_ID "/sensor/humidity";
static const char *TOPIC_DISPLAY = "torii/display/update";
static const char *TOPIC_HEALTH = "hiki/health";
static const char *TOPIC_KILLSWITCH = "hiki/killswitch/status";
static const char *TOPIC_GW_HEALTH = "hiki/gateway/health";

// Killswitch data from hiki-gateway
static bool ks_received = false;
static char ks_state[16] = "unknown";
static char ks_address[64] = "";
static bool ks_ws_connected = false;
static char ks_isolated_at[8] = "";
static int ks_block_number = 0;

// Gateway health data
static bool gw_health_received = false;
static int gw_ha_errors = 0;
static bool gw_ha_reachable = false;

// Health data from hiki-agent
static bool health_received = false;
static int h_ha = 0, h_gw = 0, h_inet = 0, h_ha_api = 0;
static int h_ha_ms = 0, h_gw_ms = 0, h_inet_ms = 0;
static char h_up[16] = "";
static int h_mem = 0, h_disk = 0;
static int h_msgs_24h = 0;
static char h_model[24] = "";

// Screen cycling: 0=mascot, 1=dashboard, 2=network, 3=killswitch
static int current_screen = 0;
static int cycle_count = 0;
#define SCREEN_COUNT      4
#define CYCLE_MS          10000       // switch screens every 10s
#define SENSOR_EVERY_N    6           // read+publish sensors every 6 cycles (60s)
#define FULL_REFRESH_EVERY_N 5        // full e-ink refresh every 5 cycles (~50s)

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

    if (strcmp(topic, TOPIC_HEALTH) == 0 && length < 512) {
        char buf[512];
        memcpy(buf, payload, length);
        buf[length] = '\0';

        h_ha = jsonInt(buf, "ha");
        h_gw = jsonInt(buf, "gw");
        h_inet = jsonInt(buf, "inet");
        h_ha_api = jsonInt(buf, "ha_api");
        h_ha_ms = jsonInt(buf, "ha_ms");
        h_gw_ms = jsonInt(buf, "gw_ms");
        h_inet_ms = jsonInt(buf, "inet_ms");
        h_mem = jsonInt(buf, "mem");
        h_disk = jsonInt(buf, "disk");
        h_msgs_24h = jsonInt(buf, "msgs_24h");
        jsonStr(buf, "up", h_up, sizeof(h_up));
        jsonStr(buf, "model", h_model, sizeof(h_model));

        health_received = true;
        Serial.println("Health data parsed OK");
    }

    if (strcmp(topic, TOPIC_KILLSWITCH) == 0 && length < 256) {
        char buf[256];
        memcpy(buf, payload, length);
        buf[length] = '\0';

        jsonStr(buf, "state", ks_state, sizeof(ks_state));
        jsonStr(buf, "address", ks_address, sizeof(ks_address));
        ks_ws_connected = jsonBool(buf, "ws_connected");
        jsonStr(buf, "isolated_at", ks_isolated_at, sizeof(ks_isolated_at));
        ks_block_number = jsonInt(buf, "block_number");
        ks_received = true;
        Serial.printf("Killswitch: state=%s ws=%d addr=%s\n",
                       ks_state, ks_ws_connected, ks_address);
    }

    if (strcmp(topic, TOPIC_GW_HEALTH) == 0 && length < 256) {
        char buf[256];
        memcpy(buf, payload, length);
        buf[length] = '\0';

        gw_ha_errors = jsonInt(buf, "ha_errors");
        gw_ha_reachable = jsonBool(buf, "ha_reachable");
        gw_health_received = true;
        Serial.printf("GW health: errors=%d reachable=%d\n",
                       gw_ha_errors, gw_ha_reachable);
    }
}

static void publishDiscovery() {
    const char *co2_config =
        "{\"name\":\"CO2\","
        "\"device_class\":\"carbon_dioxide\","
        "\"state_topic\":\"" DEVICE_ID "/sensor/co2\","
        "\"unit_of_measurement\":\"ppm\","
        "\"unique_id\":\"torii_ink_co2\","
        "\"device\":{\"identifiers\":[\"torii_ink\"],\"name\":\"Torii Ink\",\"model\":\"ESP32-C6 e-ink\",\"manufacturer\":\"Hiki\"}}";
    mqtt.publish("homeassistant/sensor/torii_ink_co2/config", co2_config, true);

    const char *temp_config =
        "{\"name\":\"Temperature\","
        "\"device_class\":\"temperature\","
        "\"state_topic\":\"" DEVICE_ID "/sensor/temperature\","
        "\"unit_of_measurement\":\"\u00b0C\","
        "\"unique_id\":\"torii_ink_temperature\","
        "\"device\":{\"identifiers\":[\"torii_ink\"],\"name\":\"Torii Ink\",\"model\":\"ESP32-C6 e-ink\",\"manufacturer\":\"Hiki\"}}";
    mqtt.publish("homeassistant/sensor/torii_ink_temperature/config", temp_config, true);

    const char *hum_config =
        "{\"name\":\"Humidity\","
        "\"device_class\":\"humidity\","
        "\"state_topic\":\"" DEVICE_ID "/sensor/humidity\","
        "\"unit_of_measurement\":\"%\","
        "\"unique_id\":\"torii_ink_humidity\","
        "\"device\":{\"identifiers\":[\"torii_ink\"],\"name\":\"Torii Ink\",\"model\":\"ESP32-C6 e-ink\",\"manufacturer\":\"Hiki\"}}";
    mqtt.publish("homeassistant/sensor/torii_ink_humidity/config", hum_config, true);

    Serial.println("MQTT: HA discovery configs published");
}

static void connectMQTT() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (mqtt.connected()) return;

    Serial.print("MQTT: connecting... ");
    if (mqtt.connect(DEVICE_ID)) {
        Serial.println("connected");
        mqtt_connected = true;
        mqtt.subscribe(TOPIC_DISPLAY);
        mqtt.subscribe(TOPIC_HEALTH);
        mqtt.subscribe(TOPIC_KILLSWITCH);
        mqtt.subscribe(TOPIC_GW_HEALTH);
        for (int i = 0; i < 5; i++) { delay(100); mqtt.loop(); }
        publishDiscovery();
    } else {
        Serial.printf("failed (rc=%d)\n", mqtt.state());
        mqtt_connected = false;
    }
}

static void publishSensors() {
    if (!mqtt.connected()) return;

    char val[16];
    if (scd4x_ok) {
        snprintf(val, sizeof(val), "%.0f", co2);
        mqtt.publish(TOPIC_CO2, val);
        snprintf(val, sizeof(val), "%.1f", scd_temp);
        mqtt.publish(TOPIC_TEMP, val);
        snprintf(val, sizeof(val), "%.0f", scd_hum);
        mqtt.publish(TOPIC_HUM, val);
    }
    Serial.println("MQTT: sensors published");
}

// ─── Hardware init ─────────────────────────────────────────────

static void initDisplay() {
    DEV_Module_Init();
    EPD_4IN2_V2_Init();
    EPD_4IN2_V2_Clear();

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
    if (scd4x.begin(Wire, false, false, false)) {
        scd4x_ok = true;
        Serial.println("SCD4x: detected, starting periodic measurement...");
        scd4x.startPeriodicMeasurement();
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
    if (!scd4x_ok) return false;
    if (!scd4x.getDataReadyStatus()) return false;
    if (scd4x.readMeasurement()) {
        co2 = scd4x.getCO2();
        scd_temp = scd4x.getTemperature();
        scd_hum = scd4x.getHumidity();
        Serial.printf("SCD4x: CO2=%.0f ppm, T=%.1f C, H=%.0f%%\n", co2, scd_temp, scd_hum);
        return true;
    }
    return false;
}

static void readSensors() {
    readSCD4x();
}

// ─── Drawing helpers ───────────────────────────────────────────

static void renderCompactHeader(const char *title) {
    Paint_DrawString_EN(20, 4, title, &Font20, WHITE, BLACK);
    Paint_DrawLine(20, 28, 380, 28, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

static void renderFooter() {
    char buf[64];
    Paint_DrawLine(20, 259, 380, 259, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

    const char *web3 = ks_ws_connected ? "ok" : "--";
    const char *smarthome;
    char sm_buf[16];
    if (gw_health_received && gw_ha_reachable) {
        if (gw_ha_errors == 0) smarthome = "no error";
        else {
            snprintf(sm_buf, sizeof(sm_buf), "%d error%s",
                     gw_ha_errors, gw_ha_errors > 1 ? "s" : "");
            smarthome = sm_buf;
        }
    } else { smarthome = "--"; }
    snprintf(buf, sizeof(buf), "Web3:%s  Smart home:%s", web3, smarthome);
    Paint_DrawString_EN(20, 263, buf, &Font16, WHITE, BLACK);

    bool isolated = (strcmp(ks_state, "isolated") == 0);
    if (isolated) snprintf(buf, sizeof(buf), "AI: ISOLATED");
    else if (health_received) snprintf(buf, sizeof(buf), "AI:online (%d msg/24h)", h_msgs_24h);
    else snprintf(buf, sizeof(buf), "AI:--");
    Paint_DrawString_EN(20, 281, buf, &Font16, WHITE, BLACK);
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

static const char *getCO2Label() {
    if (co2 < 600) return "Excellent";
    if (co2 < 1000) return "Good";
    if (co2 < 1500) return "Stuffy";
    return "Ventilate!";
}

static bool hasAnyProblem() {
    bool isolated = (strcmp(ks_state, "isolated") == 0);
    bool node_down = health_received && (!h_ha || !h_gw || !h_inet);
    bool co2_high = scd4x_ok && co2 > 1000;
    return isolated || node_down || co2_high;
}

static const char *getPersonalityMessage() {
    bool isolated = (strcmp(ks_state, "isolated") == 0);
    if (isolated) return "...cut off from outside.";
    if (health_received && (!h_ha || !h_gw || !h_inet)) return "Something feels off...";
    if (scd4x_ok && co2 > 1500) return "Open a window, please?";
    if (scd4x_ok && co2 > 1000) return "Air's getting stuffy...";
    if (health_received && h_msgs_24h == 0) return "It's quiet today.";
    if (health_received && h_msgs_24h > 10) return "Busy day!";
    if (h_up[0] == '0') return "Just woke up...";
    return "All systems nominal.";
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

// Chat bubble: rectangle + tail + dots, ~14x14 at (x,y)
static void drawIconChat(int x, int y) {
    Paint_DrawRectangle(x, y, x + 13, y + 9, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawLine(x + 2, y + 9, x + 1, y + 13, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(x + 1, y + 13, x + 6, y + 9, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawCircle(x + 4, y + 5, 1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawCircle(x + 7, y + 5, 1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawCircle(x + 10, y + 5, 1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
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

// Section header: "LABEL ..........." labeled divider
static void drawSectionHeader(int y, const char *label) {
    Paint_DrawString_EN(28, y + 2, label, &Font16, WHITE, BLACK);
    int lw = strlen(label) * 11 + 34;
    Paint_DrawLine(lw, y + 9, 380, y + 9, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
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

// QR code helper: draw ks_address at position with px_size per module
static void drawQR(int qr_x, int qr_y, int px_sz) {
    if (!ks_address[0]) return;
    QRCode qrcode;
    uint8_t qrcodeData[qrcode_getBufferSize(6)];
    qrcode_initText(&qrcode, qrcodeData, 6, ECC_LOW, ks_address);
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

// ─── Screen 0: Mascot (home screen) ───────────────────────────

static void renderMascotPage() {
    char buf[48];

    // Mascot bitmap — left side
    const unsigned char *mascot = hasAnyProblem() ? hiki_worried : hiki_normal;
    Paint_DrawImage(mascot, 8, 6, MASCOT_W, MASCOT_H);

    // Speech bubble — personality message (right of mascot, top)
    const char *msg = getPersonalityMessage();
    int msg_len = strlen(msg);
    int bw = msg_len * 11 + 12;
    if (bw > 270) bw = 270;
    if (bw < 100) bw = 100;
    drawSpeechBubble(120, 4, bw, 24);
    Paint_DrawString_EN(126, 8, msg, &Font16, WHITE, BLACK);

    // ROOM panel — right of mascot, framed
    int px = 118, py = 38, pw = 274, ph = 98;
    Paint_DrawRectangle(px, py, px + pw, py + ph, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    // Embedded label on top border
    Paint_DrawRectangle(px + 8, py - 2, px + 58, py + 2, WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    Paint_DrawString_EN(px + 10, py - 8, "ROOM", &Font16, WHITE, BLACK);

    int ix = px + 10;  // inner x
    if (scd4x_ok) {
        // CO2 hero number
        snprintf(buf, sizeof(buf), "%.0f", co2);
        Paint_DrawString_EN(ix, py + 8, buf, &Font24, WHITE, BLACK);
        int nd = strlen(buf);
        Paint_DrawString_EN(ix + nd * 17 + 4, py + 14, "ppm", &Font16, WHITE, BLACK);

        // Quality label — right-aligned
        const char *label = getCO2Label();
        int lx = px + pw - (int)strlen(label) * 11 - 10;
        Paint_DrawString_EN(lx, py + 14, label, &Font16, WHITE, BLACK);

        // CO2 bar
        int co2v = (int)co2; if (co2v > 2000) co2v = 2000;
        drawProgressBar(ix, py + 36, pw - 20, 8, co2v, 2000);

        // Dotted sep inside panel
        Paint_DrawLine(ix, py + 50, px + pw - 10, py + 50, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

        // Temp with thermometer icon
        drawIconThermo(ix, py + 56);
        snprintf(buf, sizeof(buf), "%.1fC", scd_temp);
        Paint_DrawString_EN(ix + 14, py + 60, buf, &Font20, WHITE, BLACK);

        // Humidity with droplet icon
        int hx = ix + 110;
        drawIconDrop(hx, py + 56);
        snprintf(buf, sizeof(buf), "%.0f%%", scd_hum);
        Paint_DrawString_EN(hx + 14, py + 60, buf, &Font20, WHITE, BLACK);

        // WiFi RSSI
        int wx = ix + 210;
        int rssi = WiFi.RSSI();
        drawSignalBars(wx, py + 58, rssi);
        snprintf(buf, sizeof(buf), "%ddB", rssi);
        Paint_DrawString_EN(wx + 24, py + 60, buf, &Font16, WHITE, BLACK);
    } else {
        Paint_DrawString_EN(ix, py + 30, "Sensors offline", &Font20, WHITE, BLACK);
    }

    // Status icons row — full width below mascot
    Paint_DrawLine(8, 144, 392, 144, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    int iy = 152;

    // Clock + uptime
    drawIconClock(12, iy);
    Paint_DrawString_EN(28, iy + 2, health_received && h_up[0] ? h_up : "--",
                        &Font16, WHITE, BLACK);

    // Chat + messages
    drawIconChat(120, iy);
    if (health_received) snprintf(buf, sizeof(buf), "%d msg", h_msgs_24h);
    else snprintf(buf, sizeof(buf), "--");
    Paint_DrawString_EN(138, iy + 2, buf, &Font16, WHITE, BLACK);

    // HA API
    snprintf(buf, sizeof(buf), "HA:%s",
             health_received ? (h_ha_api ? "ok" : "!") : "--");
    Paint_DrawString_EN(230, iy + 2, buf, &Font16, WHITE, BLACK);

    // Model badge
    Paint_DrawString_EN(310, iy + 2, h_model[0] ? h_model : "--", &Font16, WHITE, BLACK);

    // Node tiles row
    Paint_DrawLine(8, 174, 392, 174, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    int ny = 182;

    drawNodeCircle(20,  ny, health_received && h_gw,  "GW");
    drawNodeCircle(80,  ny, health_received,          "AI");
    drawNodeCircle(140, ny, health_received && h_ha,  "HA");
    drawNodeCircle(196, ny, health_received && h_inet, "NET");
    drawNodeCircle(260, ny, ks_ws_connected,          "Web3");

    // Smart home + killswitch status
    int by = 200;
    const char *smarthome;
    char sm_buf[24];
    if (gw_health_received && gw_ha_reachable) {
        if (gw_ha_errors == 0) smarthome = "ok";
        else { snprintf(sm_buf, sizeof(sm_buf), "%d err", gw_ha_errors); smarthome = sm_buf; }
    } else { smarthome = "--"; }
    snprintf(buf, sizeof(buf), "Home:%s", smarthome);
    Paint_DrawString_EN(20, by, buf, &Font16, WHITE, BLACK);

    bool isolated = (strcmp(ks_state, "isolated") == 0);
    if (isolated) {
        Paint_DrawRectangle(200, by - 1, 370, by + 16, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawString_EN(204, by, "KS: ISOLATED", &Font16, BLACK, WHITE);
    } else if (ks_received) {
        Paint_DrawString_EN(230, by, "KS: connected", &Font16, WHITE, BLACK);
    }
}

// ─── Screen 1: Server Room (dashboard) ────────────────────────

static void renderDashboardPage() {
    renderCompactHeader("SERVER ROOM");

    char buf[48];
    int rssi = WiFi.RSSI();

    // WiFi bars in header (right side)
    drawSignalBars(340, 6, rssi);
    snprintf(buf, sizeof(buf), "%ddB", rssi);
    Paint_DrawString_EN(364, 10, buf, &Font16, WHITE, BLACK);

    // ── ENVIRONMENT section ──
    drawSectionHeader(32, "ENVIRONMENT");

    if (scd4x_ok) {
        // CO2 hero
        snprintf(buf, sizeof(buf), "%.0f", co2);
        Paint_DrawString_EN(28, 48, buf, &Font24, WHITE, BLACK);
        Paint_DrawString_EN(28 + (int)strlen(buf) * 17 + 2, 54, "ppm", &Font16, WHITE, BLACK);

        // Quality label in bordered box
        const char *label = getCO2Label();
        int ll = strlen(label);
        int lbx = 380 - ll * 11 - 8;
        Paint_DrawRectangle(lbx, 48, 382, 68, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        Paint_DrawString_EN(lbx + 4, 50, label, &Font16, WHITE, BLACK);

        // CO2 bar
        int co2v = (int)co2; if (co2v > 2000) co2v = 2000;
        drawProgressBar(28, 74, 352, 10, co2v, 2000);

        // Temp + humidity with icons
        drawIconThermo(28, 90);
        snprintf(buf, sizeof(buf), "%.1f C", scd_temp);
        Paint_DrawString_EN(42, 92, buf, &Font16, WHITE, BLACK);

        drawIconDrop(160, 90);
        snprintf(buf, sizeof(buf), "%.0f%%", scd_hum);
        Paint_DrawString_EN(176, 92, buf, &Font16, WHITE, BLACK);

        snprintf(buf, sizeof(buf), "MQTT:%s", mqtt_connected ? "ok" : "--");
        Paint_DrawString_EN(280, 92, buf, &Font16, WHITE, BLACK);
    } else {
        Paint_DrawString_EN(28, 48, "Sensors: --", &Font20, WHITE, BLACK);
    }

    // ── NETWORK section ──
    drawSectionHeader(112, "NETWORK");

    drawNodeCircle(36,  130, health_received && h_ha,   "HA");
    if (health_received) { snprintf(buf, sizeof(buf), "%dms", h_ha_ms); Paint_DrawString_EN(66, 124, buf, &Font16, WHITE, BLACK); }

    drawNodeCircle(150, 130, health_received && h_gw,   "GW");
    if (health_received) { snprintf(buf, sizeof(buf), "%dms", h_gw_ms); Paint_DrawString_EN(182, 124, buf, &Font16, WHITE, BLACK); }

    drawNodeCircle(280, 130, health_received && h_inet,  "NET");
    if (health_received) { snprintf(buf, sizeof(buf), "%dms", h_inet_ms); Paint_DrawString_EN(322, 124, buf, &Font16, WHITE, BLACK); }

    snprintf(buf, sizeof(buf), "HA API:%s",
             health_received ? (h_ha_api ? "ok" : "FAIL") : "--");
    Paint_DrawString_EN(28, 146, buf, &Font16, WHITE, BLACK);

    // ── SYSTEM section ──
    drawSectionHeader(164, "SYSTEM");

    // Uptime + memory + disk with icons
    drawIconClock(28, 180);
    Paint_DrawString_EN(46, 182, health_received && h_up[0] ? h_up : "--", &Font16, WHITE, BLACK);

    if (health_received) {
        snprintf(buf, sizeof(buf), "Mem:%dM", h_mem);
        Paint_DrawString_EN(140, 182, buf, &Font16, WHITE, BLACK);

        snprintf(buf, sizeof(buf), "Disk:%d%%", h_disk);
        Paint_DrawString_EN(250, 182, buf, &Font16, WHITE, BLACK);
        drawProgressBar(250, 200, 120, 8, h_disk, 100);
    }

    // AI + Web3 status
    bool ai_isolated = (strcmp(ks_state, "isolated") == 0);
    if (ai_isolated) {
        Paint_DrawRectangle(28, 216, 150, 234, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawString_EN(32, 218, "AI: ISOLATED", &Font16, BLACK, WHITE);
    } else if (health_received) {
        snprintf(buf, sizeof(buf), "AI:online  %d msg/24h", h_msgs_24h);
        Paint_DrawString_EN(28, 218, buf, &Font16, WHITE, BLACK);
    } else {
        Paint_DrawString_EN(28, 218, "AI: --", &Font16, WHITE, BLACK);
    }
    Paint_DrawString_EN(280, 218, ks_ws_connected ? "Web3:ok" : "Web3:--", &Font16, WHITE, BLACK);

    // Smart home status
    if (gw_health_received && gw_ha_reachable) {
        if (gw_ha_errors == 0) Paint_DrawString_EN(28, 238, "Smart home: no errors", &Font16, WHITE, BLACK);
        else { snprintf(buf, sizeof(buf), "Smart home: %d err", gw_ha_errors); Paint_DrawString_EN(28, 238, buf, &Font16, WHITE, BLACK); }
    } else {
        Paint_DrawString_EN(28, 238, "Smart home: --", &Font16, WHITE, BLACK);
    }

    // Bottom border (no standard footer — data is integrated)
    Paint_DrawLine(20, 255, 380, 255, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
}

// ─── Screen 2: Hallway (network topology) ─────────────────────

// Node box: double border when online, dotted when offline
static void drawNodeBox(int x, int y, int w, int h, bool online) {
    Paint_DrawRectangle(x, y, x + w, y + h, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    if (online)
        Paint_DrawRectangle(x + 2, y + 2, x + w - 2, y + h - 2, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
}

// Connection line: thick when healthy, thin dotted when down
static void drawLink(int x1, int y1, int x2, int y2, bool healthy) {
    Paint_DrawLine(x1, y1, x2, y2, BLACK,
                   healthy ? DOT_PIXEL_2X2 : DOT_PIXEL_1X1,
                   healthy ? LINE_STYLE_SOLID : LINE_STYLE_DOTTED);
}

static void renderNetworkPage() {
    Paint_DrawString_EN(20, 4, "HALLWAY", &Font20, WHITE, BLACK);
    // WiFi bars in header
    int rssi = WiFi.RSSI();
    drawSignalBars(330, 4, rssi);
    char rssi_buf[16];
    snprintf(rssi_buf, sizeof(rssi_buf), "%ddB", rssi);
    Paint_DrawString_EN(354, 8, rssi_buf, &Font16, WHITE, BLACK);
    Paint_DrawLine(20, 26, 380, 26, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    char buf[48];
    bool inet_ok = health_received && h_inet;
    bool gw_ok = health_received && h_gw;
    bool ha_ok = health_received && h_ha;

    // Internet label + status
    Paint_DrawString_EN(160, 32, "INTERNET", &Font16, WHITE, BLACK);
    if (health_received) {
        snprintf(buf, sizeof(buf), "%dms", h_inet_ms);
        Paint_DrawString_EN(260, 32, buf, &Font16, WHITE, BLACK);
    }

    // Connection: Internet → Gateway
    int gw_cx = 200;
    drawLink(gw_cx, 50, gw_cx, 64, inet_ok);

    // Gateway box
    int gw_x = 120, gw_y = 64, gw_w = 160, gw_h = 40;
    drawNodeBox(gw_x, gw_y, gw_w, gw_h, gw_ok);
    Paint_DrawString_EN(gw_x + 8, gw_y + 4, "GATEWAY", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(gw_x + 8, gw_y + 20, "10.0.0.1", &Font16, WHITE, BLACK);
    // Details beside box
    Paint_DrawString_EN(gw_x + gw_w + 6, gw_y + 4, "OPi RV2", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(gw_x + gw_w + 6, gw_y + 20, ks_ws_connected ? "Web3:ok" : "Web3:--", &Font16, WHITE, BLACK);

    // Branch: GW → Agent and GW → HA
    int gw_bot = gw_y + gw_h;
    int agent_cx = 85, ha_cx = 310;
    int branch_y = gw_bot + 6;

    drawLink(gw_cx, gw_bot, gw_cx, branch_y, gw_ok);
    Paint_DrawLine(agent_cx, branch_y, ha_cx, branch_y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

    int child_y = branch_y + 16;
    drawLink(agent_cx, branch_y, agent_cx, child_y, gw_ok);
    drawLink(ha_cx, branch_y, ha_cx, child_y, ha_ok);

    // Latency on branches
    if (health_received) {
        snprintf(buf, sizeof(buf), "%dms", h_gw_ms);
        Paint_DrawString_EN(agent_cx - 30, branch_y + 2, buf, &Font16, WHITE, BLACK);
        snprintf(buf, sizeof(buf), "%dms", h_ha_ms);
        Paint_DrawString_EN(ha_cx + 6, branch_y + 2, buf, &Font16, WHITE, BLACK);
    }

    // Agent box
    int ag_w = 130, ag_h = 50;
    drawNodeBox(agent_cx - ag_w / 2, child_y, ag_w, ag_h, health_received);
    Paint_DrawString_EN(agent_cx - ag_w / 2 + 6, child_y + 4, "AI AGENT", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(agent_cx - ag_w / 2 + 6, child_y + 20, "10.0.0.2", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(agent_cx - ag_w / 2 + 6, child_y + 36, "openClaw", &Font16, WHITE, BLACK);

    // HA box
    drawNodeBox(ha_cx - ag_w / 2, child_y, ag_w, ag_h, ha_ok);
    Paint_DrawString_EN(ha_cx - ag_w / 2 + 6, child_y + 4, "SMART HOME", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(ha_cx - ag_w / 2 + 6, child_y + 20, "10.0.0.3", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(ha_cx - ag_w / 2 + 6, child_y + 36, "HA+MQTT+Z2M", &Font16, WHITE, BLACK);

    // Summary bar
    int sum_y = child_y + ag_h + 6;
    Paint_DrawLine(20, sum_y, 380, sum_y, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    snprintf(buf, sizeof(buf), "HA API:%s",
             health_received ? (h_ha_api ? "ok" : "fail") : "--");
    Paint_DrawString_EN(24, sum_y + 4, buf, &Font16, WHITE, BLACK);

    renderFooter();
}

// ─── Screen 3: Front Door (killswitch) ────────────────────────

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
        Paint_DrawString_EN(nodes[i] - (int)strlen(labels[i]) * 11 / 2, y - 20,
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

static void renderKillswitchPage() {
    if (!ks_received) {
        renderCompactHeader("FRONT DOOR");
        Paint_DrawString_EN(100, 100, "Waiting for", &Font20, WHITE, BLACK);
        Paint_DrawString_EN(80, 130, "killswitch data...", &Font20, WHITE, BLACK);
        renderFooter();
        return;
    }

    bool connected = (strcmp(ks_state, "connected") == 0);

    if (connected) {
        // ── CONNECTED: calm, reassuring ──
        Paint_DrawString_EN(20, 4, "FRONT DOOR", &Font20, WHITE, BLACK);
        drawShield(370, 16, 12, true);
        Paint_DrawLine(20, 28, 380, 28, BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);

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

        // Topology
        Paint_DrawLine(20, 180, 380, 180, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
        drawTopology(206, false);

        // WS status
        char ws_buf[32];
        snprintf(ws_buf, sizeof(ws_buf), "WS: %s", ks_ws_connected ? "connected" : "disconnected");
        Paint_DrawString_EN(20, 228, ws_buf, &Font16, WHITE, BLACK);

        renderFooter();

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
        Paint_DrawRectangle(200, 86, 380, 106, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawString_EN(204, 88, "Traffic: DROPPED", &Font16, BLACK, WHITE);

        // Details box
        char bbuf[48];
        Paint_DrawRectangle(20, 120, 380, 168, BLACK, DOT_PIXEL_2X2, DRAW_FILL_EMPTY);
        if (ks_block_number > 0) {
            snprintf(bbuf, sizeof(bbuf), "Block: #%d", ks_block_number);
            Paint_DrawString_EN(30, 126, bbuf, &Font20, WHITE, BLACK);
        }
        if (ks_isolated_at[0]) {
            snprintf(bbuf, sizeof(bbuf), "Isolated at: %s", ks_isolated_at);
            Paint_DrawString_EN(30, 148, bbuf, &Font16, WHITE, BLACK);
        }

        // Broken topology
        drawTopology(196, true);

        // Restore instructions
        Paint_DrawString_EN(20, 220, "To restore, send:", &Font16, WHITE, BLACK);
        Paint_DrawRectangle(20, 238, 250, 256, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawString_EN(24, 240, "Launch(param=true)", &Font16, BLACK, WHITE);

        // Inverted bottom bar
        Paint_DrawRectangle(0, 270, 399, 299, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawString_EN(56, 276, "! KILLSWITCH ACTIVE !", &Font24, BLACK, WHITE);
    }
}

// ─── Screen rendering ──────────────────────────────────────────

static void renderScreen(bool full_refresh) {
    if (!framebuffer) return;

    Paint_SelectImage(framebuffer);
    Paint_Clear(WHITE);

    switch (current_screen) {
        case 0: renderMascotPage();       break;
        case 1: renderDashboardPage();    break;
        case 2: renderNetworkPage();      break;
        case 3: renderKillswitchPage();   break;
    }

    if (full_refresh) {
        EPD_4IN2_V2_Display(framebuffer);
    } else {
        EPD_4IN2_V2_PartialDisplay(framebuffer);
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

    Serial.println("Waiting for SCD4x first reading...");
    bool got_reading = false;
    for (int i = 0; i < 15; i++) {
        delay(1000);
        mqtt.loop();
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
    current_screen = 0;
    renderScreen(true);
    Serial.println("Display updated (full).");
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi: reconnecting...");
        WiFi.reconnect();
        delay(5000);
    }
    connectMQTT();

    bool button_pressed = false;
    bool btn_prev = HIGH;
    unsigned long debounce_time = 0;

    for (unsigned long elapsed = 0; elapsed < CYCLE_MS; elapsed += 100) {
        mqtt.loop();
        delay(100);

        bool btn_now = digitalRead(BTN_PIN);
        if (btn_prev == HIGH && btn_now == LOW && (millis() - debounce_time) > DEBOUNCE_MS) {
            debounce_time = millis();
            button_pressed = true;
            break;
        }
        btn_prev = btn_now;
    }
    cycle_count++;

    if (cycle_count % SENSOR_EVERY_N == 0) {
        readSensors();
        publishSensors();
    }

    current_screen = (current_screen + 1) % SCREEN_COUNT;

    bool full = (cycle_count % FULL_REFRESH_EVERY_N == 0);
    renderScreen(full);
    Serial.printf("Screen %d (%s%s).\n", current_screen,
                  full ? "full" : "partial",
                  button_pressed ? ", btn" : "");
}
