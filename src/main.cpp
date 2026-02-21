#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SparkFun_SCD4x_Arduino_Library.h>

#include <qrcode.h>

#include "EPD_4in2.h"
#include "GUI_Paint.h"
#include "config.h"

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

// Screen cycling: 0=agent, 1=net, 2=health, 3=env, 4=killswitch
static int current_screen = 0;
static int cycle_count = 0;
#define SCREEN_COUNT      5
#define CYCLE_MS          10000       // switch screens every 10s
#define SENSOR_EVERY_N    6           // read+publish sensors every 6 cycles (60s)
#define FULL_REFRESH_EVERY_N 30       // full e-ink refresh every 30 cycles (~5min)

// Extract int value for a key from JSON string (e.g. "ha":1 → 1)
static int jsonInt(const char *json, const char *key) {
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return 0;
    p += strlen(search);
    return atoi(p);
}

// Extract string value for a key from JSON (e.g. "up":"2d5h" → "2d5h")
static void jsonStr(const char *json, const char *key, char *out, size_t out_sz) {
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":\"", key);
    const char *p = strstr(json, search);
    if (!p) { out[0] = '\0'; return; }
    p += strlen(search);
    const char *end = strchr(p, '"');
    if (!end) { out[0] = '\0'; return; }
    size_t len = end - p;
    if (len >= out_sz) len = out_sz - 1;
    memcpy(out, p, len);
    out[len] = '\0';
}

// Extract bool value for a key from JSON (e.g. "ws_connected":true → true)
static bool jsonBool(const char *json, const char *key) {
    char search[32];
    snprintf(search, sizeof(search), "\"%s\":", key);
    const char *p = strstr(json, search);
    if (!p) return false;
    p += strlen(search);
    while (*p == ' ') p++;
    return strncmp(p, "true", 4) == 0;
}

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
    // CO2
    const char *co2_config =
        "{\"name\":\"CO2\","
        "\"device_class\":\"carbon_dioxide\","
        "\"state_topic\":\"" DEVICE_ID "/sensor/co2\","
        "\"unit_of_measurement\":\"ppm\","
        "\"unique_id\":\"torii_ink_co2\","
        "\"device\":{\"identifiers\":[\"torii_ink\"],\"name\":\"Torii Ink\",\"model\":\"ESP32-C6 e-ink\",\"manufacturer\":\"Hiki\"}}";
    mqtt.publish("homeassistant/sensor/torii_ink_co2/config", co2_config, true);

    // Temperature
    const char *temp_config =
        "{\"name\":\"Temperature\","
        "\"device_class\":\"temperature\","
        "\"state_topic\":\"" DEVICE_ID "/sensor/temperature\","
        "\"unit_of_measurement\":\"\u00b0C\","
        "\"unique_id\":\"torii_ink_temperature\","
        "\"device\":{\"identifiers\":[\"torii_ink\"],\"name\":\"Torii Ink\",\"model\":\"ESP32-C6 e-ink\",\"manufacturer\":\"Hiki\"}}";
    mqtt.publish("homeassistant/sensor/torii_ink_temperature/config", temp_config, true);

    // Humidity
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
        // Process retained messages (broker needs a few loop ticks to deliver)
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

static void renderHeader(const char *subtitle) {
    Paint_DrawString_EN(20, 20, "Hello, Ens!", &Font24, WHITE, BLACK);
    Paint_DrawLine(20, 55, 380, 55, BLACK, DOT_PIXEL_2X2, LINE_STYLE_SOLID);
    Paint_DrawString_EN(20, 65, subtitle, &Font20, WHITE, BLACK);
}

static void renderFooter() {
    char buf[64];
    Paint_DrawLine(20, 259, 380, 259, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

    // Line 1: Web3 + Smart home
    const char *web3 = ks_ws_connected ? "ok" : "--";
    const char *smarthome;
    char sm_buf[16];
    if (gw_health_received && gw_ha_reachable) {
        if (gw_ha_errors == 0)
            smarthome = "no error";
        else {
            snprintf(sm_buf, sizeof(sm_buf), "%d error%s",
                     gw_ha_errors, gw_ha_errors > 1 ? "s" : "");
            smarthome = sm_buf;
        }
    } else {
        smarthome = "--";
    }
    snprintf(buf, sizeof(buf), "Web3:%s  Smart home:%s", web3, smarthome);
    Paint_DrawString_EN(20, 263, buf, &Font16, WHITE, BLACK);

    // Line 2: AI status
    bool isolated = (strcmp(ks_state, "isolated") == 0);
    if (isolated) {
        snprintf(buf, sizeof(buf), "AI: ISOLATED");
    } else if (health_received) {
        snprintf(buf, sizeof(buf), "AI:online (%d msg/24h)", h_msgs_24h);
    } else {
        snprintf(buf, sizeof(buf), "AI:--");
    }
    Paint_DrawString_EN(20, 281, buf, &Font16, WHITE, BLACK);
}

// Screen 0: Agent identity card
static void renderAgentPage() {
    char buf[48];
    Paint_DrawString_EN(20, 100, "Name:    Hiki", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(20, 122, "Role:    Smart Home AI", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(20, 144, "Model:   Sonnet 4.5", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(20, 166, "Node:    RPi 4B", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(20, 188, "Channel: Telegram", &Font16, WHITE, BLACK);
    if (health_received && h_up[0]) {
        snprintf(buf, sizeof(buf), "Uptime:  %s", h_up);
        Paint_DrawString_EN(20, 210, buf, &Font16, WHITE, BLACK);
    }
}

// Screen 1: Network architecture
static void renderNetPage() {
    // Data flow diagram
    Paint_DrawString_EN(20, 98, "[GW]-->[Agent]-->[HA]", &Font20, WHITE, BLACK);
    if (health_received) {
        char buf[48];
        snprintf(buf, sizeof(buf), " %s       %s        %s",
                 h_gw ? "ok" : "--", h_up[0] ? "ok" : "--", h_ha ? "ok" : "--");
        Paint_DrawString_EN(20, 124, buf, &Font16, WHITE, BLACK);
    }
    Paint_DrawLine(20, 148, 380, 148, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

    // Node addresses
    Paint_DrawString_EN(20, 158, "GW    10.0.0.1 OPi RV2", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(20, 178, "Agent 10.0.0.2 RPi 4B", &Font16, WHITE, BLACK);
    Paint_DrawString_EN(20, 198, "HA    10.0.0.3 RPi 4B", &Font16, WHITE, BLACK);

    if (health_received) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Internet: %s (%dms)",
                 h_inet ? "ok" : "--", h_inet_ms);
        Paint_DrawString_EN(20, 228, buf, &Font16, WHITE, BLACK);
    }
}

// Screen 2: Node health status
static void renderHealthPage() {
    char buf[48];

    snprintf(buf, sizeof(buf), "HA:%s  GW:%s  NET:%s",
             h_ha ? "ok" : "--", h_gw ? "ok" : "--", h_inet ? "ok" : "--");
    Paint_DrawString_EN(20, 100, buf, &Font20, WHITE, BLACK);

    snprintf(buf, sizeof(buf), "%dms    %dms    %dms",
             h_ha_ms, h_gw_ms, h_inet_ms);
    Paint_DrawString_EN(20, 130, buf, &Font16, WHITE, BLACK);

    snprintf(buf, sizeof(buf), "HA API: %s", h_ha_api ? "ok" : "--");
    Paint_DrawString_EN(20, 155, buf, &Font16, WHITE, BLACK);

    Paint_DrawLine(20, 182, 380, 182, BLACK, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);

    snprintf(buf, sizeof(buf), "Up: %s", h_up[0] ? h_up : "--");
    Paint_DrawString_EN(20, 192, buf, &Font16, WHITE, BLACK);

    snprintf(buf, sizeof(buf), "Mem: %dM free", h_mem);
    Paint_DrawString_EN(20, 214, buf, &Font16, WHITE, BLACK);

    snprintf(buf, sizeof(buf), "Disk: %d%% used", h_disk);
    Paint_DrawString_EN(20, 236, buf, &Font16, WHITE, BLACK);
}

// Screen 3: Environment sensors
static void renderEnvPage() {
    char buf[48];
    if (scd4x_ok) {
        snprintf(buf, sizeof(buf), "CO2:  %.0f ppm", co2);
        Paint_DrawString_EN(20, 100, buf, &Font20, WHITE, BLACK);

        snprintf(buf, sizeof(buf), "Temp: %.1f C", scd_temp);
        Paint_DrawString_EN(20, 135, buf, &Font16, WHITE, BLACK);

        snprintf(buf, sizeof(buf), "Hum:  %.0f %%", scd_hum);
        Paint_DrawString_EN(20, 160, buf, &Font16, WHITE, BLACK);
    } else {
        Paint_DrawString_EN(20, 100, "SCD4x: not found", &Font16, WHITE, BLACK);
    }
}

// Screen 4: Killswitch status + QR code
static void renderKillswitchPage() {
    if (!ks_received) {
        Paint_DrawString_EN(20, 100, "Killswitch: no data", &Font16, WHITE, BLACK);
        Paint_DrawString_EN(20, 125, "Waiting for MQTT...", &Font16, WHITE, BLACK);
        return;
    }

    // QR code on left side (SS58 address)
    if (ks_address[0]) {
        QRCode qrcode;
        uint8_t qrcodeData[qrcode_getBufferSize(6)];
        qrcode_initText(&qrcode, qrcodeData, 6, ECC_LOW, ks_address);

        int qr_size = qrcode.size;  // modules per side
        int px = 3;                 // pixels per module
        int qr_px = qr_size * px;
        int qr_x = 20;
        int qr_y = 95;

        // White border
        Paint_DrawRectangle(qr_x - 2, qr_y - 2,
                            qr_x + qr_px + 2, qr_y + qr_px + 2,
                            WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);

        for (int y = 0; y < qr_size; y++) {
            for (int x = 0; x < qr_size; x++) {
                if (qrcode_getModule(&qrcode, x, y)) {
                    Paint_DrawRectangle(
                        qr_x + x * px, qr_y + y * px,
                        qr_x + (x + 1) * px - 1, qr_y + (y + 1) * px - 1,
                        BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
                }
            }
        }
    }

    // Status text on right side
    int tx = 170;
    Paint_DrawString_EN(tx, 100, "STATUS:", &Font16, WHITE, BLACK);

    bool connected = (strcmp(ks_state, "connected") == 0);
    if (connected) {
        Paint_DrawRectangle(tx, 122, tx + 16, 138, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawString_EN(tx + 22, 122, "CONNECTED", &Font20, WHITE, BLACK);
        Paint_DrawString_EN(tx, 150, "Agent has access to", &Font16, WHITE, BLACK);
        Paint_DrawString_EN(tx, 168, "internet and HA", &Font16, WHITE, BLACK);
    } else {
        Paint_DrawRectangle(tx, 122, tx + 16, 138, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        Paint_DrawString_EN(tx + 22, 122, "ISOLATED", &Font20, WHITE, BLACK);
        if (ks_isolated_at[0]) {
            char tbuf[32];
            snprintf(tbuf, sizeof(tbuf), "Isolated at %s", ks_isolated_at);
            Paint_DrawString_EN(tx, 150, tbuf, &Font16, WHITE, BLACK);
        }
        if (ks_block_number > 0) {
            char bbuf[32];
            snprintf(bbuf, sizeof(bbuf), "Block: %d", ks_block_number);
            Paint_DrawString_EN(tx, 168, bbuf, &Font16, WHITE, BLACK);
        }
    }
}

static const char *screen_titles[] = {
    "HIKI AGENT", "HIKI NET", "HIKI HEALTH", "HIKI ENV", "KILLSWITCH"
};

static void renderScreen(bool full_refresh) {
    if (!framebuffer) return;

    Paint_SelectImage(framebuffer);
    Paint_Clear(WHITE);

    renderHeader(screen_titles[current_screen]);
    switch (current_screen) {
        case 0: renderAgentPage();       break;
        case 1: renderNetPage();         break;
        case 2: renderHealthPage();      break;
        case 3: renderEnvPage();         break;
        case 4: renderKillswitchPage();  break;
    }
    renderFooter();

    if (full_refresh) {
        EPD_4IN2_V2_Display(framebuffer);
    } else {
        EPD_4IN2_V2_PartialDisplay(framebuffer);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=== TORII-INK ===");

    pinMode(BTN_PIN, INPUT_PULLUP);

    initDisplay();
    initSensors();
    initWiFi();

    // MQTT
    mqtt.setServer(MQTT_SERVER, MQTT_PORT);
    mqtt.setBufferSize(512);
    mqtt.setCallback(mqttCallback);
    connectMQTT();

    // Wait for SCD4x first measurement (up to 15s)
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
    // Reconnect if needed
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi: reconnecting...");
        WiFi.reconnect();
        delay(5000);
    }
    connectMQTT();

    // Wait CYCLE_MS, polling button + MQTT every 100ms
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
            break;  // exit wait loop immediately
        }
        btn_prev = btn_now;
    }
    cycle_count++;

    // Read sensors + publish periodically
    if (cycle_count % SENSOR_EVERY_N == 0) {
        readSensors();
        publishSensors();
    }

    // Advance screen (button press or auto-cycle)
    current_screen = (current_screen + 1) % SCREEN_COUNT;

    bool full = (cycle_count % FULL_REFRESH_EVERY_N == 0);
    renderScreen(full);
    Serial.printf("Screen %d (%s%s).\n", current_screen,
                  full ? "full" : "partial",
                  button_pressed ? ", btn" : "");
}
