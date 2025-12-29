/*
 * M5Dial Home Assistant Controller
 * 
 * Controls Home Assistant alarm via MQTT using the M5Dial's rotary encoder
 * and touch screen with a phone-like keypad interface.
 * 
 * Features:
 * - WiFi connection at startup
 * - MQTT connection to Home Assistant
 * - Rotary encoder for navigation and code input
 * - Touch screen keypad for code input
 * - RTC time display and configuration
 * - Status display showing WiFi and MQTT connection state
 * 
 * Based on M5Dial library and adapted from m5stickc-tricorder Home Assistant app
 */

#include "M5Dial.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>

// Credentials - copy credentials.h.example to credentials.h and fill in your values
#include "credentials.h"

// MQTT Topics for Home Assistant alarm
#define HA_MQTT_COMMAND_TOPIC "home/alarm/set"
#define HA_MQTT_STATE_TOPIC "home/alarm"
#define HA_MQTT_STATUS_TOPIC "home/alarm/status"

// Binary sensors for alarm (doors/windows)
#define HA_SENSOR_COUNT 11
const char* haSensorTopics[HA_SENSOR_COUNT] = {
    "home-assistant/porta_ingresso/contact",
    "home-assistant/portafinestra_cucina/contact",
    "home-assistant/portafinestra_corridoio/contact",
    "home-assistant/finestra_bagno_pt/contact",
    "home-assistant/finestra_ingresso/contact",
    "home-assistant/finestra_scale/contact",
    "home-assistant/portafinestra_camera/contact",
    "home-assistant/finestra_camera_bimbe/contact",
    "home-assistant/finestra_studio/contact",
    "home-assistant/finestra_camera/contact",
    "home-assistant/finestra_bagno_p1/contact"
};
const char* haSensorNames[HA_SENSOR_COUNT] = {
    "Porta ingresso",
    "PF cucina",
    "PF corridoio",
    "Fin bagno PT",
    "Fin ingresso",
    "Fin scale",
    "PF camera",
    "Fin cam bimbe",
    "Fin studio",
    "Fin camera",
    "Fin bagno P1"
};

// Display constants for round 240x240 display
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define CENTER_X 120
#define CENTER_Y 120

// Sprite for double-buffering (prevents flickering)
M5Canvas canvas(&M5Dial.Display);

// Menu options count for circular border (excludes MAIN/ALARM since that's the current screen)
#define MENU_COUNT 3
const char* menuNames[MENU_COUNT] = {"KEYPAD", "SENSORS", "SETTINGS"};
int menuSelection = 0;  // Currently highlighted menu item (0=KEYPAD, 1=SENSORS, 2=SETTINGS)

// Modern Color Palette - Beautiful gradients and vibrant colors
#define COLOR_BG        0x0821  // Dark blue-gray background
#define COLOR_TEXT      0xFFFF  // White
#define COLOR_ACCENT    0x05FF  // Bright cyan
#define COLOR_OK        0x07E0  // Bright green
#define COLOR_WARN      0xFEA0  // Bright yellow
#define COLOR_ERROR     0xF800  // Red
#define COLOR_DIM       0x5AEB  // Light gray
#define COLOR_ARMED     0xF800  // Red
#define COLOR_DISARMED  0x07E0  // Bright green
#define COLOR_ORANGE    0xFD20  // Vibrant orange for menu highlight
#define COLOR_PURPLE    0x781F  // Purple accent
#define COLOR_BLUE      0x041F  // Deep blue
#define COLOR_GLASS     0x2104  // Glassmorphic overlay
#define COLOR_DARK_GRAY 0x2104  // Dark gray for subtle elements

// App states
enum AppScreen {
    SCREEN_MAIN,      // Main status screen
    SCREEN_KEYPAD,    // Keypad for code entry
    SCREEN_SENSORS,   // Sensor status list
    SCREEN_SETTINGS   // RTC settings
};

// Menu screens mapping (must be after AppScreen enum)
const AppScreen menuScreens[MENU_COUNT] = {SCREEN_KEYPAD, SCREEN_SENSORS, SCREEN_SETTINGS};

// Settings fields
enum SettingsField { SET_HOUR, SET_MIN, SET_DAY, SET_MONTH, SET_YEAR, SET_FIELD_COUNT };

// Connection states
enum ConnState { CONN_DISCONNECTED, CONN_CONNECTING, CONN_CONNECTED, CONN_ERROR };

// Global state
AppScreen currentScreen = SCREEN_MAIN;
AppScreen previousScreen = SCREEN_MAIN;  // Track screen changes for redraw optimization
SettingsField settingsField = SET_HOUR;
bool settingsEditing = false;
bool needsRedraw = true;  // Force redraw when state changes
ConnState wifiState = CONN_DISCONNECTED;
ConnState mqttState = CONN_DISCONNECTED;
String haAlarmState = "unknown";
bool haSensorOpen[HA_SENSOR_COUNT] = {false};
String lastError = "";

// Code entry
#define CODE_MAX_LEN 8
char enteredCode[CODE_MAX_LEN + 1] = "";
int codeLength = 0;
int selectedKey = 5;  // Start at '5' (center of keypad)

// Encoder tracking
long lastEncoderValue = 0;
int encoderAccumulator = 0;

// Touch keypad layout: 3x3 number grid with CLR on left and OK on right as tall buttons
// Keys: 1,2,3 / 4,5,6 / 7,8,9 / 0 (center bottom)
// CLR = index 10, OK = index 11 (special tall buttons on sides)
const char keypadCharsDefault[12] = {'1','2','3','4','5','6','7','8','9','0','C','>'};
char keypadChars[12] = {'1','2','3','4','5','6','7','8','9','0','C','>'};
int keypadMapping[10] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};  // Maps position to digit index
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 3
#define KEY_WIDTH 50
#define KEY_HEIGHT 38
#define KEYPAD_START_X 45  // Centered: (240 - 3*50) / 2 = 45
#define KEYPAD_START_Y 70
#define SIDE_BTN_WIDTH 15  // Half of previous width
#define SIDE_BTN_HEIGHT (KEY_HEIGHT * 3 + 6)  // Height of 3 buttons

// MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
Preferences prefs;

// Timing
unsigned long lastReconnectAttempt = 0;
unsigned long wifiConnectStart = 0;
#define WIFI_TIMEOUT_MS 15000
#define MQTT_RECONNECT_INTERVAL 5000

// NTP sync state
bool ntpSynced = false;

// Animation state
unsigned long animationStartTime = 0;

// Alarm buzzer state
bool alarmBuzzerActive = false;
unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;

// Long press detection
#define LONG_PRESS_MS 1000
unsigned long btnPressStart = 0;
bool btnWasPressed = false;
bool btnLongTriggered = false;

// Forward declarations
void drawMainScreen();
void drawKeypadScreen();
void drawSensorsScreen();
void drawSettingsScreen();
void drawStatusBar();
void drawWifiIcon(int x, int y, bool connected);
void handleTouch();
void handleEncoder();
void connectWiFi();
void connectMqtt();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void setRtcFromCompileTime();
void syncRtcFromNtp();
void shuffleKeypad();
void playAlarmBuzzer();
void stopAlarmBuzzer();
void drawGradientCircle(int cx, int cy, int radius, uint16_t color1, uint16_t color2);
void drawGlassPanel(int x, int y, int w, int h, int radius);
void drawSmoothArc(int cx, int cy, int radius, float startAngle, float endAngle, int thickness, uint16_t color);
uint16_t interpolateColor(uint16_t color1, uint16_t color2, float t);

void setup() {
    Serial.begin(115200);
    Serial.println("M5Dial Home Assistant Controller");
    
    // Initialize M5Dial with encoder enabled
    auto cfg = M5.config();
    M5Dial.begin(cfg, true, false);  // encoder=true, RFID=false
    
    // Configure display
    M5Dial.Display.setRotation(0);
    M5Dial.Display.fillScreen(COLOR_BG);
    
    // Create sprite for double-buffering
    canvas.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);
    canvas.setTextColor(COLOR_TEXT);
    canvas.setTextDatum(middle_center);
    canvas.setTextSize(1);
    
    // Show startup message
    canvas.fillScreen(COLOR_BG);
    canvas.drawString("M5Dial HA", CENTER_X, CENTER_Y - 20);
    canvas.drawString("Connecting...", CENTER_X, CENTER_Y + 20);
    canvas.pushSprite(0, 0);
    
    // Initialize preferences for RTC sync
    prefs.begin("m5dial-ha", false);
    
    // Set RTC from compile time if not already set
    setRtcFromCompileTime();
    
    // Initialize encoder
    lastEncoderValue = M5Dial.Encoder.read();
    
    // Start WiFi connection
    connectWiFi();
    
    // Setup MQTT
    mqttClient.setServer(HA_MQTT_SERVER, HA_MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(512);
}

void loop() {
    M5Dial.update();
    
    // Handle WiFi connection
    if (wifiState == CONN_CONNECTING) {
        if (WiFi.status() == WL_CONNECTED) {
            wifiState = CONN_CONNECTED;
            Serial.println("WiFi connected!");
            Serial.print("IP: ");
            Serial.println(WiFi.localIP());
            
            // Sync RTC from NTP (will be checked in loop)
            // Set timezone for Italy: CET-1 means UTC+1 (POSIX sign is inverted)
            configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.google.com");
            ntpSynced = false;
            
            // Connect to MQTT
            connectMqtt();
        } else if (millis() - wifiConnectStart > WIFI_TIMEOUT_MS) {
            wifiState = CONN_ERROR;
            lastError = "WiFi timeout";
            Serial.println("WiFi connection timeout");
        }
    }
    
    // Handle MQTT connection
    if (wifiState == CONN_CONNECTED) {
        if (!mqttClient.connected()) {
            mqttState = CONN_DISCONNECTED;
            if (millis() - lastReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
                lastReconnectAttempt = millis();
                connectMqtt();
            }
        } else {
            mqttState = CONN_CONNECTED;
            mqttClient.loop();
        }
        
        // Check if NTP time is available and sync RTC
        if (!ntpSynced) {
            syncRtcFromNtp();
        }
    }
    
    // Handle alarm buzzer
    if (haAlarmState == "triggered") {
        if (!alarmBuzzerActive) {
            alarmBuzzerActive = true;
            lastBuzzerToggle = millis();
            buzzerState = true;
        }
        playAlarmBuzzer();
    } else {
        if (alarmBuzzerActive) {
            stopAlarmBuzzer();
        }
    }
    
    // Handle input
    handleEncoder();
    handleTouch();
    handleButton();
    
    // Draw current screen (only when needed or state changed)
    switch (currentScreen) {
        case SCREEN_MAIN:
            drawMainScreen();
            break;
        case SCREEN_KEYPAD:
            drawKeypadScreen();
            break;
        case SCREEN_SENSORS:
            drawSensorsScreen();
            break;
        case SCREEN_SETTINGS:
            drawSettingsScreen();
            break;
    }
    
    // Push sprite to display
    canvas.pushSprite(0, 0);
    
    delay(50);  // Reduce update rate to prevent flicker
}

void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(HA_WIFI_SSID, HA_WIFI_PASSWORD);
    wifiState = CONN_CONNECTING;
    wifiConnectStart = millis();
    Serial.printf("Connecting to WiFi: %s\n", HA_WIFI_SSID);
}

void connectMqtt() {
    if (WiFi.status() != WL_CONNECTED) return;
    
    mqttState = CONN_CONNECTING;
    Serial.printf("Connecting to MQTT: %s:%d\n", HA_MQTT_SERVER, HA_MQTT_PORT);
    
    if (mqttClient.connect("m5dial-ha", HA_MQTT_USER, HA_MQTT_PASSWORD)) {
        mqttState = CONN_CONNECTED;
        Serial.println("MQTT connected!");
        
        // Subscribe to topics
        mqttClient.subscribe(HA_MQTT_STATE_TOPIC);
        mqttClient.subscribe(HA_MQTT_STATUS_TOPIC);
        for (int i = 0; i < HA_SENSOR_COUNT; i++) {
            mqttClient.subscribe(haSensorTopics[i]);
        }
    } else {
        mqttState = CONN_ERROR;
        int rc = mqttClient.state();
        Serial.printf("MQTT failed, rc=%d\n", rc);
        switch(rc) {
            case -4: lastError = "MQTT timeout"; break;
            case -2: lastError = "MQTT conn fail"; break;
            case 4: lastError = "MQTT bad creds"; break;
            case 5: lastError = "MQTT unauth"; break;
            default: lastError = "MQTT error"; break;
        }
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    Serial.printf("MQTT [%s]: %s\n", topic, message.c_str());
    
    String topicStr = String(topic);
    
    if (topicStr == HA_MQTT_STATE_TOPIC) {
        haAlarmState = message;
    }
    
    // Check sensor topics
    for (int i = 0; i < HA_SENSOR_COUNT; i++) {
        if (topicStr == haSensorTopics[i]) {
            haSensorOpen[i] = (message == "ON" || message == "on" || message == "1" || message == "true");
            break;
        }
    }
}

void handleButton() {
    unsigned long now = millis();
    bool btnShortPress = false;
    bool btnLongPress = false;
    
    if (M5Dial.BtnA.isPressed()) {
        if (!btnWasPressed) {
            btnWasPressed = true;
            btnPressStart = now;
            btnLongTriggered = false;
        } else if (!btnLongTriggered && now - btnPressStart >= LONG_PRESS_MS) {
            btnLongPress = true;
            btnLongTriggered = true;
        }
    } else {
        if (btnWasPressed && !btnLongTriggered && (now - btnPressStart < LONG_PRESS_MS)) {
            btnShortPress = true;
        }
        btnWasPressed = false;
        btnLongTriggered = false;
    }
    
    // Handle button actions based on screen
    if (btnLongPress) {
        // Long press: go to settings from any screen, or back to main from settings
        if (currentScreen == SCREEN_SETTINGS) {
            currentScreen = SCREEN_MAIN;
            settingsEditing = false;
        } else {
            currentScreen = SCREEN_SETTINGS;
            settingsField = SET_HOUR;
            settingsEditing = false;
        }
        M5Dial.Speaker.tone(1000, 50);
    }
    
    if (btnShortPress) {
        M5Dial.Speaker.tone(2000, 20);
        
        switch (currentScreen) {
            case SCREEN_MAIN:
                // Short press on main: go to selected screen from menu
                currentScreen = menuScreens[menuSelection];
                if (currentScreen == SCREEN_KEYPAD) {
                    codeLength = 0;
                    enteredCode[0] = '\0';
                    selectedKey = 5;
                    shuffleKeypad();
                }
                break;
                
            case SCREEN_KEYPAD:
                // Short press on keypad: select current key
                handleKeypadSelect();
                break;
                
            case SCREEN_SENSORS:
                // Short press on sensors: back to main
                currentScreen = SCREEN_MAIN;
                break;
                
            case SCREEN_SETTINGS:
                // Short press on settings: toggle edit mode or confirm
                if (settingsEditing) {
                    // Confirm edit, move to next field
                    settingsEditing = false;
                    settingsField = (SettingsField)((settingsField + 1) % SET_FIELD_COUNT);
                } else {
                    // Enter edit mode
                    settingsEditing = true;
                }
                break;
        }
    }
}

void handleEncoder() {
    long newValue = M5Dial.Encoder.read();
    long delta = newValue - lastEncoderValue;
    
    if (delta == 0) return;
    
    lastEncoderValue = newValue;
    encoderAccumulator += delta;
    
    // Require 2 steps for action (reduces sensitivity)
    if (abs(encoderAccumulator) < 2) return;
    
    int direction = (encoderAccumulator > 0) ? 1 : -1;
    encoderAccumulator = 0;
    
    M5Dial.Speaker.tone(4000, 10);
    
    switch (currentScreen) {
        case SCREEN_MAIN:
            // Encoder on main: cycle through menu selection (shown on border)
            menuSelection = (menuSelection + direction + MENU_COUNT) % MENU_COUNT;
            break;
            
        case SCREEN_KEYPAD:
            // Encoder on keypad: navigate keys
            selectedKey = (selectedKey + direction + 12) % 12;
            break;
            
        case SCREEN_SENSORS:
            // Encoder on sensors: scroll (if needed) or go back
            if (direction < 0) {
                currentScreen = SCREEN_MAIN;
            }
            break;
            
        case SCREEN_SETTINGS:
            if (settingsEditing) {
                // Adjust current field value
                adjustSettingsField(direction);
            } else {
                // Navigate fields
                settingsField = (SettingsField)((settingsField + direction + SET_FIELD_COUNT) % SET_FIELD_COUNT);
            }
            break;
    }
}

void handleTouch() {
    auto touch = M5Dial.Touch.getDetail();
    
    if (!touch.wasPressed()) return;
    
    int tx = touch.x;
    int ty = touch.y;
    
    if (currentScreen == SCREEN_KEYPAD) {
        // Check if touch is on CLR button (left side)
        if (tx >= 10 && tx < 10 + SIDE_BTN_WIDTH + 10 &&
            ty >= KEYPAD_START_Y && ty < KEYPAD_START_Y + SIDE_BTN_HEIGHT) {
            selectedKey = 10;  // CLR
            handleKeypadSelect();
            M5Dial.Speaker.tone(2000, 20);
        }
        // Check if touch is on OK button (right side)
        else if (tx >= 190 && tx < 230 &&
                 ty >= KEYPAD_START_Y && ty < KEYPAD_START_Y + SIDE_BTN_HEIGHT) {
            selectedKey = 11;  // OK
            handleKeypadSelect();
            M5Dial.Speaker.tone(2000, 20);
        }
        // Check if touch is on number keypad (3x3 grid + 0)
        else if (tx >= KEYPAD_START_X && tx < KEYPAD_START_X + KEYPAD_COLS * KEY_WIDTH &&
                 ty >= KEYPAD_START_Y && ty < KEYPAD_START_Y + KEYPAD_ROWS * KEY_HEIGHT) {
            
            int col = (tx - KEYPAD_START_X) / KEY_WIDTH;
            int row = (ty - KEYPAD_START_Y) / KEY_HEIGHT;
            
            int keyIndex = -1;
            if (row < 3) {
                // Keys 1-9
                keyIndex = row * 3 + col;
            } else if (row == 3 && col == 1) {
                // Key 0 (center of bottom row)
                keyIndex = 9;
            }
            
            if (keyIndex >= 0 && keyIndex < 10) {
                selectedKey = keyIndex;
                handleKeypadSelect();
                M5Dial.Speaker.tone(2000, 20);
            }
        }
    } else if (currentScreen == SCREEN_MAIN) {
        // Touch on main screen: arm directly, but require code to disarm
        bool isArmed = (haAlarmState == "armed_away" || haAlarmState == "armed_home" || 
                        haAlarmState == "armed_night" || haAlarmState == "arming" ||
                        haAlarmState == "pending" || haAlarmState == "triggered");
        
        if (isArmed) {
            // Alarm is armed - go to keypad to enter code for disarming
            currentScreen = SCREEN_KEYPAD;
            codeLength = 0;
            enteredCode[0] = '\0';
            selectedKey = 5;
            shuffleKeypad();
        } else {
            // Alarm is disarmed - arm directly without code
            if (mqttClient.connected()) {
                mqttClient.publish(HA_MQTT_COMMAND_TOPIC, "ARM_AWAY");
                Serial.println("Sent: ARM_AWAY");
            }
        }
        M5Dial.Speaker.tone(2000, 20);
    } else if (currentScreen == SCREEN_SENSORS) {
        // Touch anywhere to go back
        currentScreen = SCREEN_MAIN;
        M5Dial.Speaker.tone(2000, 20);
    }
}

void handleKeypadSelect() {
    char key = keypadChars[selectedKey];
    
    if (key == 'C') {
        // Clear
        codeLength = 0;
        enteredCode[0] = '\0';
    } else if (key == '>') {
        // Submit code
        if (codeLength > 0) {
            submitCode();
        }
    } else {
        // Add digit
        if (codeLength < CODE_MAX_LEN) {
            enteredCode[codeLength++] = key;
            enteredCode[codeLength] = '\0';
        }
    }
}

void submitCode() {
    // Send the code to Home Assistant
    // The code is typically used for arming/disarming
    bool isArmed = (haAlarmState == "armed_away" || haAlarmState == "armed_home" || 
                    haAlarmState == "armed_night" || haAlarmState == "triggered");
    
    if (isArmed) {
        // Validate PIN before disarming
        if (strcmp(enteredCode, HA_ALARM_PIN) != 0) {
            // Wrong PIN - play error tone and clear
            Serial.printf("Wrong PIN entered: %s\n", enteredCode);
            M5Dial.Speaker.tone(200, 500);  // Low error tone
            codeLength = 0;
            enteredCode[0] = '\0';
            return;  // Stay on keypad screen
        }
    }
    
    if (mqttClient.connected()) {
        // Build command with code
        String command = isArmed ? "DISARM" : "ARM_AWAY";
        if (mqttClient.publish(HA_MQTT_COMMAND_TOPIC, command.c_str())) {
            Serial.printf("Sent: %s (code: %s)\n", command.c_str(), enteredCode);
            M5Dial.Speaker.tone(1000, 100);  // Success tone
        }
    }
    
    // Clear code and go back to main
    codeLength = 0;
    enteredCode[0] = '\0';
    currentScreen = SCREEN_MAIN;
}

void toggleAlarm() {
    bool isArmed = (haAlarmState == "armed_away" || haAlarmState == "armed_home" || 
                    haAlarmState == "armed_night" || haAlarmState == "arming" ||
                    haAlarmState == "pending");
    
    if (mqttClient.connected()) {
        const char* command = isArmed ? "DISARM" : "ARM_AWAY";
        if (mqttClient.publish(HA_MQTT_COMMAND_TOPIC, command)) {
            Serial.printf("Sent: %s\n", command);
            M5Dial.Speaker.tone(1000, 100);
        }
    }
}

void adjustSettingsField(int direction) {
    // Get current RTC time
    auto dt = M5Dial.Rtc.getDateTime();
    
    switch (settingsField) {
        case SET_HOUR:
            dt.time.hours = (dt.time.hours + direction + 24) % 24;
            break;
        case SET_MIN:
            dt.time.minutes = (dt.time.minutes + direction + 60) % 60;
            break;
        case SET_DAY:
            dt.date.date = ((dt.date.date - 1 + direction + 31) % 31) + 1;
            break;
        case SET_MONTH:
            dt.date.month = ((dt.date.month - 1 + direction + 12) % 12) + 1;
            break;
        case SET_YEAR:
            dt.date.year = constrain(dt.date.year + direction, 2020, 2099);
            break;
        default:
            break;
    }
    
    // Update RTC
    M5Dial.Rtc.setDateTime(dt);
}

// Draw WiFi icon (SVG-style wireless signal arcs) - white when connected, gray when not
void drawWifiIcon(int x, int y, bool connected) {
    uint16_t color = connected ? COLOR_TEXT : COLOR_DIM;  // White if connected, gray if not
    
    // Draw concentric arcs for WiFi symbol (SVG-style with thicker lines)
    // Small dot at bottom (the transmitter point)
    canvas.fillCircle(x, y + 8, 3, color);
    
    // Arc 1 (smallest) - draw with thickness
    for (int a = -40; a <= 40; a += 2) {
        float rad = a * PI / 180.0;
        int px = x + (int)(6 * sin(rad));
        int py = y + 5 - (int)(6 * cos(rad));
        canvas.fillCircle(px, py, 1, color);
    }
    
    // Arc 2 (medium)
    for (int a = -40; a <= 40; a += 2) {
        float rad = a * PI / 180.0;
        int px = x + (int)(10 * sin(rad));
        int py = y + 5 - (int)(10 * cos(rad));
        canvas.fillCircle(px, py, 1, color);
    }
    
    // Arc 3 (largest)
    for (int a = -40; a <= 40; a += 2) {
        float rad = a * PI / 180.0;
        int px = x + (int)(14 * sin(rad));
        int py = y + 5 - (int)(14 * cos(rad));
        canvas.fillCircle(px, py, 1, color);
    }
}

void drawStatusBar() {
    // Glass panel for status bar
    drawGlassPanel(10, 10, 220, 30, 15);

    // Status indicators at top, beside the clock
    int y = 25;

    // WiFi icon at left of clock
    bool wifiOk = (wifiState == CONN_CONNECTED);
    drawWifiIcon(CENTER_X - 50, y, wifiOk);

    // Time at top center with shadow
    auto dt = M5Dial.Rtc.getDateTime();
    char timeStr[16];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d", dt.time.hours, dt.time.minutes);
    canvas.setTextDatum(middle_center);
    canvas.setTextSize(1);
    // Shadow
    canvas.setTextColor(COLOR_DARK_GRAY);
    canvas.drawString(timeStr, CENTER_X + 1, y + 1);
    // Main text
    canvas.setTextColor(COLOR_TEXT);
    canvas.drawString(timeStr, CENTER_X, y);

    // MQTT label at right of clock with indicator dot
    canvas.setTextDatum(middle_center);
    if (mqttState == CONN_CONNECTED) {
        // Green dot for connected
        canvas.fillCircle(CENTER_X + 38, y, 3, COLOR_OK);
        canvas.setTextColor(COLOR_TEXT);  // White when connected
    } else {
        // Red dot for disconnected
        canvas.fillCircle(CENTER_X + 38, y, 3, COLOR_ERROR);
        canvas.setTextColor(COLOR_ERROR);  // Red when disconnected
    }
    canvas.drawString("MQTT", CENTER_X + 56, y);
}

// Draw text along circular arc (for menu items on border)
// Text is drawn character by character following the arc
// clockwise=true: text reads clockwise, clockwise=false: text reads anticlockwise
void drawTextOnArc(const char* text, float centerAngleDeg, int radius, uint16_t color, bool highlight, bool clockwise = true) {
    int len = strlen(text);
    float charSpacing = 8.0;  // Degrees between characters
    float totalSpan = (len - 1) * charSpacing;
    float startAngle;
    if (clockwise) {
        startAngle = centerAngleDeg - totalSpan / 2.0;
    } else {
        // Anticlockwise: start from the end
        startAngle = centerAngleDeg + totalSpan / 2.0;
    }
    
    canvas.setTextDatum(middle_center);
    canvas.setTextSize(1);
    
    // Draw highlight arc segment behind text if selected
    if (highlight) {
        // Calculate arc bounds correctly for both clockwise and anticlockwise
        float arcStartAngle, arcEndAngle;
        if (clockwise) {
            arcStartAngle = startAngle - 5;
            arcEndAngle = startAngle + totalSpan + 5;
        } else {
            // For anticlockwise, startAngle is at the end of the word
            arcEndAngle = startAngle + 5;
            arcStartAngle = startAngle - totalSpan - 5;
        }
        float arcStart = arcStartAngle * PI / 180.0;
        float arcEnd = arcEndAngle * PI / 180.0;

        // Draw glow effect
        for (int layer = 3; layer > 0; layer--) {
            uint16_t glowColor = interpolateColor(COLOR_BG, COLOR_ORANGE, layer / 3.0f);
            for (float a = arcStart; a <= arcEnd; a += 0.02) {
                int x = CENTER_X + (int)((radius) * sin(a));
                int y = CENTER_Y - (int)((radius) * cos(a));
                canvas.fillCircle(x, y, 8 + layer * 2, glowColor);
            }
        }

        // Main highlight
        for (float a = arcStart; a <= arcEnd; a += 0.02) {
            int x = CENTER_X + (int)((radius) * sin(a));
            int y = CENTER_Y - (int)((radius) * cos(a));
            canvas.fillCircle(x, y, 8, COLOR_ORANGE);  // Orange highlight
        }
        canvas.setTextColor(COLOR_TEXT);  // White text on orange
    } else {
        canvas.setTextColor(color);
    }
    
    // Draw each character along the arc
    for (int i = 0; i < len; i++) {
        float angleDeg;
        if (clockwise) {
            angleDeg = startAngle + i * charSpacing;
        } else {
            angleDeg = startAngle - i * charSpacing;
        }
        float angleRad = angleDeg * PI / 180.0;
        int x = CENTER_X + (int)(radius * sin(angleRad));
        int y = CENTER_Y - (int)(radius * cos(angleRad));
        
        char ch[2] = {text[i], '\0'};
        canvas.drawString(ch, x, y);
    }
}

void drawMainScreen() {
    // Beautiful gradient background
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
        uint16_t lineColor = interpolateColor(COLOR_BG, COLOR_BLUE, y / (float)SCREEN_HEIGHT);
        canvas.drawFastHLine(0, y, SCREEN_WIDTH, lineColor);
    }

    // Draw menu labels around the circular border (3 items: KEYPAD, SENSORS, SETTINGS)
    // KEYPAD at 270° (left side), SENSORS at 90° (right side), SETTINGS at 180° (bottom)
    // KEYPAD and SETTINGS are drawn anticlockwise so they read correctly
    float angles[MENU_COUNT] = {270, 90, 180};  // KEYPAD left, SENSORS right, SETTINGS bottom
    bool anticlockwiseFlags[MENU_COUNT] = {true, false, true};  // KEYPAD anticlockwise, SENSORS clockwise, SETTINGS anticlockwise
    for (int i = 0; i < MENU_COUNT; i++) {
        bool isSelected = (i == menuSelection);
        drawTextOnArc(menuNames[i], angles[i], 108, COLOR_DIM, isSelected, !anticlockwiseFlags[i]);
    }

    drawStatusBar();

    // Alarm state - large centered display
    canvas.setTextDatum(middle_center);

    // Draw circular background for alarm state
    uint16_t stateColor = COLOR_DIM;
    String stateText = haAlarmState;
    bool isArmedState = false;
    bool isTriggeredState = false;

    if (haAlarmState == "armed_away" || haAlarmState == "armed_home" || haAlarmState == "armed_night") {
        stateColor = COLOR_ARMED;
        stateText = "ARMED";
        isArmedState = true;
    } else if (haAlarmState == "disarmed") {
        stateColor = COLOR_DISARMED;
        stateText = "DISARMED";
    } else if (haAlarmState == "pending" || haAlarmState == "arming") {
        stateColor = COLOR_WARN;
        stateText = "PENDING";
    } else if (haAlarmState == "triggered") {
        stateColor = COLOR_ERROR;
        stateText = "TRIGGERED";
        isTriggeredState = true;
    }

    // Calculate animated color for circle only (not text)
    // Sleeping man breathing: ~12-20 breaths/min = 0.2-0.33 Hz, use 5 second cycle (0.2 Hz)
    // Running man breathing: ~40-60 breaths/min = 0.67-1 Hz, use 400ms cycle (2.5 Hz)
    uint16_t circleColor = stateColor;
    float breathPhase = 0.0f;
    if (isArmedState || isTriggeredState) {
        unsigned long now = millis();
        float brightness;

        if (isTriggeredState) {
            // Fast pulsing for triggered state (400ms cycle = 2.5 Hz)
            float phase = (now % 400) / 400.0f;
            brightness = 0.3f + 0.7f * sin(phase * 2.0f * PI);
            breathPhase = brightness;
        } else {
            // Slow breathing for armed state (5000ms cycle = 0.2 Hz)
            float phase = (now % 5000) / 5000.0f;
            brightness = 0.4f + 0.6f * sin(phase * 2.0f * PI);
            breathPhase = brightness;
        }

        // Scale color with brightness
        uint8_t r = (uint8_t)(31 * brightness);  // 5 bits for red in RGB565
        circleColor = (r << 11);  // RGB565: RRRRR GGGGGG BBBBB
    }

    // Draw beautiful gradient circle with glow effect
    if (isArmedState || isTriggeredState) {
        // Outer glow
        for (int i = 0; i < 8; i++) {
            uint16_t glowColor = interpolateColor(COLOR_BG, circleColor, (8 - i) / 8.0f * breathPhase);
            canvas.drawCircle(CENTER_X, CENTER_Y, 70 + i, glowColor);
        }
    }

    // Main state circle with gradient fill
    drawGradientCircle(CENTER_X, CENTER_Y, 60, circleColor, stateColor);

    // Multiple circle outlines for depth
    canvas.drawCircle(CENTER_X, CENTER_Y, 60, stateColor);
    canvas.drawCircle(CENTER_X, CENTER_Y, 61, stateColor);
    canvas.drawCircle(CENTER_X, CENTER_Y, 62, interpolateColor(stateColor, COLOR_BG, 0.5f));

    // State text with shadow effect
    canvas.setTextColor(COLOR_DARK_GRAY);
    canvas.setTextSize(2);
    canvas.drawString(stateText, CENTER_X + 2, CENTER_Y + 2);
    canvas.setTextColor(stateColor);
    canvas.drawString(stateText, CENTER_X, CENTER_Y);
    canvas.setTextSize(1);

    // Sensor summary below center with glass panel
    int openCount = 0;
    String openSensorNames = "";
    for (int i = 0; i < HA_SENSOR_COUNT; i++) {
        if (haSensorOpen[i]) {
            openCount++;
            if (openSensorNames.length() > 0) {
                openSensorNames += ", ";
            }
            openSensorNames += haSensorNames[i];
        }
    }

    // Draw glass panel for sensor info
    drawGlassPanel(30, CENTER_Y + 52, 180, 28, 14);

    canvas.setTextDatum(middle_center);
    if (openCount == 0) {
        canvas.setTextColor(COLOR_OK);
        canvas.drawString("All closed", CENTER_X, CENTER_Y + 66);
    } else {
        canvas.setTextColor(COLOR_ERROR);
        // Show sensor names when triggered, otherwise show count
        if (isTriggeredState && openCount <= 2) {
            // Show names if 1-2 sensors open (fits on screen)
            canvas.drawString(openSensorNames, CENTER_X, CENTER_Y + 66);
        } else if (isTriggeredState) {
            // Too many open, show first name + count
            String firstOpen = "";
            for (int i = 0; i < HA_SENSOR_COUNT; i++) {
                if (haSensorOpen[i]) {
                    firstOpen = haSensorNames[i];
                    break;
                }
            }
            char buf[48];
            snprintf(buf, sizeof(buf), "%s +%d", firstOpen.c_str(), openCount - 1);
            canvas.drawString(buf, CENTER_X, CENTER_Y + 66);
        } else {
            char buf[32];
            snprintf(buf, sizeof(buf), "%d open", openCount);
            canvas.drawString(buf, CENTER_X, CENTER_Y + 66);
        }
    }
}

void drawKeypadScreen() {
    // Gradient background
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
        uint16_t lineColor = interpolateColor(COLOR_BG, COLOR_BLUE, y / (float)SCREEN_HEIGHT);
        canvas.drawFastHLine(0, y, SCREEN_WIDTH, lineColor);
    }

    drawStatusBar();

    // Title - only show "ENTER CODE" when no code entered
    canvas.setTextDatum(middle_center);
    if (codeLength == 0) {
        canvas.setTextColor(COLOR_ORANGE);
        canvas.drawString("ENTER CODE", CENTER_X, 48);
    }

    // Draw entered code (masked with asterisks) with glass panel
    if (codeLength > 0) {
        drawGlassPanel(60, 36, 120, 24, 12);
        canvas.setTextSize(2);
        canvas.setTextColor(COLOR_TEXT);
        String maskedCode = "";
        for (int i = 0; i < codeLength; i++) {
            maskedCode += "*";
        }
        canvas.drawString(maskedCode, CENTER_X, 48);
        canvas.setTextSize(1);
    }

    // Draw CLR button on left side with glass effect
    int clrX = 15;
    int clrY = KEYPAD_START_Y;
    bool clrSelected = (selectedKey == 10);

    if (clrSelected) {
        canvas.fillRoundRect(clrX - 8, clrY - 5, 26, SIDE_BTN_HEIGHT + 10, 13, COLOR_ACCENT);
    } else {
        drawGlassPanel(clrX - 8, clrY - 5, 26, SIDE_BTN_HEIGHT + 10, 13);
    }

    canvas.setTextColor(clrSelected ? COLOR_TEXT : COLOR_DIM);
    canvas.setTextDatum(middle_center);
    // Draw CLR vertically
    canvas.drawString("C", clrX, clrY + SIDE_BTN_HEIGHT/2 - 15);
    canvas.drawString("L", clrX, clrY + SIDE_BTN_HEIGHT/2);
    canvas.drawString("R", clrX, clrY + SIDE_BTN_HEIGHT/2 + 15);

    // Draw OK button on right side with glass effect
    int okX = 225;
    int okY = KEYPAD_START_Y;
    bool okSelected = (selectedKey == 11);

    if (okSelected) {
        canvas.fillRoundRect(okX - 8, okY - 5, 26, SIDE_BTN_HEIGHT + 10, 13, COLOR_OK);
    } else {
        drawGlassPanel(okX - 8, okY - 5, 26, SIDE_BTN_HEIGHT + 10, 13);
    }

    canvas.setTextColor(okSelected ? COLOR_TEXT : COLOR_DIM);
    canvas.setTextDatum(middle_center);
    // Draw OK vertically
    canvas.drawString("O", okX, okY + SIDE_BTN_HEIGHT/2 - 8);
    canvas.drawString("K", okX, okY + SIDE_BTN_HEIGHT/2 + 8);

    // Draw number keypad (3x3 grid for 1-9, then 0 centered at bottom) with beautiful buttons
    for (int i = 0; i < 10; i++) {
        int row, col, x, y;
        if (i < 9) {
            // Keys 1-9 in 3x3 grid
            row = i / 3;
            col = i % 3;
            x = KEYPAD_START_X + col * KEY_WIDTH;
            y = KEYPAD_START_Y + row * KEY_HEIGHT;
        } else {
            // Key 0 centered at bottom
            row = 3;
            col = 1;  // Center column
            x = KEYPAD_START_X + col * KEY_WIDTH;
            y = KEYPAD_START_Y + row * KEY_HEIGHT;
        }

        bool isSelected = (i == selectedKey);

        // Draw key with gradient and glow effect
        if (isSelected) {
            // Glow effect for selected key
            for (int g = 0; g < 4; g++) {
                uint16_t glowColor = interpolateColor(COLOR_BG, COLOR_ORANGE, (4 - g) / 4.0f);
                canvas.drawRoundRect(x + 2 - g, y + 2 - g, KEY_WIDTH - 4 + g*2, KEY_HEIGHT - 4 + g*2, 8, glowColor);
            }
            // Filled gradient button
            canvas.fillRoundRect(x + 2, y + 2, KEY_WIDTH - 4, KEY_HEIGHT - 4, 8, COLOR_ORANGE);
            // Highlight at top
            canvas.fillRoundRect(x + 4, y + 4, KEY_WIDTH - 8, 3, 6, interpolateColor(COLOR_ORANGE, COLOR_TEXT, 0.3f));
            canvas.setTextColor(COLOR_TEXT);
        } else {
            // Glass effect for unselected keys
            drawGlassPanel(x + 2, y + 2, KEY_WIDTH - 4, KEY_HEIGHT - 4, 8);
            canvas.setTextColor(COLOR_TEXT);
        }

        // Draw key label with shadow
        canvas.setTextDatum(middle_center);
        canvas.setTextSize(2);
        char keyLabel[2] = {keypadChars[i], '\0'};
        if (!isSelected) {
            canvas.setTextColor(COLOR_DARK_GRAY);
            canvas.drawString(keyLabel, x + KEY_WIDTH/2 + 1, y + KEY_HEIGHT/2 + 1);
        }
        canvas.setTextColor(COLOR_TEXT);
        canvas.drawString(keyLabel, x + KEY_WIDTH/2, y + KEY_HEIGHT/2);
        canvas.setTextSize(1);
    }
}

void drawSensorsScreen() {
    // Gradient background
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
        uint16_t lineColor = interpolateColor(COLOR_BG, COLOR_BLUE, y / (float)SCREEN_HEIGHT);
        canvas.drawFastHLine(0, y, SCREEN_WIDTH, lineColor);
    }

    drawStatusBar();

    // Title with glow effect
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COLOR_ACCENT);
    canvas.drawString("SENSORS", CENTER_X, 48);

    // Draw sensor list in 2 columns with glass panels
    int startY = 65;
    int lineHeight = 15;
    int colWidth = 115;  // Width for each column
    int col1X = 15;      // Left column X position
    int col2X = 125;     // Right column X position
    int rowsPerCol = (HA_SENSOR_COUNT + 1) / 2;  // Ceiling division

    canvas.setTextDatum(middle_left);

    for (int i = 0; i < HA_SENSOR_COUNT; i++) {
        int col = i / rowsPerCol;  // 0 = left column, 1 = right column
        int row = i % rowsPerCol;
        int x = (col == 0) ? col1X : col2X;
        int y = startY + row * lineHeight;

        // Draw glass panel for each sensor
        drawGlassPanel(x - 2, y - 7, colWidth - 10, 14, 7);

        // Status indicator with glow
        if (haSensorOpen[i]) {
            // Red glow for open sensors
            for (int g = 0; g < 3; g++) {
                canvas.drawCircle(x + 5, y, 4 + g, interpolateColor(COLOR_BG, COLOR_ERROR, (3 - g) / 3.0f));
            }
            canvas.fillCircle(x + 5, y, 4, COLOR_ERROR);
            canvas.setTextColor(COLOR_ERROR);
        } else {
            // Green indicator for closed sensors
            canvas.fillCircle(x + 5, y, 4, COLOR_OK);
            canvas.setTextColor(COLOR_DIM);
        }

        // Sensor name (shortened to fit column)
        canvas.drawString(haSensorNames[i], x + 14, y);
    }

    // Hint with glass panel
    drawGlassPanel(50, 207, 140, 20, 10);
    canvas.setTextColor(COLOR_ACCENT);
    canvas.setTextDatum(middle_center);
    canvas.drawString("Press/Touch: Back", CENTER_X, 217);
}

void drawSettingsScreen() {
    // Gradient background
    for (int y = 0; y < SCREEN_HEIGHT; y++) {
        uint16_t lineColor = interpolateColor(COLOR_BG, COLOR_BLUE, y / (float)SCREEN_HEIGHT);
        canvas.drawFastHLine(0, y, SCREEN_WIDTH, lineColor);
    }

    drawStatusBar();

    // Title
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(COLOR_ORANGE);
    canvas.drawString("SETTINGS", CENTER_X, 48);

    // Get current RTC time
    auto dt = M5Dial.Rtc.getDateTime();

    // Field names and values
    const char* fieldNames[] = {"Hour", "Min", "Day", "Month", "Year"};
    int fieldValues[] = {
        dt.time.hours,
        dt.time.minutes,
        dt.date.date,
        dt.date.month,
        dt.date.year
    };

    int startY = 70;
    int lineHeight = 24;

    for (int i = 0; i < SET_FIELD_COUNT; i++) {
        int y = startY + i * lineHeight;
        bool isSelected = (i == settingsField);

        // Draw glass panel for each field
        drawGlassPanel(30, y - 10, 180, 22, 11);

        // Highlight selected field with glow
        if (isSelected) {
            if (settingsEditing) {
                // Glow effect when editing
                for (int g = 0; g < 3; g++) {
                    canvas.drawRoundRect(30 - g, y - 10 - g, 180 + g*2, 22 + g*2, 11,
                                       interpolateColor(COLOR_BG, COLOR_OK, (3 - g) / 3.0f));
                }
                canvas.fillRoundRect(30, y - 10, 180, 22, 11, COLOR_OK);  // Green when editing
                // Highlight
                canvas.fillRoundRect(32, y - 8, 176, 4, 9, interpolateColor(COLOR_OK, COLOR_TEXT, 0.5f));
            } else {
                // Glow effect when selected
                for (int g = 0; g < 3; g++) {
                    canvas.drawRoundRect(30 - g, y - 10 - g, 180 + g*2, 22 + g*2, 11,
                                       interpolateColor(COLOR_BG, COLOR_ORANGE, (3 - g) / 3.0f));
                }
                canvas.fillRoundRect(30, y - 10, 180, 22, 11, COLOR_ORANGE);  // Orange when selected
                // Highlight
                canvas.fillRoundRect(32, y - 8, 176, 4, 9, interpolateColor(COLOR_ORANGE, COLOR_TEXT, 0.5f));
            }
            canvas.setTextColor(COLOR_TEXT);
        } else {
            canvas.setTextColor(COLOR_DIM);
        }

        // Field name
        canvas.setTextDatum(middle_left);
        canvas.drawString(fieldNames[i], 40, y);

        // Field value
        canvas.setTextDatum(middle_right);
        char valStr[8];
        if (i == SET_YEAR) {
            snprintf(valStr, sizeof(valStr), "%04d", fieldValues[i]);
        } else {
            snprintf(valStr, sizeof(valStr), "%02d", fieldValues[i]);
        }
        canvas.drawString(valStr, 200, y);
    }

    // Hint - positioned to fit on screen
    drawGlassPanel(15, 185, 210, 32, 10);
    canvas.setTextColor(COLOR_ACCENT);
    canvas.setTextDatum(middle_center);
    if (settingsEditing) {
        canvas.drawString("Turn: Adjust | Press: Next", CENTER_X, 192);
    } else {
        canvas.drawString("Turn: Select | Press: Edit", CENTER_X, 192);
    }
    canvas.setTextColor(COLOR_DIM);
    canvas.drawString("Long Press: Back", CENTER_X, 208);
}

// Set RTC from compile time (__DATE__ and __TIME__ macros)
// Only sets if RTC year is before 2024 (indicates unset or reset RTC)
void setRtcFromCompileTime() {
    // Check current RTC time
    auto dt = M5Dial.Rtc.getDateTime();
    
    // If year is reasonable (>= 2024), assume RTC is already set
    if (dt.date.year >= 2024) {
        Serial.printf("RTC already set: %04d-%02d-%02d %02d:%02d:%02d\n",
                      dt.date.year, dt.date.month, dt.date.date,
                      dt.time.hours, dt.time.minutes, dt.time.seconds);
        return;
    }
    
    // Parse __DATE__ which is "Mmm DD YYYY" format (e.g., "Dec 27 2025")
    const char* dateStr = __DATE__;
    const char* timeStr = __TIME__;  // "HH:MM:SS" format
    
    // Month names
    const char* months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    
    // Parse month
    int month = 1;
    for (int i = 0; i < 12; i++) {
        if (strncmp(dateStr, months[i], 3) == 0) {
            month = i + 1;
            break;
        }
    }
    
    // Parse day and year from __DATE__
    int day = atoi(dateStr + 4);
    int year = atoi(dateStr + 7);
    
    // Parse time from __TIME__
    int hour = atoi(timeStr);
    int minute = atoi(timeStr + 3);
    int second = atoi(timeStr + 6);
    
    // Set RTC
    m5::rtc_datetime_t newDt;
    newDt.date.year = year;
    newDt.date.month = month;
    newDt.date.date = day;
    newDt.time.hours = hour;
    newDt.time.minutes = minute;
    newDt.time.seconds = second;
    
    M5Dial.Rtc.setDateTime(newDt);
    
    Serial.printf("RTC set from compile time: %04d-%02d-%02d %02d:%02d:%02d\n",
                  year, month, day, hour, minute, second);
}

// Sync RTC from NTP server (called when WiFi is connected)
void syncRtcFromNtp() {
    time_t now;
    time(&now);
    
    // Check if time is valid (after year 2020)
    if (now < 1577836800) {  // Jan 1, 2020 in Unix time
        // NTP not yet available, will retry next loop
        return;
    }
    
    // Use localtime to get local time with timezone applied
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 1000)) {
        return;  // NTP not ready yet
    }
    
    // NTP time obtained successfully - update RTC
    m5::rtc_datetime_t newDt;
    newDt.date.year = timeinfo.tm_year + 1900;
    newDt.date.month = timeinfo.tm_mon + 1;
    newDt.date.date = timeinfo.tm_mday;
    newDt.time.hours = timeinfo.tm_hour;
    newDt.time.minutes = timeinfo.tm_min;
    newDt.time.seconds = timeinfo.tm_sec;
    
    M5Dial.Rtc.setDateTime(newDt);
    ntpSynced = true;
    
    Serial.printf("RTC synced from NTP: %04d-%02d-%02d %02d:%02d:%02d\n",
                  newDt.date.year, newDt.date.month, newDt.date.date,
                  newDt.time.hours, newDt.time.minutes, newDt.time.seconds);
}

// Shuffle the keypad numbers randomly (Fisher-Yates shuffle)
void shuffleKeypad() {
    // Reset to default first
    for (int i = 0; i < 12; i++) {
        keypadChars[i] = keypadCharsDefault[i];
    }
    
    // Shuffle only the digits (indices 0-9), keep CLR and OK in place
    for (int i = 9; i > 0; i--) {
        int j = random(0, i + 1);
        // Swap keypadChars[i] and keypadChars[j]
        char temp = keypadChars[i];
        keypadChars[i] = keypadChars[j];
        keypadChars[j] = temp;
    }
    
    Serial.print("Keypad shuffled: ");
    for (int i = 0; i < 10; i++) {
        Serial.print(keypadChars[i]);
    }
    Serial.println();
}

// Play alarm buzzer tone (alternating high-low siren)
void playAlarmBuzzer() {
    unsigned long now = millis();
    
    // Toggle between two frequencies every 800ms for slower siren effect
    if (now - lastBuzzerToggle >= 800) {
        lastBuzzerToggle = now;
        buzzerState = !buzzerState;
        
        if (buzzerState) {
            M5Dial.Speaker.tone(800, 800);   // High tone, longer duration
        } else {
            M5Dial.Speaker.tone(500, 800);   // Low tone, longer duration
        }
    }
}

// Stop alarm buzzer
void stopAlarmBuzzer() {
    alarmBuzzerActive = false;
    M5Dial.Speaker.stop();
}

// ========== Beautiful UI Helper Functions ==========

// Interpolate between two RGB565 colors
uint16_t interpolateColor(uint16_t color1, uint16_t color2, float t) {
    if (t <= 0.0f) return color1;
    if (t >= 1.0f) return color2;

    // Extract RGB components from RGB565
    uint8_t r1 = (color1 >> 11) & 0x1F;
    uint8_t g1 = (color1 >> 5) & 0x3F;
    uint8_t b1 = color1 & 0x1F;

    uint8_t r2 = (color2 >> 11) & 0x1F;
    uint8_t g2 = (color2 >> 5) & 0x3F;
    uint8_t b2 = color2 & 0x1F;

    // Interpolate each component
    uint8_t r = r1 + (uint8_t)((r2 - r1) * t);
    uint8_t g = g1 + (uint8_t)((g2 - g1) * t);
    uint8_t b = b1 + (uint8_t)((b2 - b1) * t);

    // Combine back to RGB565
    return (r << 11) | (g << 5) | b;
}

// Draw a gradient-filled circle
void drawGradientCircle(int cx, int cy, int radius, uint16_t color1, uint16_t color2) {
    for (int r = radius; r > 0; r--) {
        float t = 1.0f - (r / (float)radius);
        uint16_t color = interpolateColor(color1, color2, t * 0.3f);
        canvas.drawCircle(cx, cy, r, color);
    }
}

// Draw a glassmorphic panel (modern UI effect)
void drawGlassPanel(int x, int y, int w, int h, int radius) {
    // Semi-transparent dark background
    canvas.fillRoundRect(x, y, w, h, radius, COLOR_GLASS);

    // Lighter border on top-left (highlight)
    canvas.drawRoundRect(x, y, w, h, radius, interpolateColor(COLOR_TEXT, COLOR_GLASS, 0.7f));

    // Darker border on bottom-right (shadow)
    canvas.drawRoundRect(x + 1, y + 1, w - 2, h - 2, radius - 1, COLOR_DARK_GRAY);
}

// Draw a smooth arc with thickness
void drawSmoothArc(int cx, int cy, int radius, float startAngle, float endAngle, int thickness, uint16_t color) {
    for (int t = 0; t < thickness; t++) {
        int r = radius - t;
        for (float a = startAngle; a <= endAngle; a += 0.02f) {
            int x = cx + (int)(r * cos(a));
            int y = cy + (int)(r * sin(a));
            canvas.drawPixel(x, y, color);
        }
    }
}
