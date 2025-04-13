//
// #include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>
#include <float.h>

// za BMP280 (temp)
#define BMP_SCK (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS (10)

Adafruit_BMP280 bmp;  // I2C

#include "secrets.h"
#include "settings.h"

// vrednosti iz secrets.h za Wi-Fi
const char *wifiSSID = WIFI_SSID;
const char *wifiPASS = WIFI_PASS;

// vrednosti iz secrets.h za MQTT
const char *mqttBroker = MQTT_BROKER;
const int mqttPort = MQTT_PORT;
// WARNING: mqtt credentials se v resnici ne uporabljajo
// saj je dovoljen dostop vsem
const char *mqttUser = MQTT_USER;
const char *mqttPass = MQTT_PASS;
// mqttBaseTopic predstavlja osnovni topic kamor se podatki pošiljajo
// če bi naredili drugačen projekt bi uporabili drugačen mqttBaseTopci
// const char *mqttBaseTopic = "/iot/grega";

// TODO: change mqttBaseTopic to "/iot/roleta/babi";
const char *mqttBaseTopic = "/iot/roleta/babi";

// Ustvari instanco razreda WiFiClient, ki omogoča povezavo z Wi-Fi omrežjem
WiFiClient wifiClient;

// Ustvari instanco razreda PubSubClient, ki omogoča povezavo z MQTT strežnikom
// PubSubClient uporablja objekt wifiClient za komunikacijo preko TCP/IP
PubSubClient mqttClient(wifiClient);

void setupBmp() {
    unsigned status;
    status = bmp.begin(0x76);
    if (!status) {
        Serial.println(F(
            "Could not find a valid BMP280 sensor, check wiring or "
            "try a different address!"));
        Serial.print("SensorID was: 0x");
        Serial.println(bmp.sensorID(), 16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
    } else {
        /* Default settings from datasheet. */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    }
}

struct Roleta {
    String id;
    uint8_t powerPinUp;
    uint8_t powerPinDown;
    uint8_t command;
    bool hasCommandChanged;
    long powerStartedTime;
};
Roleta roletas[2];

struct LightBulb {
    String id;
    uint8_t powerPin;
    uint8_t command;
};
LightBulb lightBulbs[2];

void setLightBulbCommand(const String lightBulbId, uint8_t command) {
    uint8_t lightBulbIndex = getLightBulbIndexById(lightBulbId);
    if (lightBulbIndex == 255) {
        return;
    }
    // Serial.println(lightBulbIndex);
    // Serial.println(lightBulbId);
    // pazi tukaj je če več kot ena ker pri roletah je lahko tudi 2
    if (command > 1) {
        command = 0;
    }
    // Serial.println(lightBulbId);
    // Serial.println(lightBulbIndex);
    // Serial.println(command);
    // Serial.print("lightBulbIndex");
    // Serial.println(lightBulbIndex);
    lightBulbs[lightBulbIndex].command = command;
}

struct RgbwLight {
    uint8_t redPin;
    uint8_t greenPin;
    uint8_t bluePin;
    uint8_t whitePin;

    uint8_t startRed;
    uint8_t startRedTemp;
    int diffRed;
    uint8_t endRed;

    uint8_t startGreen;
    uint8_t startGreenTemp;
    int diffGreen;
    uint8_t endGreen;

    uint8_t startBlue;
    uint8_t startBlueTemp;
    uint8_t diffBlue;
    uint8_t endBlue;

    uint8_t startWhite;
    uint8_t startWhiteTemp;
    int diffWhite;
    uint8_t endWhite;

    uint8_t currRed;
    uint8_t currGreen;
    uint8_t currBlue;
    uint8_t currWhite;

    int transitionTimeMS;
    int transitionStartTime;

    void setRgbwPins(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
        redPin = red;
        greenPin = green;
        bluePin = blue;
        whitePin = white;

        ledcAttach(redPin, LED_FREQ, LED_RES);
        ledcAttach(greenPin, LED_FREQ, LED_RES);
        ledcAttach(bluePin, LED_FREQ, LED_RES);
        ledcAttach(whitePin, LED_FREQ, LED_RES);
        int redValue = 0;
        int greenValue = 0;
        int blueValue = 0;
        int whiteValue = 0;
        setRGBW(redValue, greenValue, blueValue, whiteValue);
    }

    // hex with duration has last 4 chars with duration
    void setRGBWHexWithDur(String hexRGBWDur) {
        char *endptr;  // Pointer to store the end of the conversion
        // Extract substrings for each color component and convert to uint8_t
        endRed = strtoul(hexRGBWDur.substring(0, 2).c_str(), &endptr, 16);
        endGreen = strtoul(hexRGBWDur.substring(2, 4).c_str(), &endptr, 16);
        endBlue = strtoul(hexRGBWDur.substring(4, 6).c_str(), &endptr, 16);
        endWhite = strtoul(hexRGBWDur.substring(6, 8).c_str(), &endptr, 16);
        transitionTimeMS = strtol(hexRGBWDur.substring(8, 12).c_str(), &endptr, 16);
        startRedTemp = startRed;
        startGreenTemp = startGreen;
        startBlueTemp = startBlue;
        startWhiteTemp = startWhite;
        setRGBW(endRed, endGreen, endBlue, endWhite);
        diffRed = endRed - startRed;
        diffGreen = endGreen - startGreen;
        diffBlue = endBlue - startBlue;
        diffWhite = endWhite - startWhite;
        transitionStartTime = millis();
    }
    void transitionLoop() {
        return;
        // double precentageComplete = (millis() - transitionStartTime) / (double)transitionTimeMS;
        // if (transitionTimeMS == 0) {
        //     startRed = endRed;
        //     startGreen = endGreen;
        //     startBlue = endBlue;
        //     startWhite = endWhite;
        //     setRGBW(endRed, endGreen, endBlue, endWhite);
        //     return;
        // }
        // // Serial.println(precentageComplete);
        // else if (precentageComplete >= 1.0) {
        //     startRed = endRed;
        //     startGreen = endGreen;
        //     startBlue = endBlue;
        //     startWhite = endWhite;
        //     setRGBW(endRed, endGreen, endBlue, endWhite);
        //     return;

        // } else {
        //     currRed = startRedTemp + (diffRed * precentageComplete);
        //     currGreen = startGreenTemp + (diffGreen * precentageComplete);
        //     currBlue = startBlueTemp + (diffBlue * precentageComplete);
        //     currWhite = startWhiteTemp + (diffWhite * precentageComplete);

        //     startRed = currRed;
        //     startGreen = currGreen;
        //     startBlue = currBlue;
        //     startWhite = currWhite;
        //     setRGBW();
        //     return;
        // }
    }

    void setRGBW(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
        ledcWrite(redPin, red);
        ledcWrite(greenPin, green);
        ledcWrite(bluePin, blue);
        ledcWrite(whitePin, white);

        endRed = red;
        endGreen = green;
        endBlue = blue;
        endWhite = white;

        // v tem primeru ugasne glavno stikalo
        if (red == 0 && green == 0 && blue == 0 && white == 0) {
            digitalWrite(RGBW_ON_OFF_PIN, 1);
        } else {
            digitalWrite(RGBW_ON_OFF_PIN, 0);
        }
    }

    void setRGBW() {
        ledcWrite(redPin, currRed);
        ledcWrite(greenPin, currGreen);
        ledcWrite(bluePin, currBlue);
        ledcWrite(whitePin, currWhite);

        // v tem primeru ugasne glavno stikalo
        if (currRed == 0 && currGreen == 0 && currBlue == 0 && currWhite == 0) {
            digitalWrite(RGBW_ON_OFF_PIN, 1);
        } else {
            digitalWrite(RGBW_ON_OFF_PIN, 0);
        }
    }
};
RgbwLight rgbwLight;

struct PingHandler {
    bool needPong = false;

    void loop() {
        if (needPong) {
            String tempTopic = String(mqttBaseTopic) + "/" + "pong";
            const char *pongPayload = "200";
            mqttClient.publish(tempTopic.c_str(), pongPayload);
            needPong = false;
        }
    }
};
PingHandler pingHandler;

struct BmpHandler {
    bool needTemp = false;
    unsigned long lastReading = 0;
    float minTemp = FLT_MAX;
    float maxTemp = FLT_MIN;

    void loop() {
        if (millis() - lastReading > READ_TEMP_INTERVAL) {
            readAndStoreMinMax();
            lastReading = millis();
        }
        if (needTemp) {
            String tempTopic = String(mqttBaseTopic) + "/" + "temp_data";

            float bmpTemperature = readAndStoreMinMax();
            String bpmPayload = String(String(bmpTemperature) + "," + String(maxTemp) + "," + String(minTemp));
            mqttClient.publish(tempTopic.c_str(), bpmPayload.c_str());
            needTemp = false;
        }
    }
    float readAndStoreMinMax() {
        float bmpTemperature = bmp.readTemperature();

        if (bmpTemperature > maxTemp) {
            maxTemp = bmpTemperature;
        } else if (bmpTemperature < minTemp) {
            minTemp = bmpTemperature;
        }

        return bmpTemperature;
    }
    void resetMinMax() {
        minTemp = FLT_MAX;
        maxTemp = FLT_MIN;
    }
};
BmpHandler bmpHandler;

uint8_t getRoletaIndexById(const String &id) {
    for (uint8_t i = 0; i < sizeof(roletas) / sizeof(roletas[0]); i++) {
        if (roletas[i].id == id) {
            return i;
        }
    }
    return 255;
}

uint8_t getLightBulbIndexById(const String &id) {
    for (uint8_t i = 0; i < sizeof(lightBulbs) / sizeof(lightBulbs[0]); i++) {
        if (lightBulbs[i].id == id) {
            return i;
        }
    }
    return 255;
}

void setRoletaCommand(const String rolId, uint8_t command) {
    uint8_t roletaIndex = getRoletaIndexById(rolId);
    if (roletaIndex == 255) {
        return;
    }
    if (command > 2) {
        command = 0;
    }

    if (roletas[roletaIndex].command != command) {
        roletas[roletaIndex].hasCommandChanged = true;
        roletas[roletaIndex].command = command;
        // nastavljanje spremenljivke za to da lahko automatično ugasnemo po določenem času
    }
}

void setupRoletas() {
    // WARNING: če želiš dodati spremeni tudi velikost roletas

    // id rolete se mora začeti z rcuid
    // saj se pri prepoznavanju na podlagi mqtt topic-a
    // ugotovi kateri roleti je bil ukaz poslan
    // INFO: rcuid --> roleta control unique id
    roletas[0].id = "rcuid01";
    roletas[0].powerPinUp = (uint8_t)PIN_POWER_UP_01;
    roletas[0].powerPinDown = (uint8_t)PIN_POWER_DOWN_01;
    // nastavi pin mode in zapiše zero state za relay in power pin
    pinMode(PIN_POWER_UP_01, OUTPUT);
    digitalWrite(PIN_POWER_UP_01, POWER_ZERO_STATE);
    pinMode(PIN_POWER_DOWN_01, OUTPUT);
    digitalWrite(PIN_POWER_DOWN_01, POWER_ZERO_STATE);

    roletas[0].command = 0;

    roletas[1].id = "rcuid02";
    roletas[1].powerPinUp = (uint8_t)PIN_POWER_UP_02;
    roletas[1].powerPinDown = (uint8_t)PIN_POWER_DOWN_02;
    // nastavi pin mode in zapiše zero state za relay in power pin
    pinMode(PIN_POWER_UP_02, OUTPUT);
    digitalWrite(PIN_POWER_UP_02, POWER_ZERO_STATE);
    pinMode(PIN_POWER_DOWN_02, OUTPUT);
    digitalWrite(PIN_POWER_DOWN_02, POWER_ZERO_STATE);
    roletas[1].command = 0;
}

void setupLightBulbs() {
    // lbuid --> light bulb unique id
    // id se mora začeti z lbuid
    // FIXME: lbuid01 is fake
    lightBulbs[0].id = "lbuid01";
    lightBulbs[0].powerPin = LIGHT_BULB_POWER_PIN_02;
    pinMode(LIGHT_BULB_POWER_PIN_01, OUTPUT);
    digitalWrite(LIGHT_BULB_POWER_PIN_02, POWER_ZERO_STATE);
    lightBulbs[0].command = 0;

    lightBulbs[1].id = "lbuid02";
    lightBulbs[1].powerPin = LIGHT_BULB_POWER_PIN_02;
    pinMode(LIGHT_BULB_POWER_PIN_02, OUTPUT);
    digitalWrite(LIGHT_BULB_POWER_PIN_02, POWER_ZERO_STATE);
    lightBulbs[1].command = 0;

    // FIXME: dodaj nekako drugače
    pinMode(RGBW_ON_OFF_PIN, OUTPUT);
}

void mqttSubscribeToTopics() {
    // subscribe na vse topics za rolete
    for (uint8_t i = 0; i < sizeof(roletas) / sizeof(roletas[0]); i++) {
        const String tempTopic = String(mqttBaseTopic) + "/" + String(roletas[i].id);
        mqttClient.subscribe(tempTopic.c_str());
        Serial.print("mqttSetup: subscribed to topic: ");
        Serial.println(tempTopic);
    }

    // subscribe na vse topics za žarnice
    for (uint8_t i = 0; i < sizeof(lightBulbs) / sizeof(lightBulbs[0]); i++) {
        const String tempTopic = String(mqttBaseTopic) + "/" + String(lightBulbs[i].id);
        mqttClient.subscribe(tempTopic.c_str());
        Serial.print("mqttSetup: subscribed to topic: ");
        Serial.println(tempTopic);
    }

    // subscribe na maintain topic
    String tempTopic = String(mqttBaseTopic) + "/" + String("maintain");
    mqttClient.subscribe(tempTopic.c_str());
    tempTopic = String(mqttBaseTopic) + "/" + String("ping");
    mqttClient.subscribe(tempTopic.c_str());
    tempTopic = String(mqttBaseTopic) + "/" + String("rgbw");
    mqttClient.subscribe(tempTopic.c_str());
    tempTopic = String(mqttBaseTopic) + "/" + String("temp_req");
    mqttClient.subscribe(tempTopic.c_str());
}

void setupWiFi() {
    delay(100);
    Serial.print("setupWiFi()");

    Serial.print("setupWiFI(): ");
    Serial.print("connecting to --> ");
    Serial.println(wifiSSID);

    WiFi.begin(wifiSSID, wifiPASS);

    long wifiStartAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        // v primeru, da pride do timeouta se ESP32 restarta
        if (millis() - wifiStartAttemptTime >= WIFI_CONNECT_TIMEOUT) {
            Serial.println("setupWiFi(): WiFi connection timeout reached. Restarting...");
            ESP.restart();
        }
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("setupWiFi: 200");
    Serial.print("setupWiFi: IPV4 --> ");
    Serial.println(WiFi.localIP());
    Serial.println();
}

void setupMqtt() {
    Serial.println("setupMqtt()");
    mqttClient.setServer(mqttBroker, mqttPort);
    mqttClient.setCallback(mqttCallback);

    long mqttStartAttemptTime = millis();
    while (!mqttClient.connected()) {
        String clientId = MQTT_CLIENT_ID;

        if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPass)) {
            Serial.println("setupMqtt(): 200");
        } else {
            Serial.print("setupMqtt(): state: ");
            Serial.println(mqttClient.state());
            delay(500);
            if (millis() - mqttStartAttemptTime >= MQTT_CONNECT_TIMEOUT) {
                Serial.print("setupMqtt(): MQTT connection timeout reached. Restarting...");
                ESP.restart();
            }
        }
    }
    mqttSubscribeToTopics();
}

void reconnectMqtt() {
    // Loop until we're reconnected
    long mqttStartAttemptTime = millis();
    unsigned long previousMillisForWiFi = 0;
    while (!mqttClient.connected()) {
        // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
        if ((WiFi.status() != WL_CONNECTED) && (millis() - previousMillisForWiFi >= WIFI_CHECK_INTERVAL)) {
            Serial.print(millis());
            Serial.println("Reconnecting to WiFi...");
            WiFi.disconnect();
            WiFi.reconnect();
            previousMillisForWiFi = millis();
        }
        Serial.println("reconnectMqtt()");
        // Attempt to connect
        // FIXME: se mora generirat samo
        String clientId = MQTT_CLIENT_ID;
        if (mqttClient.connect(clientId.c_str(), mqttUser, mqttPass)) {
            Serial.println("reconnectMqtt: 200");
            // Subscribe
            // FIXME: subscribe na topics ki jih mraš
            mqttSubscribeToTopics();
        } else {
            Serial.print("reconnectMqtt(): state: ");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void mqttCallback(char *topic, byte *message, unsigned int length) {
    // Serial.println("mqttCallback()");
    // če želiš izpis topic-a
    // Serial.print("mqttCallback(): topic: ");
    // Serial.println(topic);

    String topicTemp = String(topic);
    String messageTemp;

    for (unsigned int i = 0; i < length; i++) {
        messageTemp += (char)message[i];
    }

    // pridobi roleta id iz topic-a
    int rolIdPos = topicTemp.indexOf("rcuid");
    if (rolIdPos != -1) {
        String rolId = topicTemp.substring(rolIdPos);
        setRoletaCommand(rolId, (uint8_t)messageTemp.toInt());
        return;
    }

    // pridobi light bulb id iz topic-a
    int lightBulbIdPos = topicTemp.indexOf("lbuid");
    if (lightBulbIdPos != -1) {
        String lightBulbId = topicTemp.substring(lightBulbIdPos);
        setLightBulbCommand(lightBulbId, (uint8_t)messageTemp.toInt());
        return;
    }

    int rgbwIdPos = topicTemp.indexOf("rgbw");
    if (rgbwIdPos != -1) {
        rgbwLight.setRGBWHexWithDur(messageTemp);
        return;
    }

    // če topic vsebuje maintain
    int maintainPos = topicTemp.indexOf("maintain");
    if (maintainPos != -1) {
        if (messageTemp == "restart") {
            ESP.restart();
        }
        return;
    }
    // če topic ping
    int ping = topicTemp.indexOf("ping");
    if (ping != -1) {
        pingHandler.needPong = true;
        return;
    }

    int bmpPos = topicTemp.indexOf("temp_req");
    if (bmpPos != -1) {
        if (messageTemp == "reset") {
            bmpHandler.resetMinMax();
        } else {
            bmpHandler.needTemp = true;
            Serial.println("OK");
        }
    }
}

void roletasCommandLoop() {
    for (int i = 0; i < sizeof(roletas) / sizeof(roletas[0]); i++) {
        Roleta &currentRoleta = roletas[i];

        // preveri ali je potrebno command dati na 0 ker je že preteklo dovolj časa
        if (millis() - currentRoleta.powerStartedTime > MAX_POWER_TIME_MS) {
            currentRoleta.hasCommandChanged = true;
            // ugasne
            currentRoleta.command = 0;

            currentRoleta.powerStartedTime = millis();
        }

        if (currentRoleta.hasCommandChanged) {
            currentRoleta.hasCommandChanged = false;

            switch (currentRoleta.command) {
                case 0:
                    // NOTE: power goes to zero
                    digitalWrite(currentRoleta.powerPinUp, POWER_ZERO_STATE);
                    digitalWrite(currentRoleta.powerPinDown, POWER_ZERO_STATE);
                    Serial.println("OFF");
                    break;
                // up
                case 1:
                    currentRoleta.powerStartedTime = millis();
                    digitalWrite(currentRoleta.powerPinUp, !POWER_ZERO_STATE);
                    digitalWrite(currentRoleta.powerPinDown, POWER_ZERO_STATE);
                    Serial.println("UP");
                    break;
                case 2:
                    currentRoleta.powerStartedTime = millis();
                    digitalWrite(currentRoleta.powerPinUp, POWER_ZERO_STATE);
                    digitalWrite(currentRoleta.powerPinDown, !POWER_ZERO_STATE);
                    Serial.println("DOWN");
                    break;
            }
        }
    }
}

void lightBulbsCommandLoop() {
    for (int i = 0; i < sizeof(lightBulbs) / sizeof(lightBulbs[0]); i++) {
        LightBulb &currentLightBulb = lightBulbs[i];
        // Serial.println(currentLightBulb.command);
        switch (currentLightBulb.command) {
            case 0:
                digitalWrite(currentLightBulb.powerPin, POWER_ZERO_STATE);
                break;
            case 1:
                digitalWrite(currentLightBulb.powerPin, !POWER_ZERO_STATE);
                break;
        }
    }
}

void setup() {
    Serial.begin(115200);

    setupRoletas();
    setupLightBulbs();
    setupBmp();
    rgbwLight.setRgbwPins(RED_PIN, GREEN_PIN, BLUE_PIN, WHITE_PIN);
    setupWiFi();
    setupMqtt();
    delay(1000);
}

unsigned long previousMillisForWiFi = 0;
void loop() {
    // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
    if ((WiFi.status() != WL_CONNECTED) && (millis() - previousMillisForWiFi >= WIFI_CHECK_INTERVAL)) {
        Serial.print(millis());
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        previousMillisForWiFi = millis();
    }
    if (!mqttClient.connected()) {
        reconnectMqtt();  // includes wifi reconnection
    }

    mqttClient.loop();

    roletasCommandLoop();
    // lightBulbsCommandLoop();

    pingHandler.loop();
    rgbwLight.transitionLoop();
    bmpHandler.loop();
}
