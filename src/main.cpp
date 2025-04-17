#include <WiFi.h>
#include <PubSubClient.h>
#include <DHTesp.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "model-parameters/model_metadata.h"
#include "edge-impulse-sdk/dsp/numpy_types.h"

const int DHT_PIN = 15;
const int LightSensor_Pin = 35;
const int MoistureSensor_Pin = 34;
const int MQ2_Pin = 33;

const float GAMMA = 0.7; // Gamma constant for LDR
const float RL10 = 50;   // LDR resistance at 10 lux

DHTesp dht;
const char *ssid = "Wokwi-GUEST";
const char *password = "";
const char *mqtt_server = "mqtt.eclipseprojects.io";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
float temp = 0;
float hum = 0;

String classifyPlantCondition(float temp, float hum, float lux, float moisture)
{
    // 1. log_light
    float log_light = log(lux + 1);

    // 2. temp/hum ratio
    float temp_hum_ratio = hum > 0 ? temp / hum : 0;

    // 3. ETI and sigmoid
    float ETI = (temp * log_light) / (hum + 1);
    float ETI_Sigmoid = 1.0 / (1.0 + exp(-ETI));

    // 4. VPD Calculation
    float SVP = 0.6108 * exp((17.27 * temp) / (temp + 237.3));
    float AVP = (hum / 100.0) * SVP;
    float VPD = SVP - AVP;

    // 5. Match training feature order
    float features[] = {
        temp,              // temperature
        hum,               // humidity
        log_light,         // log_light
        moisture,          // moisture_smooth
        temp_hum_ratio,    // temp_hum_ratio
        ETI_Sigmoid,       // ETI_Sigmoid
        VPD                // VPD_Column
    };

    signal_t signal;
    numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

    ei_impulse_result_t result;
    EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

    if (res != EI_IMPULSE_OK) {
        Serial.print("Edge Impulse Error: ");
        Serial.println(res);
        return "Error";
    }

    float maxValue = 0;
    String predictedLabel = "Unknown";
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > maxValue) {
            maxValue = result.classification[i].value;
            predictedLabel = result.classification[i].label;
        }
    }

    Serial.print("Predicted Condition: ");
    Serial.println(predictedLabel);
    return predictedLabel;
}

void setup_wifi()
{
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void callback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (int i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
}

void reconnect()
{
    while (!client.connected())
    {
        Serial.print("Attempting MQTT connection...");
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        if (client.connect(clientId.c_str()))
        {
            Serial.println("Connected");
            client.publish("/ThinkIOT/Publish", "Welcome");
            client.subscribe("/ThinkIOT/Subscribe");
        }
        else
        {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup()
{
    pinMode(2, OUTPUT);
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    dht.setup(DHT_PIN, DHTesp::DHT22);
}

void loop()
{
    if (!client.connected())
    {
        reconnect();
    }
    client.loop();

    unsigned long now = millis();
    // unsigned long now = millis();
    if (now - lastMsg > 1000) // 10 seconds interval
    {
        lastMsg = now;
        TempAndHumidity data = dht.getTempAndHumidity();

        // Read Sensor Data
        float temp = data.temperature;
        float hum = data.humidity;
        float moisture = analogRead(MoistureSensor_Pin) / 4095.0 * 100;

        int analogValue = analogRead(LightSensor_Pin);
        float voltage = analogValue / 4095.0 * 3.3;
        float resistance = (2000.0 * (3.3 - voltage)) / voltage;
        float lux = pow((RL10 * 1e3 / resistance), (1 / GAMMA));
        float gas = (analogRead(MQ2_Pin) / 4095.0) * 2474.03 - 444.29; // MQ2 sensor value

        // Classify Condition using TinyML
        String plantCondition = classifyPlantCondition(temp, hum, lux, moisture);

        // Publish Data to MQTT
        client.publish("/ThinkIOT/temp", String(temp).c_str());
        client.publish("/ThinkIOT/hum", String(hum).c_str());
        client.publish("/ThinkIOT/light", String(lux).c_str());
        client.publish("/ThinkIOT/moist", String(moisture).c_str());
        client.publish("/ThinkIOT/classification", plantCondition.c_str());

        // Print Data
        Serial.println("===== Features =====");
        Serial.println("Temp: " + String(temp));
        Serial.println("Hum: " + String(hum));
        // Serial.println("Log Light: " + String(log_light));
        Serial.println("Moisture: " + String(moisture));
        Serial.println("Lux: " + String(lux));
        Serial.println("Gas: " + String(gas));
        // Serial.println("Temp/Hum Ratio: " + String(temp_hum_ratio));
        // Serial.println("ETI Sigmoid: " + String(ETI_Sigmoid));
        // Serial.println("VPD: " + String(VPD));
    }
}