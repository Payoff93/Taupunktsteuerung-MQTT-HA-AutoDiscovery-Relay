#include <ESP8266WiFi.h>  // Wifi connection
#include <PubSubClient.h>  // MQTT client http://knolleary.net
#include <ArduinoJson.h>  // For MQTT: JSON generator https://arduinojson.org
#include <DNSServer.h>  // DNS Server for connected client
#include <ESP8266WebServer.h>  // HTTP Server by Ivan Grokhotkov
#include <WiFiManager.h>  // Wifi Access Point https://github.com/tzapu
#include <DHT.h>
#include <DHT_U.h>

// ---- Configuration ----
// Connectivity
const char*         kWifiSSID = "SSID";
const char*         kWifiPass = "PASS";
const char*         kMQTTServer = "MQTT-SERVER-IP";
const char*         kMQTTUser = "USER";
const char*         kMQTTPass = "PASS";
const int           kMQTTPort = 1883;
const size_t        kMQTTBufferSize = 600; // Don't set to low, otherwise mqtt-msgs may be ignored
// Device Info
const char*         kDeviceModel = "ESP8266";  // Hardware Model
const char*         kSWVersion = "1.0";  // Firmware Version
const char*         kManufacturer = "Lars";  // Manufacturer Name
const String        kDeviceName = "TaupunktSteuerung_Zuluft";  // Device Name
const String        kMQTTStatusTopic = "TaupunktSteuerungen/" + kDeviceName;  // MQTT Topic

// Switch goes between 5V and RELAY Pin for Shortcut

#define LED 16 // IO16 (GPIO16)
#define BLUELED 2 // IO2 (GPIO2)
#define RELAY 5 // IO5 (GPIO5)
#define DHT_PIN 15 // IO15 (GPIO15)
#define DHT_TYPE DHT22

// ---- Configuration END ----

#define PERIOD_MILLSEC_1000    1000
#define PERIOD_MILLSEC_500     500
#define PERIOD_MILLSEC_250     250

WiFiClient          _wifi_client;
PubSubClient        _mqtt_pub_sub(_wifi_client);
WiFiServer          _web_server(80);
DHT_Unified         _dht(DHT_PIN, DHT_TYPE);
int                 _mqtt_counter_conn = 0;
uint32_t            _time = 0;
bool                _init_system = true;
bool                _send_mqtt_data = false;
String              _mac_id;
String              _header;

// storage for sensor data
float latest_temp = 99;
float latest_hum = 99;
float latest_dew = 99;


void MQTTHomeAssistantDiscovery() {
    String discovery_topic;
    String payload;
    String str_payload;
    if (_mqtt_pub_sub.connected()) {
        Serial.println("Sending mqtt message for Home Assistant auto discovery feature");
        StaticJsonDocument<kMQTTBufferSize> payload;
        JsonObject device;
        JsonArray identifiers;

        // Define Entities here
        // ---- Temperature ----
        discovery_topic = "homeassistant/sensor/" + kMQTTStatusTopic + "_Temperatur" + "/config";

        payload["name"] = "Temperatur";
        payload["uniq_id"] = _mac_id + "_Temperatur";
        payload["stat_t"] = kMQTTStatusTopic;
        payload["dev_cla"] = "temperature";
        payload["val_tpl"] = "{{ value_json.Temperatur | is_defined }}";
        payload["unit_of_meas"] = "°C";
        device = payload.createNestedObject("device");
        device["name"] = kDeviceName;
        device["model"] = kDeviceModel;
        device["sw_version"] = kSWVersion;
        device["manufacturer"] = kManufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(_mac_id);
        // generate JSON from string
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, str_payload);
        // Send message to MQTT
        _mqtt_pub_sub.publish(discovery_topic.c_str(), str_payload.c_str());

        // Prepare storage for new discovery entities
        payload.clear();
        device.clear();
        identifiers.clear();
        str_payload.clear();

        // ---- Humidity ----
        discovery_topic = "homeassistant/sensor/" + kMQTTStatusTopic + "_Luftfeuchtigkeit" + "/config";

        payload["name"] = "Luftfeuchtigkeit";
        payload["uniq_id"] = _mac_id + "_Luftfeuchtigkeit";
        payload["stat_t"] = kMQTTStatusTopic;
        payload["dev_cla"] = "humidity";
        payload["val_tpl"] = "{{ value_json.Luftfeuchtigkeit | is_defined }}";
        payload["unit_of_meas"] = "%";
        device = payload.createNestedObject("device");
        device["name"] = kDeviceName;
        device["model"] = kDeviceModel;
        device["sw_version"] = kSWVersion;
        device["manufacturer"] = kManufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(_mac_id);
        // generate JSON from string
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, str_payload);
        // Send message to MQTT
        _mqtt_pub_sub.publish(discovery_topic.c_str(), str_payload.c_str());

        // Prepare storage for new discovery entities
        payload.clear();
        device.clear();
        identifiers.clear();
        str_payload.clear();

        // ---- Dew Point ----
        discovery_topic = "homeassistant/sensor/" + kMQTTStatusTopic + "_Taupunkt" + "/config";

        payload["name"] = "Taupunkt";
        payload["uniq_id"] = _mac_id + "_Taupunkt";
        payload["stat_t"] = kMQTTStatusTopic;
        payload["dev_cla"] = "temperature";
        payload["val_tpl"] = "{{ value_json.Taupunkt | is_defined }}";
        payload["unit_of_meas"] = "°C";
        device = payload.createNestedObject("device");
        device["name"] = kDeviceName;
        device["model"] = kDeviceModel;
        device["sw_version"] = kSWVersion;
        device["manufacturer"] = kManufacturer;
        identifiers = device.createNestedArray("identifiers");
        identifiers.add(_mac_id);
        // generate JSON from string
        serializeJsonPretty(payload, Serial);
        Serial.println(" ");
        serializeJson(payload, str_payload);
        // Send message to MQTT
        _mqtt_pub_sub.publish(discovery_topic.c_str(), str_payload.c_str());

        // Prepare storage for new discovery entities
        payload.clear();
        device.clear();
        identifiers.clear();
        str_payload.clear();
        // add other entities like above ...
    }
}


void MQTTReceiverCallback(char* topic, byte* inFrame, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String message_temp;

    // Generate message string
    for (unsigned int i = 0; i < length; i++) {
        Serial.print(static_cast<char>(inFrame[i]));
        message_temp += static_cast<char>(inFrame[i]);
    }
    Serial.println();

    if (String(topic) == String("homeassistant/status")) {
        if (message_temp == "online")
            MQTTHomeAssistantDiscovery();
    } else if (String(topic) == String(kDeviceName + "/zuluft_an")) {
      // Turn on the relay
      digitalWrite(RELAY, HIGH);
      Serial.println("Relay turned on");
    } else if (String(topic) == String(kDeviceName + "/zuluft_aus")) {
      // Turn off the relay
      digitalWrite(RELAY, LOW);
      Serial.println("Relay turned off");
    }
}

void MQTTReconnect() {
    // Loop until we're reconnected
    while (!_mqtt_pub_sub.connected()  && (_mqtt_counter_conn++ < 4)) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (_mqtt_pub_sub.connect(kDeviceName.c_str(), kMQTTUser, kMQTTPass)) {
            Serial.println("connected");
            // Subscribe
            _mqtt_pub_sub.subscribe("homeassistant/status");
            _mqtt_pub_sub.subscribe(String(kDeviceName + "/zuluft_an").c_str());
            _mqtt_pub_sub.subscribe(String(kDeviceName + "/zuluft_aus").c_str());
            delay(PERIOD_MILLSEC_250);
        } else {
            Serial.print("failed, rc=");
            Serial.print(_mqtt_pub_sub.state());
            Serial.println(" try again in 1 seconds");
            delay(PERIOD_MILLSEC_1000);
        }
    }
    _mqtt_counter_conn = 0;
}

/** \brief Connect to WiFi and generate the unique id from MAC */
void WiFiSetup() {
    int counter = 0;
    byte mac[6];

    delay(PERIOD_MILLSEC_250);

    // Connect to Wifi network
    Serial.print("Connecting to ");
    Serial.println(kWifiSSID);

    WiFi.begin(kWifiSSID, kWifiPass);

    WiFi.macAddress(mac);
    _mac_id =  String(mac[0], HEX) + String(mac[1], HEX)
     + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);

    Serial.print("Unique ID: ");
    Serial.println(_mac_id);

    while (WiFi.status() != WL_CONNECTED && counter++ < 8) {
        delay(PERIOD_MILLSEC_1000);
        Serial.print(".");
    }
    Serial.println("");

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi is connected");
        Serial.print("Optained IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi connection failed!");
    }
}

void HTTPHandler() {
    // Listen for incoming clients
    WiFiClient client = _web_server.available();
    // If a new client connects,
    if (client) {
        Serial.println("New HTTP connection");
        String currentLine = "";
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                Serial.write(c);
                _header += c;
                if (c == '\n') {
                    if (currentLine.length() == 0) {
                        // Handle the requested URLs
                        if (_header.indexOf("GET /5/on") >= 0) {
                            // Turn the relay on
                            digitalWrite(RELAY, HIGH);
                        } else if (_header.indexOf("GET /5/off") >= 0) {
                            // Turn the relay off
                            digitalWrite(RELAY, LOW);
                        }

                        // HTTP response
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-type:text/html");
                        client.println("Connection: close");
                        client.println();

                        // Display the HTML web page
                        client.println("<!DOCTYPE html><html>");
                        client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
                        client.println("<link rel=\"icon\" href=\"data:,\">");
                        client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
                        client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
                        client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
                        client.println(".button2 {background-color: #77878A;}</style></head>");

                        // Web Page Heading
                        client.println("<body><h1>Taupunktsteuerung Zuluft</h1>");
                        // Display the buttons based on the relay state
                        client.print("<p><a href=\"/5/on\"><button class=\"button ");
                        client.print(digitalRead(RELAY) == HIGH ? "button2" : "");
                        client.println("\">Ein</button></a></p>");
                        client.print("<p><a href=\"/5/off\"><button class=\"button ");
                        client.print(digitalRead(RELAY) == LOW ? "button2" : "");
                        client.println("\">Aus</button></a></p>");
                        client.println("</body></html>");

                        client.println();
                        break;
                    } else {
                        currentLine = "";
                    }
                } else if (c != '\r') {
                    currentLine += c;
                }
            }
        }
        _header = "";
        client.stop();
        Serial.println("Client disconnected.");
        Serial.println("");
    }
}


float calculateDewPoint(float temperature, float humidity) {
  // Constants for the Magnus formula
  const float A = 17.27;
  const float B = 237.7;

  // Calculate the intermediate value 'gamma'
  float gamma = ((A * temperature) / (B + temperature)) + log(humidity / 100.0);

  // Calculate the dew point in Celsius
  float dewPoint = (B * gamma) / (A - gamma);

  return dewPoint;
}

/** \brief Mandatory setup routine */
void setup() {
    // Input and output definitions
    Serial.begin(9600);
    delay(PERIOD_MILLSEC_500);

    Serial.println("");
    Serial.println("");
    Serial.println("----------------------------------------------");
    Serial.print("MODEL: ");
    Serial.println(kDeviceModel);
    Serial.print("DEVICE: ");
    Serial.println(kDeviceName);
    Serial.print("SW Rev: ");
    Serial.println(kSWVersion);
    Serial.println("----------------------------------------------");

    // Init network connection
    WiFiSetup();
    // Wifi Manager initialization and setup
    WiFiManager wifi_manager;
    // Define the name of the access point in case no wifi credentials are supplied
    wifi_manager.autoConnect((String(kDeviceName) + String("AP")).c_str());

    // MQTT initialization
    _mqtt_pub_sub.setServer(kMQTTServer, kMQTTPort);
    _mqtt_pub_sub.setCallback(MQTTReceiverCallback);
    // Very important, if buffer is on default value the mqtt message will not be send
    _mqtt_pub_sub.setBufferSize(kMQTTBufferSize);

    Serial.println("Connected.");

    _web_server.begin();

    _dht.begin();

    pinMode(RELAY, OUTPUT);
    // Turn off relay at the beginning
    digitalWrite(RELAY, LOW); 
}

/** \brief Mandatory loop routine */
void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        if (!_mqtt_pub_sub.connected()) {
            MQTTReconnect();
        } else {
            _mqtt_pub_sub.loop();
        }
    }

    if (_init_system) {
        delay(PERIOD_MILLSEC_1000);
        _init_system = false;
        Serial.println("Initializing system ...");
         // Send Discovery Data
        MQTTHomeAssistantDiscovery();
    }
    // Do stuff every 500ms ... like reading a sensor
    if (millis() - _time > PERIOD_MILLSEC_1000) {
        _time = millis();

        // Read sensor data
        sensors_event_t event;
        _dht.temperature().getEvent(&event);
        float sensor_temp = event.temperature;

        _dht.humidity().getEvent(&event);
        float sensor_hum = event.relative_humidity;

        // Check for valid sensor values
        if (isnan(sensor_temp) || isnan(sensor_hum)) {
          Serial.println("Invalid DHT22 reading...");
          return;
        }
        // Filter out jumps in sensor readings (after first readout)
        if ( latest_temp != 99 && latest_hum != 99 && latest_dew != 99) {
          float temp_diff = abs(sensor_temp - latest_temp);
          float hum_diff = abs(sensor_hum - latest_hum);
          if (temp_diff > 5.0 || hum_diff > 10.0) {
            Serial.println("Jump in sensor readings detected!");
            return;
          }
        }
        latest_dew  = calculateDewPoint(sensor_temp, sensor_hum);
        latest_temp = sensor_temp;
        latest_hum  = sensor_hum;

        _send_mqtt_data = true;
    }

    // Update MQTT entities
    if (_send_mqtt_data) {
        StaticJsonDocument<200> payload;
        // Convert floats to string and round them
        char temperatureStr[6];
        dtostrf(latest_temp, 4, 2, temperatureStr);
        payload["Temperatur"] = temperatureStr;
        char humidityStr[6];
        dtostrf(latest_hum, 4, 2, humidityStr);
        payload["Luftfeuchtigkeit"] = humidityStr;
        char dewPointStr[6];
        dtostrf(latest_dew, 4, 2, dewPointStr);
        payload["Taupunkt"] = dewPointStr;
        payload["Relais"] = digitalRead(RELAY) == HIGH ? "zuluft_an" : "zuluft_aus";

        String str_payload;
        serializeJson(payload, str_payload);

        if (_mqtt_pub_sub.connected()) {
            _mqtt_pub_sub.publish(kMQTTStatusTopic.c_str(), str_payload.c_str());
            Serial.println("MQTT updated");
            _send_mqtt_data = false;
        }
    }


    HTTPHandler();
}