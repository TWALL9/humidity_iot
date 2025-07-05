/**
 * humidity_iot
 * Copyright (C) 2025, TWALL9
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_GIGA) || defined(ARDUINO_OPTA)
#include <WiFi.h>
#elif defined(ARDUINO_PORTENTA_C33)
#include <WiFiC3.h>
#elif defined(ARDUINO_UNOR4_WIFI)
#include <WiFiS3.h>
#endif

#include "secrets.h"  // SECRET_SSID, SECRET_PASS, BROKER_HOST
#include <RTCZero.h>
#include <cstdint>
#include <utility>

static float getHumidity();
static float getTemperature();
static float getCellVoltage();
static float getHeartbeat();

const char ssid[] = SECRET_SSID;
const char pass[] = SECRET_PASS;

const char broker_host[] = BROKER_HOST;
const int port = 1883;

WiFiClient wifi_client;
MqttClient mqtt_client(wifi_client);

typedef float (*MeasurementFunc)();

constexpr std::pair<const char*, MeasurementFunc> measurements[] = {
  { "basement/humidity", getHumidity },
  { "basement/temperature", getTemperature },
  { "basement/cell_voltage", getCellVoltage },
  { "basement/heartbeat", getHeartbeat }
};

RTCZero rtc;

// TODO use official arduino DateTime types
// TODO add duration as difference between two datetimes.
struct DateTime {
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
};

DateTime current{
  .year = 25,
  .month = 7,
  .day = 1,
  .hour = 19,
  .minute = 29,
  .second = 0,
};

DateTime target = current;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(9600);
  while (!Serial) {
    ;
  }

  mqtt_client.setId("basement");

  reconnectToMqtt();

  // TODO get NTP time, populate DateTime with that info.
  rtc.begin();
  rtc.setTime(current.hour, current.minute, current.second);
  rtc.setDate(current.day, current.month, current.year);
  target.minute = current.minute + 1;
  rtc.setAlarmTime(target.hour, target.minute, target.second);

  // TODO check if this is the correct way to get a 1 hour sleep
  // rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(reconnectToMqtt);
}

void loop() {
  bool messages_sent = false;

  if (mqtt_client.connected()) {
    Serial.print("Sending messages...");
    for (const auto m : measurements) {
      mqtt_client.beginMessage(m.first);
      mqtt_client.print(m.second());
      mqtt_client.endMessage();
    }

    Serial.println("sent!");

    messages_sent = true;

    delay(500);
  } else {
    Serial.println("Reconnecting to MQTT");
    reconnectToMqtt();
  }

  if (messages_sent) {
    Serial.println("Entering sleep mode");
    calculateSleepDuration();
    wifi_client.stop();
    rtc.standbyMode();
  }
}

static void reconnectToMqtt() {
  Serial.print("Connecting to: ");
  Serial.print(ssid);
  Serial.print("...");

  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }

  Serial.println("...Connected!");

  Serial.print("Connecting to MQTT broker: ");
  Serial.println(broker_host);

  bool mqtt_connected = false;

  for (uint8_t i = 0; i < 5; i++) {
    mqtt_connected = mqtt_client.connect(broker_host, port);
    if (!mqtt_connected) {
      Serial.print("MQTT connection failed! Error code = ");
      Serial.println(mqtt_client.connectError());
    } else {
      Serial.println("MQTT connected");
      break;
    }
  }

  if (!mqtt_connected) {
    Serial.println("MQTT is probably dead. Giving up now");
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
    }
  }
}

void calculateSleepDuration() {
  // TODO get NTP time
  current.minute += 1;
  target.minute += 1;

  rtc.setAlarmTime(target.hour, target.minute, target.second);
}

static float getCellVoltage() {
  return 0.0;
}

static float getHumidity() {
  return 1.0;
}

static float getTemperature() {
  return 2.0;
}

static float getHeartbeat() {
  static uint32_t heartbeat = 0;
  heartbeat++;

  return (float)heartbeat;
}