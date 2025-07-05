# humidity_iot
dumb sensor to see how wet my basement is.

Really basic arduino sketch to connect to an MQTT server at `BROKER_HOST` on `SECRET_SSID` network. It reads the temperature and humidity, and publishes that information on MQTT. Also published on MQTT is the cell voltage of a connected battery, and an incrementing heartbeat.

After publishing the topics, the device will enter a low-power mode for a period. While it is in low-power mode, it will be disconnected from wifi and MQTT.

This sketch was created on the Arduino Nano IOT 33 board, and requires the following libraries:

- ArduinoMqttClient
- WiFININA
- RTCZero
