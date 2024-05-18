# Arduino ESP32 RF24 Wireless DMX receiver library

Arduino library that uses RF24 (<https://www.arduino.cc/reference/en/libraries/rf24/>) to
implement a Wireless DMX receiver compatible with several commercially available Wireless DMX transmitters.

Based on the analysis found at <https://juskihackery.wordpress.com/2021/01/31/how-the-cheap-wireless-dmx-boards-use-the-nrf24l01-protocol/>.

## TL;DR;

```
WirelessDMXReceiver receiver(RF24_PIN_CE, RF24_PIN_CSN, STATUS_LED_PIN);

void setup() { 
  receiver.begin();
}

void loop() {
  analogWrite(LED_PIN, receiver.getValue(DMX_ADDRESS));
}
```

On `begin()`, the library spawns a parallel thread that continuously receives and updates `receiver.dmxBuffer`. In other words, the `dmxBuffer` array always contains the most recent received data. The status LED will slowly be pulsing to indicate data being received.

The current implementation is somewhat linked to Arduino ESP32 (used with Adafruit Huzzah32 boards) and uses the ESP32's second core to run the receive thread, for maximum real-time data processing (but at the expense of portability). Future improvements to this library may include compatibility with other boards, and will likely decouple the status LED from the core module.