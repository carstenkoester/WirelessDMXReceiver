/*
  WirelessDMXReceiver.cpp - Library for receiving DMX using a Wireless module
  Carsten Koester, ckoester@cisco.com, May-2024.
  Released into the public domain.
*/

#include "WirelessDMXReceiver.h"
#include <esp_task_wdt.h>

/*
 * Helper function to convert a (Unit ID, Channel ID) tuple into the
 * RF24 header structure that is used for this tuple.
 *
 * Ref https://juskihackery.wordpress.com/2021/01/31/how-the-cheap-wireless-dmx-boards-use-the-nrf24l01-protocol/
 * for a description of these values.
 *
 */
inline uint64_t WirelessDMXReceiver::_getAddress(unsigned int channel, wdmxID_t ID)
{
  union wdmxAddress {
    struct {
      uint8_t channel;      // Channel ID
      uint8_t ID;           // Unit ID
      uint8_t notChannel;   // Bitwise complement (bitwise NOT) of the Channel ID
      uint8_t notID;        // Bitwise complement (bitwise NOT) of the unit ID
      uint8_t sum;          // Algebraic sum of Unit ID and Channel ID
    } structured;
    uint64_t uint64;
  } address;

  address.structured.channel = channel;
  address.structured.ID = ID;
  address.structured.notChannel = ~channel;
  address.structured.notID = ~ID;
  address.structured.sum = channel + ID;

  return address.uint64;
}

/*
 * Given a unit ID and a channel, probe a single channel.
 *
 * Waits up to 10ms for data, and returns false if we encountered a timeout or read anything other than DMX data.
 * Return true if we did read DMX data.
 */
bool WirelessDMXReceiver::_scanChannel()
{
  wdmxReceiveBuffer rxBuf;

  if ((_statusLEDPin != 0) && (_channel % 16 == 0)) {
    digitalWrite(_statusLEDPin, !digitalRead(_statusLEDPin)); // Blink status LED while scanning - this will flash quickly
  }

  _radio.flush_rx();
  _radio.openReadingPipe(0, _getAddress(_channel, _ID));
  _radio.startListening();
  _radio.setChannel(_channel);
  if (debug) {
    Serial.printf("SCAN: Channel %d (%d), unit ID %d, address %llx\n", _radio.getChannel(), _channel, _ID, _getAddress(_channel, _ID));
  }

  unsigned long started_waiting_at = micros(); // timeout setup
  while (!_radio.available()) {                   // While nothing is received
    if (micros() - started_waiting_at > 10000) {  // If waited longer than 10ms, indicate timeout and exit while loop
       return(false);
    }
  }

  _radio.read(&rxBuf, sizeof(rxBuf));
  if ((rxBuf.magic == WDMX_MAGIC_1) || (rxBuf.magic == WDMX_MAGIC_2)) {
    if (debug) {
      Serial.printf("SCAN: Found a transmitter on channel %d, unit ID %d\n", _channel, _ID);
    }
    return(true);
  }

  // If we got here, we found *something* but it wasn't valid Wireless DMX data
  Serial.printf("SCAN: Found invalid data on channel %d, unit ID %d\n", _channel, _ID);
  return(false);
}

void WirelessDMXReceiver::_scanNext()
{
  _locked = _scanChannel();
  if (_locked) {
    return;
  }

  _channel++;
  if (_channel > 126) {
    _channel = 0;
    if (_configID == AUTO) {
      if (_ID < WHITE) {
        ++_ID;
      } else {
        _ID = RED;
      }
    }
  }
  return;
}

void WirelessDMXReceiver::_dmxReceiveLoop()
{
  wdmxReceiveBuffer rxBuf;
  uint8_t prevPayloadID = 0;
  bool firstFrame = true;

  while (true) {
    if (_radio.rxFifoFull()) {
      _rxOverruns++;
    }

    if (_radio.available())
    {
      /*
        * Read DMX values from radio.
        */
      _radio.read(&rxBuf, sizeof(rxBuf));

#ifdef WDMX_CAPTURE
      if (_capture) {
        _captureBuffer.pushOverwrite(rxBuf);
      }
#endif

      if ((rxBuf.magic != WDMX_MAGIC_1) && (rxBuf.magic != WDMX_MAGIC_2)) {
        // Received frame with unexpected magic number. Ignore.
        _rxInvalid++;
        continue;
      }

      if (firstFrame) {
        firstFrame = false;
      } else {
        if (((rxBuf.payloadID > 0) && (rxBuf.payloadID != prevPayloadID +1 )) || ((rxBuf.payloadID == 0) && prevPayloadID != rxBuf.highestChannelID/sizeof(rxBuf.dmxData))) {
          // Received a frame with gap in sequence number. We'll process it but count the error.
          _rxSeqErrors++;
        }
      }

      _rxCount++;
      prevPayloadID = rxBuf.payloadID;
      esp_task_wdt_reset();

      int dmxChanStart = rxBuf.payloadID* sizeof(rxBuf.dmxData);

      // put payload into dmx buffer. If data goes beyond 512 channels, wrap over.
      memcpy(&dmxBuffer[dmxChanStart], &rxBuf.dmxData, min(sizeof(rxBuf.dmxData), sizeof(dmxBuffer)-dmxChanStart));
      if (dmxChanStart+sizeof(rxBuf.dmxData) > sizeof(dmxBuffer)) {
        memcpy(&dmxBuffer, &rxBuf.dmxData[sizeof(dmxBuffer)-dmxChanStart], dmxChanStart+sizeof(rxBuf.dmxData)-sizeof(dmxBuffer));
      }

      // Pulse status LED when we're receiving
      if (_statusLEDPin != 0) {
        if ((_rxCount / 1024) % 2) {
          analogWrite(_statusLEDPin, (_rxCount % 1024)/4);
        } else {
          analogWrite(_statusLEDPin, 255-((_rxCount % 1024)/4));
        }
      }
    }
  }
}

void WirelessDMXReceiver::_startDMXReceiveThread(void* _this)
{
  ((WirelessDMXReceiver*)_this)->_dmxReceiveLoop();
}

WirelessDMXReceiver::WirelessDMXReceiver(int cePin, int csnPin, int statusLEDPin)
  : _radio(cePin, csnPin)
{
  _statusLEDPin = statusLEDPin;
}

void WirelessDMXReceiver::begin(wdmxID_t ID)
{
  begin(ID, nullptr);
}

void WirelessDMXReceiver::begin(wdmxID_t ID, std::function<void()> scanCallback)
{ 
  if (!_radio.begin()){
    if (debug) {
      Serial.println("ERROR: failed to start radio");
    }
  }

  _radio.setDataRate(RF24_250KBPS);
  _radio.setCRCLength(RF24_CRC_16);
  _radio.setPALevel(RF24_PA_LOW);
  _radio.setAutoAck(false);
  _radio.setPayloadSize(WDMX_PAYLOAD_SIZE);

  if (debug) {
    _radio.printPrettyDetails();
  }

  // Initial configuration to begin scanning
  _configID = ID;
  _channel = 0;
  _locked = false;
  if (_configID == AUTO) {
    _ID = RED;
  } else {
    _ID = _configID;
  }

  // Scan for receiver. If we were given a callback function, invoke the callback function between scan attempts.
  while(!_locked) {
    _scanNext();
    if (scanCallback) {
      scanCallback();
    }
  }

  // Clear the DMX buffer
  memset(&dmxBuffer, 0x00, sizeof(dmxBuffer)); // Clear DMX buffer

  // Start the output task
  xTaskCreatePinnedToCore(
    _startDMXReceiveThread, /* Function to implement the task */
    "DMX Receive Thread",   /* Name of the task */
    10000,                  /* Stack size in words */
    this,                   /* Task input parameter */
    0,                      /* Priority of the task */
    &_dmxReceiveTask,       /* Task handle. */
    0                       /* Core where the task should run */
  );
  esp_task_wdt_add(_dmxReceiveTask);
}

void WirelessDMXReceiver::startCapture()
{
#ifdef WDMX_CAPTURE
  _capture = true;
#endif  
}

void WirelessDMXReceiver::stopCapture()
{
#ifdef WDMX_CAPTURE
  _capture = false;
#endif  
}

bool WirelessDMXReceiver::isCaptureBufferFull()
{
#ifdef WDMX_CAPTURE
  return _captureBuffer.isFull();
#else
  return(false);
#endif  
}

void WirelessDMXReceiver::printCapture()
{
#ifdef WDMX_CAPTURE
  wdmxReceiveBuffer buf;
  unsigned int i=0;
  while (_captureBuffer.pop(buf)) {
    i++;
    Serial.printf("Pkt %04d Magic %02x Payload %02x (%d) HighestChannel %04x (%d), Data ", i, buf.magic, buf.payloadID, buf.payloadID, buf.highestChannelID, buf.highestChannelID);
    for (int j = 0; j < sizeof(buf.dmxData); j++) {
      Serial.printf("%02x ", buf.dmxData[j]);
    }
    Serial.printf("\n");
  }
#endif
}