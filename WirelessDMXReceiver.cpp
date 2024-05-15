/*
  WirelessDMXReceiver.cpp - Library for receiving DMX using a Wireless module
  Carsten Koester, ckoester@cisco.com, May-2024.
  Released into the public domain.
*/

#include "Arduino.h"
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
uint64_t WirelessDMXReceiver::_getAddress(int unitID, int channelID) {
  union wdmxAddress {
    struct {
      uint8_t channel;      // Channel ID
      uint8_t unitID;       // Unit ID
      uint8_t notChannel;   // Bitwise complement (bitwise NOT) of the Channel ID
      uint8_t notUnitID;    // Bitwise complement (bitwise NOT) of the unit ID
      uint8_t sum;          // Algebraic sum of Unit ID and Channel ID
    } structured;
    uint64_t uint64;
  } address;

  address.structured.channel = channelID;
  address.structured.unitID = unitID;
  address.structured.notChannel = ~channelID;
  address.structured.notUnitID = ~unitID;
  address.structured.sum = channelID + unitID;

  return address.uint64;
}

/*
 * Given a Unit ID, probe all channels to see if we're receiving data for this unit ID
 */
bool WirelessDMXReceiver::_doScan(int unitID) {
  wdmxReceiveBuffer rxBuf;

  for (int rfCH = 0; rfCH < 126; rfCH++) {  
    delay(1);
    if (rfCH % 16 == 0) {
      digitalWrite(_statusLEDPin, !digitalRead(_statusLEDPin)); // Blink status LED while scanning - this will flash quickly
    }

    _radio.flush_rx();
    _radio.openReadingPipe(0, _getAddress(unitID, rfCH));
    _radio.startListening();
    _radio.setChannel(rfCH);
    if (debug) {
      Serial.printf("Trying channel %d (%d), unit ID %d, address %llx\n", _radio.getChannel(), rfCH, unitID, _getAddress(unitID, rfCH));
    }

    unsigned long started_waiting_at = micros(); // timeout setup
    bool timeout = false; 
    while (!_radio.available()) {                   // While nothing is received
      if (micros() - started_waiting_at > 10000) {  // If waited longer than 10ms, indicate timeout and exit while loop
         timeout = true;
         break;
      }     
    }

    if (!timeout) {  
      _radio.read(&rxBuf, sizeof(rxBuf));
      if (rxBuf.magic == WDMX_MAGIC) {
        if (debug) {
          Serial.printf("Found a transmitter on channel %d, unit ID %d\n", rfCH, unitID);
        }
        return(true);
      }
    }
  }
  return(false);
}

void WirelessDMXReceiver::_dmxReceiveLoop() {
  wdmxReceiveBuffer rxBuf;

  while (true) {
    while (_radio.available()) {
      /*
       * Read DMX values from radio.
       */

      _radio.read(&rxBuf, sizeof(rxBuf));

      if (rxBuf.magic != WDMX_MAGIC) {
        // Received packet with unexpected magic number. Ignore.
        _rxErrors++;
        continue;
      }

      _rxCount++;
      esp_task_wdt_reset();

      int dmxChanStart = rxBuf.payloadID* sizeof(rxBuf.dmxData);

      // put payload into dmx buffer. If data goes beyond 512 channels, wrap over.
      memcpy(&dmxBuffer[dmxChanStart], &rxBuf.dmxData, min(sizeof(rxBuf.dmxData), sizeof(dmxBuffer)-dmxChanStart));
      if (dmxChanStart+sizeof(rxBuf.dmxData) > sizeof(dmxBuffer)) {
        memcpy(&dmxBuffer, &rxBuf.dmxData[sizeof(dmxBuffer)-dmxChanStart], dmxChanStart+sizeof(rxBuf.dmxData)-sizeof(dmxBuffer));
      }

      // Pulse status LED when we're receiving
      if ((_rxCount / 1024) % 2) {
        analogWrite(_statusLEDPin, (_rxCount % 1024)/4);
      } else {
        analogWrite(_statusLEDPin, 255-((_rxCount % 1024)/4));
      }
    }
  }
}

void WirelessDMXReceiver::_startDMXReceiveThread(void* _this) {
  ((WirelessDMXReceiver*)_this)->_dmxReceiveLoop();
}

WirelessDMXReceiver::WirelessDMXReceiver(int cePin, int csnPin, int statusLEDPin)
  : _radio(cePin, csnPin)
{
  _statusLEDPin = statusLEDPin;
  _cePin = cePin;
  _csnPin = csnPin;
}

unsigned int WirelessDMXReceiver::rxCount() {
  return (_rxCount);
}

unsigned int WirelessDMXReceiver::rxErrors() {
  return (_rxErrors);
}

void WirelessDMXReceiver::begin()
{

  if (!_radio.begin()){
    if (debug) {
      Serial.println("ERROR: failed to start radio");
    }
  }

  _radio.setDataRate(RF24_250KBPS);
  _radio.setCRCLength(RF24_CRC_16);
  //_radio.setPALevel(RF24_PA_MAX);
  _radio.setPALevel(RF24_PA_LOW);
  _radio.setAutoAck(false);
  _radio.setPayloadSize(WDMX_PAYLOAD_SIZE);

  if (debug) {
    _radio.printPrettyDetails();
  }

  bool gotLock;

  for (;;) {
    for (int unitID = 1; unitID < 8; unitID++) {
      if (debug) {
        Serial.printf("Scanning for Unit ID %d\n", unitID);
      }
      gotLock = _doScan(unitID);
      if (gotLock) {
        break;
      }
    }
    if (gotLock) {
      break;
    }
  }

  if (debug) {
    Serial.printf("Got lock\n");
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