/*
  WirelessDMXReceiver.h - Library for receiving DMX using a Wireless module
  Carsten Koester, ckoester@cisco.com, May-2024.
  Released into the public domain.
*/

// Radio code from https://juskihackery.wordpress.com/2021/01/31/how-the-cheap-wireless-dmx-boards-use-the-nrf24l01-protocol/

#ifndef WirelessDMXReceiver_h
#define WirelessDMXReceiver_h

#include "Arduino.h"

#include <nRF24L01.h>
#include <RF24.h>

#define DMX_BUFSIZE                   512   // Total number of channels in a DMX universe
#define WDMX_PAYLOAD_SIZE              32   // Payload size in the NRF24L01 protocol
#define WDMX_HEADER_SIZE                4   // Header size in the NRF24L01 protocol
#define WDMX_MAGIC                    128   // Magic number expected in byte 0 of every receive packet

enum wdmxID_t {                             // Unit IDs (aka ID LED Codes, or Channel Groups, depending on manufacturer)
  AUTO = 0,
  RED = 1,
  GREEN = 2,
  YELLOW = 3,
  BLUE = 4,
  MANGENTA = 5,
  CYAN = 6,
  WHITE = 7
};

class WirelessDMXReceiver
{
  public:
    WirelessDMXReceiver(int cePin, int csnPin, int statusLEDPin);

    void begin(wdmxID_t ID=AUTO);

    uint8_t getValue(unsigned int address) const { return (dmxBuffer[address-1]); };
    void getValues(unsigned int startAddress, unsigned int length, uint8_t* buffer) const { memcpy(buffer, &dmxBuffer[startAddress-1], length); };
    unsigned int rxCount() const { return (_rxCount); };
    unsigned int rxErrors() const { return (_rxErrors); };

    uint8_t dmxBuffer[DMX_BUFSIZE];
    bool debug;

  private:
    struct wdmxReceiveBuffer {
      uint8_t magic; // Always WDMX_MAGIC
      uint8_t payloadID;
      uint16_t highestChannelID; // Highest channel ID in the universe (not necessarily in this packet). Basically, highestChannelID + 1 = numChannels.
      uint8_t dmxData[WDMX_PAYLOAD_SIZE-WDMX_HEADER_SIZE];
    };

    bool _scanChannel(unsigned int channel, wdmxID_t ID);
    bool _scanID(wdmxID_t ID);
    bool _scanAllIDs();
    void _dmxReceiveLoop();

    static void _startDMXReceiveThread(void*);
    uint64_t _getAddress(unsigned int channel, wdmxID_t ID);

    wdmxID_t _ID;
    unsigned int _channel;

    unsigned int _rxCount;
    unsigned int _rxErrors;
    int _statusLEDPin;
    RF24 _radio;
    TaskHandle_t _dmxReceiveTask;
};

#endif