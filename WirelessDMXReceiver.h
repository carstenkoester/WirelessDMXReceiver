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

enum wdmxUnit_t {
  ALL = 0,
  RED = 1,
  GREEN = 2,
  BLUE = 4,
  PURPLE = 5,
  CYAN = 7
};

class WirelessDMXReceiver
{
  public:
    WirelessDMXReceiver(int cePin, int csnPin, int statusLEDPin);

    void begin();

    unsigned int rxCount();
    unsigned int rxErrors();
    uint8_t dmxBuffer[DMX_BUFSIZE];
    bool debug;

  private:
    struct wdmxReceiveBuffer {
      uint8_t magic; // Always WDMX_MAGIC
      uint8_t payloadID;
      uint16_t highestChannelID; // Highest channel ID in the universe (not necessarily in this packet). Basically, highestChannelID + 1 = numChannels.
      uint8_t dmxData[WDMX_PAYLOAD_SIZE-WDMX_HEADER_SIZE];
    };

    bool _doScan(int unitID);
    void _dmxReceiveLoop();

    static void _startDMXReceiveThread(void*);
    uint64_t _getAddress(int unitID, int channelID);

    unsigned int _rxCount;
    unsigned int _rxErrors;
    int _statusLEDPin;
    RF24 _radio;
    TaskHandle_t _dmxReceiveTask;

    rf24_gpio_pin_t _cePin;
    rf24_gpio_pin_t _csnPin;
};

#endif