/*
  WirelessDMXReceiver.h - Library for receiving DMX using a Wireless module
  Carsten Koester, ckoester@cisco.com, May-2024.
  Released into the public domain.
*/

// Radio code from https://juskihackery.wordpress.com/2021/01/31/how-the-cheap-wireless-dmx-boards-use-the-nrf24l01-protocol/

// 540 transmissions per second
// 540/18 = 30 full universe transmissions per second
// Bit 8 set (header A0 instead of 80) when new DMX frame received by transmitter

#ifndef WirelessDMXReceiver_h
#define WirelessDMXReceiver_h

#include "Arduino.h"
#include <nRF24L01.h>
#include <RF24.h>
#include <RingBuf.h>

#define DMX_BUFSIZE                   512   // Total number of channels in a DMX universe
#define WDMX_PAYLOAD_SIZE              32   // Payload size in the NRF24L01 protocol
#define WDMX_HEADER_SIZE                4   // Header size in the NRF24L01 protocol
#define WDMX_MAGIC_1                 0x80   // Magic number expected in byte 0 of most packet
#define WDMX_MAGIC_2                 0xA0   // Magic number received in every 14th packet. Not sure what the significance of that is.

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

inline wdmxID_t operator++ (wdmxID_t &id) {
  id = static_cast<wdmxID_t>((static_cast<int>(id) + 1) % 8);
  return id;
}

class WirelessDMXReceiver
{
  public:
    struct wdmxReceiveBuffer {
      uint8_t magic;             // Always WDMX_MAGIC_1 or WDMX_MAGIC_2
      uint8_t payloadID;
      uint16_t highestChannelID; // Highest channel ID in the universe (not necessarily in this packet). Basically, highestChannelID + 1 = numChannels.
      uint8_t dmxData[WDMX_PAYLOAD_SIZE-WDMX_HEADER_SIZE];
    };
    
    WirelessDMXReceiver(int cePin, int csnPin, int statusLEDPin);

    void begin(wdmxID_t ID=AUTO);
    void begin(wdmxID_t ID, std::function<void()> scanCallback);

    uint8_t getValue(unsigned int address) const { return (dmxBuffer[address-1]); };
    void getValues(unsigned int startAddress, unsigned int length, void* buffer) const { memcpy(buffer, &dmxBuffer[startAddress-1], length); };
    wdmxID_t getId() const { return (_ID); };
    unsigned int getChannel() const { return (_channel); };
    bool isLocked() const { return (_locked); };
    unsigned int rxCount() const { return (_rxCount); };
    unsigned int rxInvalid() const { return (_rxInvalid); };
    unsigned int rxOverruns() const { return (_rxOverruns); };
    unsigned int rxSeqErrors() const { return (_rxSeqErrors); };

    uint8_t dmxBuffer[DMX_BUFSIZE];
    bool debug;
    bool capture;

    // FIXME: Need to figure out what to do with the "packet capture" ability long-term. It is useful but we may not want to sacrifice
    // memory for 2k packets when we're not using it. Perhaps it should be heap allocated, with configurable size, and on demand.
    RingBuf<wdmxReceiveBuffer, 2048> debugBuffer;

  private:
    bool _scanChannel();
    void _scanNext();
    void _dmxReceiveLoop();

    static void _startDMXReceiveThread(void*);
    uint64_t _getAddress(unsigned int channel, wdmxID_t ID);

    wdmxID_t _configID;
    wdmxID_t _ID;
    unsigned int _channel;
    bool _locked;

    unsigned int _rxCount = 0;     // Number of frames received
    unsigned int _rxInvalid = 0;   // Number of frames with invalid header
    unsigned int _rxOverruns = 0;  // Number of times RF24 returned FifoFull when we were processing a frame 
    unsigned int _rxSeqErrors = 0; // Number of times we detected a gap in sequence numbers
    int _statusLEDPin;
    RF24 _radio;
    TaskHandle_t _dmxReceiveTask;
};

#endif