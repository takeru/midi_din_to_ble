#include <Arduino.h>
#include <M5Atom.h>

CRGB dispColor(uint8_t r, uint8_t g, uint8_t b) {
  return (CRGB)((r << 16) | (g << 8) | b);
}

// =====================================================================
//#define CS_USE_NIMBLE
#if defined(CS_USE_NIMBLE)
#  pragma message ("[CS] Forced NimBLE backend via CS_USE_NIMBLE")
#endif

#if defined(CONFIG_BT_NIMBLE_ENABLED)
#  pragma message ("[ESP32] NimBLE stack ENABLED")
#elif defined(CONFIG_BT_BLUEDROID_ENABLED)
#  pragma message ("[ESP32] Bluedroid stack ENABLED")
#else
#  pragma message ("[ESP32] No BLE stack macro detected")
#endif
#include <Control_Surface.h>
#include <MIDI_Interfaces/BluetoothMIDI_Interface.hpp>

// Instantiate a MIDI over BLE interface
BluetoothMIDI_Interface midi_ble;

// Instantiate a 5-pin DIN MIDI interface (on the TX and RX pins of Serial2)
HardwareSerial MidiSerial(2);
HardwareSerialMIDI_Interface midi_ser {MidiSerial};

// Instantiate the pipe to connect the two interfaces
BidirectionalMIDI_Pipe pipes;

void setupMidiSerial();

void setup() {
  // 本体初期化（UART有効, I2C無効, LED有効）
  M5.begin(true, false, true);
  M5.dis.drawpix(0, dispColor(0, 0, 0));
  Serial.begin(115200);

  setupMidiSerial();

  // Change the name of the BLE device (must be done before initializing it)
  midi_ble.setName("MIDI Adapter");
  
  // 6-> 7.5ms, 12 -> 15ms, 24 -> 30ms
  midi_ble.ble_settings.connection_interval.minimum =  6;
  midi_ble.ble_settings.connection_interval.maximum = 12;
  midi_ble.ble_settings.initiate_security = false;
  
  // Manually route MIDI input from the serial interface to the BLE interface,
  // and the MIDI input from the BLE interface to the serial interface
  midi_ser | pipes | midi_ble;
  // Initialize the MIDI interfaces (this will handle NimBLE initialization internally)
  MIDI_Interface::beginAll();
}

unsigned long lastUpdatedMillis = 0;

void loop() {
  // Continuously poll all interfaces and route the traffic between them
  MIDI_Interface::updateAll();

  bool isConnected = midi_ble.isConnected();
  if(lastUpdatedMillis + 100 < millis()) {
    M5.update();
    lastUpdatedMillis = millis();

    static int n = 0;
    n++;
    if (M5.Btn.isPressed()) {
      M5.dis.drawpix(0, dispColor(50, 50, 0));
    } else {
      switch(n % 20) {
        case 0:
          if(isConnected) {
            M5.dis.drawpix(0, dispColor(30, 30, 100));
          } else {
            M5.dis.drawpix(0, dispColor(50, 0, 0));
          }
          break;
        case  5:
        case 10:
        case 15:
          M5.dis.drawpix(0, dispColor(20, 20, 20));
          break;
        case  1:
        case  6:
        case 11:
        case 16:
          M5.dis.drawpix(0, dispColor(0, 0, 0));
          break;
      }
    }
  }
}

void setupMidiSerial() {
  unsigned long MIDI_BAUD_RATE = 31250;
  int MIDI_RX = 32;
  int MIDI_TX = 26;
  size_t MIDI_RX_BUFFER_SIZE = 1024;
  size_t MIDI_TX_BUFFER_SIZE = 1024;
  MidiSerial.setRxBufferSize(MIDI_RX_BUFFER_SIZE);
  MidiSerial.setTxBufferSize(MIDI_TX_BUFFER_SIZE);
  MidiSerial.begin(MIDI_BAUD_RATE, SERIAL_8N1, MIDI_RX, MIDI_TX);
}
