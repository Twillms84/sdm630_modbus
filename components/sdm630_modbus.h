#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Modbus-Zustandsdefinitionen
#define MODBUS_STATE_IDLE			0
#define MODBUS_STATE_READ			1
#define MODBUS_STATE_REST   		2
#define MODBUS_STATE_WRITE_DELAY 	3
#define MODBUS_STATE_WRITE			4
#define MODBUS_STATE_DELAY_TO_IDLE	5
#define MODBUS_STATE_CLEAR			6


// Timeout- und Delay-Werte
#define MODBUS_RX_TIMEOUT 200          // Empfangs-Timeout
#define MODBUS_READ_DELAY 3         // Lese-Delay
#define MODBUS_WRITE_DELAY 3         // Schreib-Delay
#define MODBUS_FC_READ_INPUT_REGS 0x04  // Funktion zum Lesen der Eingangsregister
#define MODBUS_FC_READ_REGS			0x03




namespace esphome {
namespace sdm630_modbus {

// Hauptklasse für die SDM630 Modbus-Komponente
class SDM630Modbus : public uart::UARTDevice, public Component {
 public:
  // Methoden, die von der Basisklasse überschrieben werden
  void setup() override; // Initialisierung
  void loop() override;  // Periodische Aufgaben
  void modbus_reset();  // Methode zum Zurücksetzen
  void modbus_rest();
  // Methoden zum Verarbeiten von Anfragen und MQTT-Nachrichten
  void handleInverter();
  void task_iv();
  void init();

  // Fügt einen Wert in das Register ein
  void insert_into_register(uint8_t startAddress, float value);

  // Behandelt eingehende MQTT-Nachrichten
  void mqtt_message_received(const std::string &topic, float value);

  // Setter für den Flow-Control-Pin
  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }

  // Methode zum Schreiben eines Wertes in ein Register
  void write_register(uint8_t address, uint8_t value);

 protected:

  volatile uint8_t modbus_state;
  GPIOPin *flow_control_pin_{nullptr}; // Pin zur Steuerung des Datenflusses
  std::vector<uint8_t> rx_buffer; // Puffer für eingehende Daten

 private:

  uint32_t modbus_rest_delay=0;
  uint32_t modbus_rest_time=0;

  // Private Methoden und Member-Variablen
  static void task_iv_static(void *parameter); // Statische Methode für den Task
  TaskHandle_t ivHandle; // Handle für den FreeRTOS-Task

  // Schreibt ein Array von Daten in das UART
  void write_array(const uint8_t *data, size_t length);
  
  // Deklaration der CRC-Funktionen
  void modbus_crc_init(uint16_t *crc);
  void modbus_crc_add(uint8_t v, uint16_t *crc);
  void modbus_uart1_send(uint8_t data);

  // Gibt die Anzahl der verfügbaren Bytes zurück
  size_t available();

  // Liest ein Byte vom UART
  uint8_t read();
  
  // Modbus-Zustandsvariablen
  uint8_t modbus_time_cnt_1 = 0;
  uint8_t modbus_in1_ptr = 0;
  uint16_t modbus_crc16 = 0;
  uint16_t modbus_crc16_0 = 0;
  uint16_t modbus_crc16_1 = 0;
  uint8_t modbus_in1_buf[256];  // Puffergröße anpassen
  uint32_t modbus_timer_1 = 0;
  size_t wr_len = 0;
  uint32_t last_millis = 0;
  uint16_t modbus_in_buffer_ptr;

  
  // Globale Variablen
  float v_sum;
  float modbus_auto_offset;
  
};


}  // namespace sdm630_modbus
}  // namespace esphome
