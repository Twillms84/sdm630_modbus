#include "esphome/core/log.h"
#include "sdm630_modbus.h"

namespace esphome {
namespace sdm630_modbus {

static const char *TAG = "sdm630_modbus.component";

// Setup-Methode
void SDM630Modbus::setup() {
    if (this->flow_control_pin_ != nullptr) {
        this->flow_control_pin_->setup();
        this->flow_control_pin_->digital_write(false); // TXE auf LOW
    }
    this->init();
    
    if (ivHandle != nullptr) {
        vTaskDelete(ivHandle);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); // Verzögerung hinzufügen
    xTaskCreate(task_iv_static, "ModbusIV", 5000, this, 1, &ivHandle);
}

void SDM630Modbus::loop() {
}

void SDM630Modbus::init() {
    this->modbus_reset();  // Füge den Reset-Aufruf hinzu
}

void SDM630Modbus::modbus_reset() {
	// reset the Modbus pointer and set to read
	modbus_state = MODBUS_STATE_IDLE;
	modbus_in_buffer_ptr = 0;
	modbus_crc_init(&modbus_crc16);
}

void SDM630Modbus::modbus_rest(void) {
	modbus_rest_delay = ((50000) / 9600) + 2; // 5 Chars 10 Bits for ms (1000/s)+ 2ms
	modbus_state = MODBUS_STATE_REST;
	modbus_rest_time = millis() + modbus_rest_delay;
}

// Statische Methode für den FreeRTOS-Task
void SDM630Modbus::task_iv_static(void *parameter) {
    SDM630Modbus *modbus = static_cast<SDM630Modbus*>(parameter);

    while (true) {
        modbus->task_iv();
    }
}

// Implementierung der CRC-Funktionen
void SDM630Modbus::modbus_crc_init(uint16_t *crc) {
    *crc = 0xFFFF;
}

void SDM630Modbus::modbus_crc_add(uint8_t v, uint16_t *crc) {
    uint16_t c = *crc;
    c ^= (uint16_t)v; // XOR byte into least significant byte of crc

    for (uint8_t i = 8; i != 0; i--) { 
        if ((c & 0x0001) != 0) {  // If the LSB is set
            c >>= 1; // Shift right and XOR 0xA001
            c ^= 0xA001;
        }
        else {  
            c >>= 1; // Just shift right
        }
    }

    *crc = c;
}

void SDM630Modbus::task_iv() {
    static uint16_t addr;
    static uint16_t wr_len;
    static uint32_t last_millis = millis();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1));
    
        if (last_millis != millis()) {
            if (modbus_time_cnt_1) {
                modbus_time_cnt_1--;
            }
            last_millis = millis();
        }

        switch (modbus_state) {
            case MODBUS_STATE_CLEAR:
                while (this->available()) {
                    this->read(); // Clear buffer by reading all data
                }
                modbus_state = MODBUS_STATE_IDLE;
                break;

            case MODBUS_STATE_IDLE:
                if (this->available()) {
                    modbus_state = MODBUS_STATE_READ;
                    modbus_time_cnt_1 = MODBUS_RX_TIMEOUT;
                    modbus_in1_ptr = 0;
                    modbus_crc_init(&modbus_crc16_1);
                }
                break;

            case MODBUS_STATE_READ:
                if (!modbus_time_cnt_1) {
                    ESP_LOGE(TAG, "Read timeout error");
                    modbus_state = MODBUS_STATE_IDLE;
                    break;
                }

                if (this->available()) {
                    uint8_t c = this->read();

                    modbus_time_cnt_1 = MODBUS_RX_TIMEOUT;  // Reset timeout

                    if (modbus_in1_ptr >= sizeof(modbus_in1_buf)) {
                        ESP_LOGE(TAG, "Buffer overflow error");
                        modbus_state = MODBUS_STATE_IDLE;
                        break;
                    }

                    if (!modbus_in1_ptr && c != 0x01) {  // 0x01 = ID
                        break;  // Ignore wrong chars at start of request
                    }

                    if (modbus_in1_ptr == 1 && c != MODBUS_FC_READ_INPUT_REGS) {
                        ESP_LOGE(TAG, "Wrong command error: %02X", c);
                        modbus_in1_ptr = 0;
                        modbus_crc_init(&modbus_crc16_1);
                        break;
                    }

                    modbus_in1_buf[modbus_in1_ptr++] = c;
                    if (modbus_in1_ptr < 7) {
                        modbus_crc_add(c, &modbus_crc16_1);
                    } else if (modbus_in1_ptr == 8) {
                        uint16_t res_crc = modbus_in1_buf[6] | (modbus_in1_buf[7] << 8);
                        if (res_crc == modbus_crc16_1) {
                            if (modbus_in1_buf[1] == MODBUS_FC_READ_REGS || modbus_in1_buf[1] == MODBUS_FC_READ_INPUT_REGS) {
                                addr = (modbus_in1_buf[2] << 8) + modbus_in1_buf[3];
                                wr_len = (modbus_in1_buf[4] << 8) + modbus_in1_buf[5];
                                modbus_state = MODBUS_STATE_WRITE;
                                modbus_time_cnt_1 = MODBUS_WRITE_DELAY;
                            } else {
                                ESP_LOGE(TAG, "Function code error");
                                modbus_state = MODBUS_STATE_CLEAR;
                            }
                        } else {
                            ESP_LOGE(TAG, "CRC error");
                            modbus_state = MODBUS_STATE_CLEAR;
                        }
                    }
                }
                break;

            case MODBUS_STATE_WRITE:
                if (!modbus_time_cnt_1) {
                    if (this->flow_control_pin_ != nullptr) {
                        this->flow_control_pin_->digital_write(true);
                    }

                    modbus_crc_init(&modbus_crc16_1);
                    modbus_uart1_send(0x01); // ID of SDM630
                    modbus_uart1_send(modbus_in1_buf[1]); // function code
                    modbus_uart1_send(wr_len * 2);

                    modbus_timer_1 = millis() + ((10000 * ((wr_len * 2) + 5)) / 9600) + MODBUS_READ_DELAY;

                    while (wr_len) {
                        switch (addr) {
                            case 0x0c:
                                modbus_uart1_send(((uint8_t*)&v_sum)[3]);
                                modbus_uart1_send(((uint8_t*)&v_sum)[2]);
                                modbus_uart1_send(((uint8_t*)&v_sum)[1]);
                                modbus_uart1_send(((uint8_t*)&v_sum)[0]);
                                addr++;
                                wr_len--;
                                break;
                            case 0x0e:
                                modbus_uart1_send(0);
                                modbus_uart1_send(0);
                                modbus_uart1_send(0);
                                modbus_uart1_send(0);
                                addr++;
                                wr_len--;
                                break;
                            case 0x10:
                                modbus_uart1_send(0);
                                modbus_uart1_send(0);
                                modbus_uart1_send(0);
                                modbus_uart1_send(0);
                                addr++;
                                wr_len--;
                                break;
                            default:
                                modbus_uart1_send(0);
                                modbus_uart1_send(0);
                                break;
                        }
                        addr++;
                        wr_len--;
                    }
                    this->write(modbus_crc16_1 & 0x00FF); // CRC high
                    this->write(modbus_crc16_1 >> 8);     // CRC low
                    modbus_state = MODBUS_STATE_DELAY_TO_IDLE;
                }
                break;

            case MODBUS_STATE_DELAY_TO_IDLE:
                if (millis() > modbus_timer_1) {
                    if (this->flow_control_pin_ != nullptr) {
                        this->flow_control_pin_->digital_write(false);
                    }
                    modbus_state = MODBUS_STATE_IDLE;
                }
                break;
        }
    }
}



size_t SDM630Modbus::available() {
    return uart::UARTDevice::available();
}

uint8_t SDM630Modbus::read() {
    return uart::UARTDevice::read();
}

void SDM630Modbus::modbus_uart1_send(uint8_t data) {
    modbus_crc_add(data, &modbus_crc16_1);
    this->write(data);
}


void SDM630Modbus::write_array(const uint8_t *data, size_t length) {
    // Nutze eine effizientere Methode zum Senden des Arrays
    // Hier setzen wir voraus, dass die UART-Schnittstelle eine Methode bietet,
    // um ein Array direkt zu senden (z.B. uart_write_bytes).
    uart::UARTDevice::write_array(data, length);

    // Berechne CRC für die gesamte Nachricht in einem Rutsch
    for (size_t i = 0; i < length; ++i) {
        modbus_crc_add(data[i], &modbus_crc16_1);
    }
}

void SDM630Modbus::mqtt_message_received(const std::string &topic, float value) {
    if (topic == "sensor/SDMEmulator") {
        ESP_LOGI(TAG, "MQTT Data Received:",value );
        if (v_sum != value) {  // Nur aktualisieren, wenn der Wert sich geändert hat.
            v_sum = value;
        }
    }
}

void SDM630Modbus::send_modbus_response(uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; ++i) {
        modbus_crc_add(data[i], &crc);
        this->write(data[i]);
    }

    this->write(crc & 0x00FF);  // CRC low byte
    this->write(crc >> 8);      // CRC high byte
}

size_t SDM630Modbus::prepare_response_data(uint8_t *buffer, uint16_t addr, uint16_t wr_len) {
    size_t length = 0;

    while (wr_len) {
        switch (addr) {
            case 0x0c:
                memcpy(&buffer[length], &v_sum, sizeof(v_sum));
                length += sizeof(v_sum);
                break;
            case 0x0e:
            case 0x10:
                memset(&buffer[length], 0, 4);
                length += 4;
                break;
            default:
                buffer[length++] = 0;
                buffer[length++] = 0;
                break;
        }
        addr++;
        wr_len--;
    }

    return length;
}

void SDM630Modbus::process_modbus_request() {
    static uint16_t addr;
    static uint16_t wr_len;
    static uint8_t response_buffer[256];  // Puffer für die Antwortdaten

    switch (modbus_state) {
        case MODBUS_STATE_CLEAR:
            while (this->available()) {
                this->read();  // Empfangspuffer leeren
            }
            modbus_state = MODBUS_STATE_IDLE;
            break;

        case MODBUS_STATE_IDLE:
            if (this->available()) {
                modbus_state = MODBUS_STATE_READ;
                modbus_time_cnt_1 = MODBUS_RX_TIMEOUT;
                modbus_in1_ptr = 0;
                modbus_crc_init(&modbus_crc16_1);
            }
            break;

        case MODBUS_STATE_READ:
            if (!modbus_time_cnt_1) {
                ESP_LOGE(TAG, "Read timeout error");
                modbus_state = MODBUS_STATE_IDLE;
                break;
            }

            if (this->available()) {
                uint8_t c = this->read();

                modbus_time_cnt_1 = MODBUS_RX_TIMEOUT;  // Timeout zurücksetzen

                if (modbus_in1_ptr >= sizeof(modbus_in1_buf)) {
                    ESP_LOGE(TAG, "Buffer overflow error");
                    modbus_state = MODBUS_STATE_IDLE;
                    break;
                }

                if (!modbus_in1_ptr && c != 0x01) {  // 0x01 = ID
                    break;  // Ignoriere falsche Zeichen am Anfang
                }

                if (modbus_in1_ptr == 1 && c != MODBUS_FC_READ_INPUT_REGS) {
                    ESP_LOGE(TAG, "Wrong command error: %02X", c);
                    modbus_in1_ptr = 0;
                    modbus_crc_init(&modbus_crc16_1);
                    break;
                }

                modbus_in1_buf[modbus_in1_ptr++] = c;
                if (modbus_in1_ptr < 7) {
                    modbus_crc_add(c, &modbus_crc16_1);
                } else if (modbus_in1_ptr == 8) {
                    uint16_t res_crc = modbus_in1_buf[6] | (modbus_in1_buf[7] << 8);
                    if (res_crc == modbus_crc16_1) {
                        if (modbus_in1_buf[1] == MODBUS_FC_READ_REGS || modbus_in1_buf[1] == MODBUS_FC_READ_INPUT_REGS) {
                            addr = (modbus_in1_buf[2] << 8) + modbus_in1_buf[3];
                            wr_len = (modbus_in1_buf[4] << 8) + modbus_in1_buf[5];
                            modbus_state = MODBUS_STATE_WRITE;
                            modbus_time_cnt_1 = MODBUS_WRITE_DELAY;
                        } else {
                            ESP_LOGE(TAG, "Function code error");
                            modbus_state = MODBUS_STATE_CLEAR;
                        }
                    } else {
                        ESP_LOGE(TAG, "CRC error");
                        modbus_state = MODBUS_STATE_CLEAR;
                    }
                }
            }
            break;

        case MODBUS_STATE_WRITE:
            if (!modbus_time_cnt_1) {
                if (this->flow_control_pin_ != nullptr) {
                    this->flow_control_pin_->digital_write(true);
                }

                // Bereite Antwortdaten vor
                size_t length = prepare_response_data(response_buffer + 3, addr, wr_len);

                // Füge Header-Daten hinzu
                response_buffer[0] = 0x01;  // ID
                response_buffer[1] = modbus_in1_buf[1];  // Funktionscode
                response_buffer[2] = length;

                // Sende Modbus-Antwort mit CRC
                send_modbus_response(response_buffer, length + 3);

                modbus_state = MODBUS_STATE_DELAY_TO_IDLE;
            }
            break;

        case MODBUS_STATE_DELAY_TO_IDLE:
            if (millis() > modbus_timer_1) {
                if (this->flow_control_pin_ != nullptr) {
                    this->flow_control_pin_->digital_write(false);
                }
                modbus_state = MODBUS_STATE_IDLE;
            }
            break;
    }
}

}
}
