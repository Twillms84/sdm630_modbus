#include "esphome/core/log.h"
#include "sdm630_modbus.h"

namespace esphome {
namespace sdm630_modbus {

static const char *TAG = "sdm630_modbus.component";

uint8_t registers[160]; // Speicher für die Register

// Setup-Methode
void SDM630Modbus::setup() {
    if (this->flow_control_pin_ != nullptr) {
        this->flow_control_pin_->setup();
    }
    modbus_state = MODBUS_STATE_IDLE;
    // Erstellen des Tasks für task_iv
    xTaskCreate(task_iv_static, "ModbusIV", 5000, this, 1, &ivHandle);
}


// Statische Methode für den FreeRTOS-Task
void SDM630Modbus::task_iv_static(void *parameter) {
    // Umwandlung des Parameters zurück zur Instanz der Klasse
    SDM630Modbus *modbus = static_cast<SDM630Modbus*>(parameter);

    // Endlosschleife für den Task
    while (true) {
        modbus->task_iv();
        vTaskDelay(pdMS_TO_TICKS(10)); // Verzögerung zur Reduzierung der CPU-Belastung
    }
}

// Loop-Methode
void SDM630Modbus::loop() {
}

// Implementierung der CRC-Funktionen
void SDM630Modbus::modbus_crc_init(uint16_t *crc) {
    *crc = 0xFFFF;
}

void SDM630Modbus::modbus_crc_add(uint8_t v, uint16_t *crc) {
    uint16_t c;
    c = *crc;
    c ^= (uint16_t)v; // XOR byte into least significant byte of crc

    for (uint8_t i = 8; i != 0; i--)  { 
        // Loop over each bit
        if ((c & 0x0001) != 0) {  // If the LSB is set
            c >>= 1; // Shift right and XOR 0xA001
            c ^= 0xA001;
        }
        else {  
            // Else LSB is not set
            c >>= 1; // Just shift right
        }
    }

    *crc = c;
}

// Verarbeitung empfangener MQTT-Nachrichten
void SDM630Modbus::mqtt_message_received(const std::string &topic, float value) {
    if (topic.find("sensor/SDMEmulator") != std::string::npos) {
        v_sum = value;
    
    }
}

void SDM630Modbus::write_array(const uint8_t *data, size_t length) {
    // Sende die Daten über UART
    for (size_t i = 0; i < length; ++i) {
        this->write(data[i]);
    }
}

void SDM630Modbus::task_iv() {

    ESP_LOGE(TAG, "Start Task IV");

	static uint16_t addr;
	static uint16_t wr_len;
	static uint32_t last_millis=millis();

    while(1) {
        ESP_LOGE(TAG, "WHILE SCHLEIFE GESTARTET");
        vTaskDelay(pdMS_TO_TICKS(1));
    
        if(last_millis!=millis()) {
		    if(modbus_time_cnt_1) {
			    modbus_time_cnt_1--;
			}
			last_millis=millis();
		}

        switch (modbus_state) {
        
            case MODBUS_STATE_CLEAR:
                ESP_LOGE(TAG, "MODBUS STATE Clear");
                while(this->available()) {
			        this->read();
			    }
			    modbus_state=MODBUS_STATE_IDLE; // next try to get data
			    break;

            case MODBUS_STATE_IDLE:
                if (this->available()) {
                    modbus_state = MODBUS_STATE_READ;
                    modbus_time_cnt_1 = MODBUS_RX_TIMEOUT;
                    modbus_in1_ptr = 0;
				    modbus_crc_init(&modbus_crc16_1);
				    ESP_LOGD(TAG, "CRC Initialized to: %04X\n", modbus_crc16_1);

                }
                break;
        
            case MODBUS_STATE_READ:
                if (!modbus_time_cnt_1) {
                ESP_LOGD(TAG,"TO-Error IV\n\r");
                modbus_state = MODBUS_STATE_IDLE;
                break;
                }

                if (this->available()) {
                    uint8_t c = this->read();
                    ESP_LOGD(TAG,"Received Byte: %02X", c);
                    modbus_time_cnt_1 = MODBUS_RX_TIMEOUT;

                    if (modbus_in1_ptr >= sizeof(modbus_in1_buf)) {
                        ESP_LOGD(TAG,"Buffer Overflow\n\r");
                        modbus_state = MODBUS_STATE_CLEAR;
                        break;
                    }

                    if (modbus_in1_ptr == 0 && c != 0x01) {
                        break;
                    }

                    if (modbus_in1_ptr == 1 && c != MODBUS_FC_READ_INPUT_REGS) {
                        ESP_LOGD(TAG,"Wrong Command\n\r");
                        modbus_in1_ptr = 0;
                        modbus_crc_init(&modbus_crc16_1);
                        ESP_LOGD(TAG, "CRC Initialized in ReAD STATE to: %04X\n", modbus_crc16_1);
                        break;
                    }

                    modbus_in1_buf[modbus_in1_ptr++] = c;
        
                    if(modbus_in1_ptr<7) {
		                modbus_crc_add(c,&modbus_crc16_1);
		               }
        
                    else if (modbus_in1_ptr == 8) {
                        uint16_t res_crc = modbus_in1_buf[6];
                        res_crc |= modbus_in1_buf[7] << 8;

                        ESP_LOGD(TAG,"Received CRC: %04X\n", res_crc);
                        ESP_LOGD(TAG,"Calculated CRC: %04X\n", modbus_crc16_1);

                        if (res_crc == modbus_crc16_1) {
                            ESP_LOGE(TAG, "Res = calculated cRC Value");
                            if (modbus_in1_buf[1] == MODBUS_FC_READ_REGS || modbus_in1_buf[1] == MODBUS_FC_READ_INPUT_REGS) {
                                addr = (modbus_in1_buf[2] << 8) + modbus_in1_buf[3];
                                wr_len = (modbus_in1_buf[4] << 8) + modbus_in1_buf[5];
                                modbus_state = MODBUS_STATE_WRITE;
                                modbus_time_cnt_1 = MODBUS_WRITE_DELAY;
                            } 
                            else {
                                ESP_LOGD(TAG,"FUNC-Error IV\n\r");
                                modbus_state = MODBUS_STATE_CLEAR;
                                break;
                            }
                        } 
                        else {
                            ESP_LOGD(TAG,"CRC-Error IV\n\r");
                            modbus_state = MODBUS_STATE_CLEAR;
                            break;
                        }
                    }
                }
             break;




            case MODBUS_STATE_WRITE:
                if (!modbus_time_cnt_1) {
        
                    // Schalte den Transmitter ein (Flow Control Pin)
                    if (this->flow_control_pin_ != nullptr) {
                        this->flow_control_pin_->digital_write(true);
                    }

                // Array für die zu sendenden Daten
                uint8_t data[256];
                size_t index = 0;

                // CRC-Initialisierung
                modbus_crc_init(&modbus_crc16_1);

        		modbus_uart1_send(0x01); // ID of SDM630
	        	modbus_uart1_send(modbus_in1_buf[1]); // functioncode
		        modbus_uart1_send(wr_len*2); // addr high

                while (wr_len) {
                    switch (addr) {
                        case 0x0c:
                            // Sende v_sum als 4 Bytes
				            modbus_uart1_send(((uint8_t*)&v_sum)[3]);
					        modbus_uart1_send(((uint8_t*)&v_sum)[2]);
					        modbus_uart1_send(((uint8_t*)&v_sum)[1]);
				        	modbus_uart1_send(((uint8_t*)&v_sum)[0]);
            	        	addr++;
				        	wr_len--;
                            break;
                        case 0x0e:
                            // Sende modbus_l2 als 4 Bytes
    			        	modbus_uart1_send(0);
				        	modbus_uart1_send(0);
				        	modbus_uart1_send(0);
			        		modbus_uart1_send(0);
        	        		addr++;
			        		wr_len--;
                          break;
                      case 0x10:
                            // Sende modbus_l3 als 4 Bytes
        		        	modbus_uart1_send(0);
					        modbus_uart1_send(0);
				        	modbus_uart1_send(0);
				        	modbus_uart1_send(0);
			        		addr++;
			        		wr_len--;
                            break;
                    default:
                        // Sende 0.0f als 4 Bytes
					    modbus_uart1_send(0); // high
					    modbus_uart1_send(0); // low
                        break;
                }
                addr++;
                wr_len--;
            }
            uint8_t crc_high = modbus_crc16_1 & 0x00FF;
            uint8_t crc_low = modbus_crc16_1 >> 8;
            this->write_array(crc_high); // crc high
		    this->write_array(crc_low; // crc low
            // Nächsten Zustand setzen
            modbus_state = MODBUS_STATE_DELAY_TO_IDLE;
            }
            break;

        case MODBUS_STATE_DELAY_TO_IDLE:
            if (millis() > modbus_timer_1) {
                if (this->flow_control_pin_ != nullptr) {
                    this->flow_control_pin_->digital_write(false); // TXE auf LOW
                }
                modbus_state = MODBUS_STATE_IDLE; // nächsten Versuch starten
            }
            break;
    }
    }
}

size_t SDM630Modbus::available() {
    // Stelle sicher, dass du die Anzahl der verfügbaren Bytes korrekt zurückgibst
    return uart::UARTDevice::available();
}

uint8_t SDM630Modbus::read() {
    // Lese ein Byte über UART
    return uart::UARTDevice::read();
}

void SDM630Modbus::modbus_uart1_send(uint8_t data)
{
    uint8_t byteArray[1] = { data };

	modbus_crc_add(data,&modbus_crc16_1);
	this->write_array(byteArray, sizeof(byteArray));
}

}
}
