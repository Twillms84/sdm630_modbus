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
        ESP_LOGI(TAG, "Deleting existing task");
        vTaskDelete(ivHandle);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); // Verzögerung hinzufügen

    ESP_LOGI(TAG, "Creating Modbus IV task");
    if (xTaskCreate(task_iv_static, "ModbusIV", 5000, this, 1, &ivHandle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create Modbus IV task");
    }
    xTaskCreate(task_iv_static, "ModbusIV", 5000, this, 1, &ivHandle);
}

void SDM630Modbus::init() {
    ESP_LOGI(TAG, "Initialization called");
    this->modbus_reset();  // Füge den Reset-Aufruf hinzu
}

void SDM630Modbus::modbus_reset() {
	// reset the Modbus pointer and set to read
	modbus_state=MODBUS_STATE_IDLE;
	modbus_in_buffer_ptr=0;
	modbus_crc_init(&modbus_crc16);
}

void SDM630Modbus::modbus_rest(void)
{
	modbus_rest_delay=((50000)/9600)+2; // 5 Chars 10 Bits for ms (1000/s)+ 2ms
	modbus_state=MODBUS_STATE_REST;
	modbus_rest_time=millis()+modbus_rest_delay;
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
                }
                break;
        
            case MODBUS_STATE_READ:
					if(!modbus_time_cnt_1)
					{
						// timeout for read
						ESP_LOGD(TAG, "TO-Error IV\n\r");
						modbus_state=MODBUS_STATE_IDLE; // next try to get data
						break;
					}

					if(this->available())
					{
						uint8_t c=this->read();

						ESP_LOGD(TAG, "Aus Task IV %02x ",c);
						// restart timeout
						modbus_time_cnt_1=MODBUS_RX_TIMEOUT;

						// data in buffer
						if(modbus_in1_ptr>=sizeof(modbus_in1_buf))
						{
							// buffer overflow
							// ignore data
							ESP_LOGD(TAG, "OV-Error IV\n\r");
							modbus_state=MODBUS_STATE_IDLE; // next try to get data
							break;
						}

						if(!modbus_in1_ptr && c!=0x01) // 0x01 = ID
						{
							// ignore wrong chars at start of request
							break;
						}

						if(modbus_in1_ptr==1 && c!=MODBUS_FC_READ_INPUT_REGS) // 
						{
							// ignore wrong chars at start of request
							ESP_LOGD(TAG, "Wrong Command-Error IV %02X\n\r",c);
							modbus_in1_ptr=0;
							modbus_crc_init(&modbus_crc16_1);
							break;
						}

						modbus_in1_buf[modbus_in1_ptr++]=c;
						if(modbus_in1_ptr<7)
						{
							modbus_crc_add(c,&modbus_crc16_1);
						}
						else if(modbus_in1_ptr==8)
						{
							// have received data, check CRC
							uint16_t res_crc= modbus_in1_buf[6];
							res_crc|=modbus_in1_buf[7]<<8;
							if(res_crc==modbus_crc16_1)
							{
								if(modbus_in1_buf[1]==MODBUS_FC_READ_REGS || modbus_in1_buf[1]==MODBUS_FC_READ_INPUT_REGS)
								{
									// result crc matching
									addr=(modbus_in1_buf[2]<<8)+modbus_in1_buf[3];
									wr_len=(modbus_in1_buf[4]<<8)+modbus_in1_buf[5];
									modbus_state=MODBUS_STATE_WRITE; // delay next read
									modbus_time_cnt_1=MODBUS_WRITE_DELAY;
								}
								else
								{
									ESP_LOGD(TAG, "FUNC-Error IV\n\r");
									modbus_state=MODBUS_STATE_CLEAR; // next read
									break;
								}
							}
							else
							{
								// crc-error
								ESP_LOGD(TAG,"CRC-Error IV\n\r");
								modbus_state=MODBUS_STATE_CLEAR; // next read
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
                
    			modbus_timer_1=millis()+((10000*((wr_len*2)+5))/9600)+MODBUS_READ_DELAY;

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
            this->write(modbus_crc16_1 & 0x00FF); // crc high
		    this->write(modbus_crc16_1>>8); // crc low
            // Nächsten Zustand setzen
            modbus_state = MODBUS_STATE_DELAY_TO_IDLE;
            }
            break;

        case MODBUS_STATE_DELAY_TO_IDLE:
            if(millis()>modbus_timer_1)
			    {
				   // Schalte den Transmitter ein (Flow Control Pin)
                    if (this->flow_control_pin_ != nullptr) {
                        this->flow_control_pin_->digital_write(false);
                    }
					modbus_state=MODBUS_STATE_IDLE; // next try to get data
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
	modbus_crc_add(data,&modbus_crc16_1);
	this->write(data);
}

}
}
