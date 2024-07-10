#ifndef AS5600_H
#define AS5600_H

#define I2C_MASTER_SCL_IO 22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_TIMEOUT_MS 1

#define AS5600_SENSOR_ADDR 0x36              /*!< Slave address of the AS5500 sensor */
#define AS5600_REG_DATA 0x0E                 /*!< Register to read data from */
#define AS5600_PULSES_PER_REVOLUTION 4096.0f /*!< Pulse per revolution */
#define AS5600_READING_MASK 0xFFF

uint16_t get_root_point();
void init_as5600();

#endif // define AS5600_H