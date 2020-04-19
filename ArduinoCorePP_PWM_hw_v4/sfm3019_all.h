#define SENSIRION_BIG_ENDIAN 0
#define SFM3019_I2C_ADDRESS 0x2E

#define SFM3019_CMD_START_CONTINUOUS_MEASUREMENT_O2 \
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS0
#define SFM3019_CMD_START_CONTINUOUS_MEASUREMENT_AIR \
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS1
#define SFM3019_CMD_START_CONTINUOUS_MEASUREMENT_AIR_O2_MIX \
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS_MIX_0

#define SFM3019_SOFT_RESET_TIME_US 2000
#define SENSIRION_I2C_CLOCK_PERIOD_USEC 10


#define STATUS_OK 0
#define STATUS_FAIL (-1)

#if SENSIRION_BIG_ENDIAN
#define be16_to_cpu(s) (s)
#define be32_to_cpu(s) (s)
#define be64_to_cpu(s) (s)
#define SENSIRION_WORDS_TO_BYTES(a, w) ()

#else /* SENSIRION_BIG_ENDIAN */

#define be16_to_cpu(s) (((uint16_t)(s) << 8) | (0xff & ((uint16_t)(s)) >> 8))
#define be32_to_cpu(s)                                                         \
    (((uint32_t)be16_to_cpu(s) << 16) | (0xffff & (be16_to_cpu((s) >> 16))))
#define be64_to_cpu(s)                                                         \
    (((uint64_t)be32_to_cpu(s) << 32) |                                        \
     (0xffffffff & ((uint64_t)be32_to_cpu((s) >> 32))))
/**
 * Convert a word-array to a bytes-array, effectively reverting the
 * host-endianness to big-endian
 * @a:  word array to change (must be (uint16_t *) castable)
 * @w:  number of word-sized elements in the array (SENSIRION_NUM_WORDS(a)).
 */
#define SENSIRION_WORDS_TO_BYTES(a, w)                                         \
    for (uint16_t *__a = (uint16_t *)(a), __e = (w), __w = 0; __w < __e;       \
         ++__w) {                                                              \
        __a[__w] = be16_to_cpu(__a[__w]);                                      \
    }
#endif /* SENSIRION_BIG_ENDIAN */

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*(x)))
#endif

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define CRC8_LEN 1

#define SENSIRION_COMMAND_SIZE 2
#define SENSIRION_WORD_SIZE 2
#define SENSIRION_NUM_WORDS(x) (sizeof(x) / SENSIRION_WORD_SIZE)
#define SENSIRION_MAX_BUFFER_WORDS 32


#define SFM_CMD_READ_PRODUCT_IDENTIFIER 0xE102

#define SFM_CMD_READ_SCALE_FACTOR_OFFSET_AND_FLOW_UNIT 0x3661

#define SFM_CMD_STOP_CONTINUOUS_MEASUREMENT 0x3FF9

typedef enum {
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS0 = 0x3603,
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS1 = 0x3608,
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS2 = 0x3615,
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS3 = 0x361E,
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS4 = 0x3624,
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS5 = 0x362F,
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS_MIX_0 = 0x3632,
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS_MIX_1 = 0x3639,
    SFM_CMD_START_CONTINUOUS_MEASUREMENT_GAS_MIX_2 = 0x3646,
} SfmCmdStartContinuousMeasurement;

typedef struct {
    uint8_t i2c_address;
    int16_t flow_scale;
    int16_t flow_offset;
} SfmConfig;

const char* SFM_DRV_VERSION_STR = "0.1.0";



/**
 * Return the driver version
 * @return  Driver version string
 */
const char* sfm_common_get_driver_version(void);

/**
 * Detects if a sensor is connected by reading out the ID register.
 * If the sensor does not answer or if the answer is not the expected value,
 * the test fails.
 *
 * @param   i2c_address I2C address to probe
 *
 * @return 0 if a sensor was detected
 */
int16_t sfm_common_probe(uint8_t i2c_address);

/**
 * Read the product identifier, consisting of a product number and a serial
 * number.
 *
 * @param   i2c_address     I2C address to read the product identifier from
 * @param   product_number  Out parameter to store the product number
 * @param   serial_number   Out parameter to store the serial number in raw
 *                          byte format
 *
 * @return  0 on success, an error code otherwise
 */
int16_t sfm_common_read_product_identifier(uint8_t i2c_address,
                                           uint32_t* product_number,
                                           uint8_t (*serial_number)[8]);

/**
 * Read the scale factor, offset and unit for the given measurement type.
 *
 * @param   sfm_config      Pointer to the SFM object
 * @param   measurement_cmd Measurement type to get the scale, offset and unit
 * @param   flow_scale      Out parameter to store the flow scale
 * @param   flow_offset     Out parameter to store the flow offset
 * @param   unit            Out parameter to store the unit
 *
 * @return  0 on success, an error code otherwise
 */
int16_t sfm_common_read_scale_factor_offset_and_unit(
    const SfmConfig* sfm_config,
    SfmCmdStartContinuousMeasurement measurement_cmd, int16_t* flow_scale,
    int16_t* flow_offset, uint16_t* unit);

/**
 * Convert the raw flow ticks to slm.
 *
 * @param   sfm_config      Pointer to the SFM object
 * @param   flow_raw        Flow value as read by
 *                          sfm_common_read_measurement_raw
 * @param   flow            Out parameter to store the converted flow
 *
 * @return  0 on success, an error code otherwise
 */
int16_t sfm_common_convert_flow_float(const SfmConfig* sfm_config,
                                      int16_t flow_raw, float* flow);

/**
 * Convert the raw temperature ticks to degree Celsius.
 *
 * @param   temperature_raw Temperature value as read by
 *                          sfm_common_read_measurement_raw
 *
 * @return  The temperature in degree Celsius
 */
float sfm_common_convert_temperature_float(int16_t temperature_raw);

/**
 * Starts a continuous measurement with the given gas configuration.
 *
 * @param   sfm_config      Pointer to the SFM object
 * @param   measurement_cmd Select which gas or gas mix should be measured
 *
 * @return  0 on success, an error code otherwise
 */
int16_t sfm_common_start_continuous_measurement(
    SfmConfig* sfm_config, SfmCmdStartContinuousMeasurement measurement_cmd);

/**
 * Read results of a continuous measurement
 *
 * @param   sfm_config      Pointer to the SFM object
 * @param   flow_raw        Out parameter for the raw flow value
 * @param   temperature_raw Out parameter for the raw temperature value
 * @param   status          Out parameter for the status word
 *
 * @return  0 on success, an error code otherwise
 */
int16_t sfm_common_read_measurement_raw(const SfmConfig* sfm_config,
                                        int16_t* flow_raw,
                                        int16_t* temperature_raw,
                                        uint16_t* status);

/**
 * Stops a continuous measurement.
 *
 * @param   sfm_config      Pointer to the SFM object
 *
 * @return  0 on success, an error code otherwise
 */
int16_t sfm_common_stop_continuous_measurement(SfmConfig* sfm_config);


uint8_t sensirion_common_generate_crc(uint8_t *data, uint16_t count);

int8_t sensirion_common_check_crc(uint8_t *data, uint16_t count,
                                  uint8_t checksum);

/**
 * Send a general call reset.
 *
 * @warning This will reset all attached I2C devices on the bus which support
 *          general call reset.
 *
 * @return  STATUS_OK on success, an error code otherwise
 */
int16_t sensirion_i2c_general_call_reset(void);

/**
 * sensirion_fill_cmd_send_buf() - create the i2c send buffer for a command and
 *                                 a set of argument words. The output buffer
 *                                 interleaves argument words with their
 *                                 checksums.
 * @buf:        The generated buffer to send over i2c. Then buffer length must
 *              be at least SENSIRION_COMMAND_LEN + num_args *
 *              (SENSIRION_WORD_SIZE + CRC8_LEN).
 * @cmd:        The i2c command to send. It already includes a checksum.
 * @args:       The arguments to the command. Can be NULL if none.
 * @num_args:   The number of word arguments in args.
 *
 * @return      The number of bytes written to buf
 */
uint16_t sensirion_fill_cmd_send_buf(uint8_t *buf, uint16_t cmd,
                                     const uint16_t *args, uint8_t num_args);

/**
 * sensirion_i2c_read_words() - read data words from sensor
 *
 * @address:    Sensor i2c address
 * @data_words: Allocated buffer to store the read words.
 *              The buffer may also have been modified on STATUS_FAIL return.
 * @num_words:  Number of data words to read (without CRC bytes)
 *
 * @return      STATUS_OK on success, an error code otherwise
 */
int16_t sensirion_i2c_read_words(uint8_t address, uint16_t *data_words,
                                 uint16_t num_words);

/**
 * sensirion_i2c_read_words_as_bytes() - read data words as byte-stream from
 *                                       sensor
 *
 * Read bytes without adjusting values to the uP's word-order.
 *
 * @address:    Sensor i2c address
 * @data:       Allocated buffer to store the read bytes.
 *              The buffer may also have been modified on STATUS_FAIL return.
 * @num_words:  Number of data words(!) to read (without CRC bytes)
 *              Since only word-chunks can be read from the sensor the size
 *              is still specified in sensor-words (num_words = num_bytes *
 *              SENSIRION_WORD_SIZE)
 *
 * @return      STATUS_OK on success, an error code otherwise
 */
int16_t sensirion_i2c_read_words_as_bytes(uint8_t address, uint8_t *data,
                                          uint16_t num_words);

/**
 * sensirion_i2c_write_cmd() - writes a command to the sensor
 * @address:    Sensor i2c address
 * @command:    Sensor command
 *
 * @return      STATUS_OK on success, an error code otherwise
 */
int16_t sensirion_i2c_write_cmd(uint8_t address, uint16_t command);

/**
 * sensirion_i2c_write_cmd_with_args() - writes a command with arguments to the
 *                                       sensor
 * @address:    Sensor i2c address
 * @command:    Sensor command
 * @data:       Argument buffer with words to send
 * @num_words:  Number of data words to send (without CRC bytes)
 *
 * @return      STATUS_OK on success, an error code otherwise
 */
int16_t sensirion_i2c_write_cmd_with_args(uint8_t address, uint16_t command,
                                          const uint16_t *data_words,
                                          uint16_t num_words);

/**
 * sensirion_i2c_delayed_read_cmd() - send a command, wait for the sensor to
 *                                    process and read data back
 * @address:    Sensor i2c address
 * @cmd:        Command
 * @delay:      Time in microseconds to delay sending the read request
 * @data_words: Allocated buffer to store the read data
 * @num_words:  Data words to read (without CRC bytes)
 *
 * @return      STATUS_OK on success, an error code otherwise
 */
int16_t sensirion_i2c_delayed_read_cmd(uint8_t address, uint16_t cmd,
                                       uint32_t delay_us, uint16_t *data_words,
                                       uint16_t num_words);
/**
 * sensirion_i2c_read_cmd() - reads data words from the sensor after a command
 *                            is issued
 * @address:    Sensor i2c address
 * @cmd:        Command
 * @data_words: Allocated buffer to store the read data
 * @num_words:  Data words to read (without CRC bytes)
 *
 * @return      STATUS_OK on success, an error code otherwise
 */
int16_t sensirion_i2c_read_cmd(uint8_t address, uint16_t cmd,
                               uint16_t *data_words, uint16_t num_words);
							   
							   
							   /**
 * Detects if a sensor is connected by reading out the ID register.
 * If the sensor does not answer or if the answer is not the expected value,
 * the test fails.
 *
 * @return 0 if a sensor was detected
 */
int16_t sfm3019_probe(void);

/**
 * Create a new SFM3019 instance
 */
SfmConfig sfm3019_create(void);



/**
 * Select the current i2c bus by index.
 * All following i2c operations will be directed at that bus.
 *
 * THE IMPLEMENTATION IS OPTIONAL ON SINGLE-BUS SETUPS (all sensors on the same
 * bus)
 *
 * @param bus_idx   Bus index to select
 * @returns         0 on success, an error code otherwise
 */
int16_t sensirion_i2c_select_bus(uint8_t bus_idx);

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_init(void);

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void);

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_read(uint8_t address, uint8_t *data, uint16_t count);

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_write(uint8_t address, const uint8_t *data,
                           uint16_t count);

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution approximately, but no less than, the given time.
 *
 * When using hardware i2c:
 * Despite the unit, a <10 millisecond precision is sufficient.
 *
 * When using software i2c:
 * The precision needed depends on the desired i2c frequency, i.e. should be
 * exact to about half a clock cycle (defined in
 * `SENSIRION_I2C_CLOCK_PERIOD_USEC` in `sensirion_arch_config.h`).
 *
 * Example with 400kHz requires a precision of 1 / (2 * 400kHz) == 1.25usec.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds);



uint8_t sensirion_common_generate_crc(uint8_t *data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte) {
        crc ^= (data[current_byte]);
        for (crc_bit = 8; crc_bit > 0; --crc_bit) {
            if (crc & 0x80)
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            else
                crc = (crc << 1);
        }
    }
    return crc;
}

int8_t sensirion_common_check_crc(uint8_t *data, uint16_t count,
                                  uint8_t checksum) {
    if (sensirion_common_generate_crc(data, count) != checksum)
        return STATUS_FAIL;
    return STATUS_OK;
}

int16_t sensirion_i2c_general_call_reset(void) {
    const uint8_t data = 0x06;
    return sensirion_i2c_write(0, &data, (uint16_t)sizeof(data));
}

uint16_t sensirion_fill_cmd_send_buf(uint8_t *buf, uint16_t cmd,
                                     const uint16_t *args, uint8_t num_args) {
    uint8_t crc;
    uint8_t i;
    uint16_t idx = 0;

    buf[idx++] = (uint8_t)((cmd & 0xFF00) >> 8);
    buf[idx++] = (uint8_t)((cmd & 0x00FF) >> 0);

    for (i = 0; i < num_args; ++i) {
        buf[idx++] = (uint8_t)((args[i] & 0xFF00) >> 8);
        buf[idx++] = (uint8_t)((args[i] & 0x00FF) >> 0);

        crc = sensirion_common_generate_crc((uint8_t *)&buf[idx - 2],
                                            SENSIRION_WORD_SIZE);
        buf[idx++] = crc;
    }
    return idx;
}

int16_t sensirion_i2c_read_words_as_bytes(uint8_t address, uint8_t *data,
                                          uint16_t num_words) {
    int16_t ret;
    uint16_t i, j;
    uint16_t size = num_words * (SENSIRION_WORD_SIZE + CRC8_LEN);
    uint16_t word_buf[SENSIRION_MAX_BUFFER_WORDS];
    uint8_t *const buf8 = (uint8_t *)word_buf;

    ret = sensirion_i2c_read(address, buf8, size);
    if (ret != STATUS_OK)
        return ret;

    /* check the CRC for each word */
    for (i = 0, j = 0; i < size; i += SENSIRION_WORD_SIZE + CRC8_LEN) {

        ret = sensirion_common_check_crc(&buf8[i], SENSIRION_WORD_SIZE,
                                         buf8[i + SENSIRION_WORD_SIZE]);
        if (ret != STATUS_OK)
            return ret;

        data[j++] = buf8[i];
        data[j++] = buf8[i + 1];
    }

    return STATUS_OK;
}

int16_t sensirion_i2c_read_words(uint8_t address, uint16_t *data_words,
                                 uint16_t num_words) {
    int16_t ret;
    uint8_t i;

    ret = sensirion_i2c_read_words_as_bytes(address, (uint8_t *)data_words,
                                            num_words);
    if (ret != STATUS_OK)
        return ret;

    for (i = 0; i < num_words; ++i)
        data_words[i] = be16_to_cpu(data_words[i]);

    return STATUS_OK;
}

int16_t sensirion_i2c_write_cmd(uint8_t address, uint16_t command) {
    uint8_t buf[SENSIRION_COMMAND_SIZE];

    sensirion_fill_cmd_send_buf(buf, command, NULL, 0);
    return sensirion_i2c_write(address, buf, SENSIRION_COMMAND_SIZE);
}

int16_t sensirion_i2c_write_cmd_with_args(uint8_t address, uint16_t command,
                                          const uint16_t *data_words,
                                          uint16_t num_words) {
    uint8_t buf[SENSIRION_MAX_BUFFER_WORDS];
    uint16_t buf_size;

    buf_size = sensirion_fill_cmd_send_buf(buf, command, data_words, num_words);
    return sensirion_i2c_write(address, buf, buf_size);
}

int16_t sensirion_i2c_delayed_read_cmd(uint8_t address, uint16_t cmd,
                                       uint32_t delay_us, uint16_t *data_words,
                                       uint16_t num_words) {
    int16_t ret;
    uint8_t buf[SENSIRION_COMMAND_SIZE];

    sensirion_fill_cmd_send_buf(buf, cmd, NULL, 0);
    ret = sensirion_i2c_write(address, buf, SENSIRION_COMMAND_SIZE);
    if (ret != STATUS_OK)
        return ret;

    if (delay_us)
        sensirion_sleep_usec(delay_us);

    return sensirion_i2c_read_words(address, data_words, num_words);
}

int16_t sensirion_i2c_read_cmd(uint8_t address, uint16_t cmd,
                               uint16_t *data_words, uint16_t num_words) {
    return sensirion_i2c_delayed_read_cmd(address, cmd, 0, data_words,
                                          num_words);
}


/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication. After this function has been called, the functions
 * i2c_read() and i2c_write() must succeed.
 */
void sensirion_i2c_init(void) {
    
}

/**
 * Release all resources initialized by sensirion_i2c_init().
 */
void sensirion_i2c_release(void) {
}

int8_t sensirion_i2c_read(uint8_t address, uint8_t *data, uint16_t count) {
    uint8_t readData[count];
    uint8_t rxByteCount = 0;

    // 2 bytes RH, 1 CRC, 2 bytes T, 1 CRC
    Wire.requestFrom(address, count);

    while (Wire.available()) {  // wait till all arrive
        readData[rxByteCount++] = Wire.read();
        if (rxByteCount >= count)
            break;
    }

    memcpy(data, readData, count);

    return 0;
}

int8_t sensirion_i2c_write(uint8_t address, const uint8_t *data,
                           uint16_t count) {
    Wire.beginTransmission(address);
    Wire.write(data, count);
    Wire.endTransmission();

    return 0;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_sleep_usec(uint32_t useconds) {
    delay((useconds / 1000) + 1);
}


const char* sfm_common_get_driver_version(void) {
    return SFM_DRV_VERSION_STR;
}

int16_t sfm_common_probe(uint8_t i2c_address) {
    uint16_t buf[6];
    return sensirion_i2c_read_cmd(i2c_address, SFM_CMD_READ_PRODUCT_IDENTIFIER,
                                  buf, 6);
}

int16_t sfm_common_read_product_identifier(uint8_t i2c_address,
                                           uint32_t* product_number,
                                           uint8_t (*serial_number)[8]) {
    uint8_t buf[6 * 2];
    int16_t error =
        sensirion_i2c_write_cmd(i2c_address, SFM_CMD_READ_PRODUCT_IDENTIFIER);
    if (error) {
        return error;
    }
    error = sensirion_i2c_read_words_as_bytes(i2c_address, buf, 6);
    if (error) {
        return error;
    }
    if (product_number) {
        *product_number = ((uint32_t)buf[0] << 24) + ((uint32_t)buf[1] << 16) +
                          ((uint32_t)buf[2] << 8) + (uint32_t)buf[3];
    }
    if (serial_number) {
        for (size_t i = 0; i < 8; ++i) {
            (*serial_number)[i] = buf[i + 4];
        }
    }
    return 0;
}

int16_t sfm_common_read_scale_factor_offset_and_unit(
    const SfmConfig* sfm_config,
    SfmCmdStartContinuousMeasurement measurement_cmd, int16_t* flow_scale,
    int16_t* flow_offset, uint16_t* unit) {

    uint16_t measurement_cmd_word = (uint16_t)measurement_cmd;

    uint16_t buf[3];
    int16_t error = sensirion_i2c_write_cmd_with_args(
        sfm_config->i2c_address, SFM_CMD_READ_SCALE_FACTOR_OFFSET_AND_FLOW_UNIT,
        &measurement_cmd_word, 1);
    if (error) {
        return error;
    }
    error =
        sensirion_i2c_read_words(sfm_config->i2c_address, buf, ARRAY_SIZE(buf));

    if (error) {
        return error;
    }
    if (flow_scale) {
        *flow_scale = (int16_t)buf[0];
    }
    if (flow_offset) {
        *flow_offset = (int16_t)buf[1];
    }
    if (unit) {
        *unit = buf[2];
    }
    return 0;
}

int16_t sfm_common_convert_flow_float(const SfmConfig* sfm_config,
                                      int16_t flow_raw, float* flow) {
    if (sfm_config->flow_scale == 0) {
        return -1;
    }

    *flow =
        (flow_raw - sfm_config->flow_offset) / (float)(sfm_config->flow_scale);

    return 0;
}

float sfm_common_convert_temperature_float(int16_t temperature_raw) {
    return temperature_raw / 200.0f;
}

int16_t sfm_common_start_continuous_measurement(
    SfmConfig* sfm_config, SfmCmdStartContinuousMeasurement measurement_cmd) {

    int16_t error = sfm_common_read_scale_factor_offset_and_unit(
        sfm_config, measurement_cmd, &sfm_config->flow_scale,
        &sfm_config->flow_offset, NULL);
    if (error) {
        return error;
    }

    return sensirion_i2c_write_cmd(sfm_config->i2c_address, measurement_cmd);
}

int16_t sfm_common_read_measurement_raw(const SfmConfig* sfm_config,
                                        int16_t* flow_raw,
                                        int16_t* temperature_raw,
                                        uint16_t* status) {
    uint16_t buf[3] = {};
    int16_t error = sensirion_i2c_read_words(sfm_config->i2c_address, buf, 3);
    if (error) {
        return error;
    }
    if (flow_raw) {
        *flow_raw = (int16_t)buf[0];
    }
    if (temperature_raw) {
        *temperature_raw = (int16_t)buf[1];
    }
    if (status) {
        *status = buf[2];
    }
    return 0;
}

int16_t sfm_common_stop_continuous_measurement(SfmConfig* sfm_config) {
    sfm_config->flow_scale = 0;
    sfm_config->flow_offset = 0;
    return sensirion_i2c_write_cmd(sfm_config->i2c_address,
                                   SFM_CMD_STOP_CONTINUOUS_MEASUREMENT);
}


int16_t sfm3019_probe(void) {
    return sfm_common_probe(SFM3019_I2C_ADDRESS);
}

SfmConfig sfm3019_create(void) {
    SfmConfig sfm_config = {
        SFM3019_I2C_ADDRESS,
        0,
        0,
    };
    return sfm_config;
}
