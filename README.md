# pico-wifi
Continuously capture ADC data and transmit it via HTTP(S) or MQTT(S) using Pico W Wi-Fi.

## Configuration

The MACRO's in the `smart_grid/include/constants.h` file specifies the configuration parameters.

- Configure the Wi-Fi credentials using macros `WIFI_SSID`, `WIFI_PASSWORD`.

- Configure the server hostname and ports using macros `SERVER_HOSTNAME`, `SERVER_IP`, `SERVER_PORT_...`.

- Configure the ADC sampling rate using macros `Fm`, `SAMPLING_FACTOR` or `Fs`.

- Ensure `HTTP_CONTENT_LENGTH` is double the `DMA_BUFFER_SIZE`, as the former specifies number of Bytes of data to send and the later specifies the number of 16-bit integers.

- `MAX_DATA_LENGTH` specifies the maximum byte-array length to fit the total HTTP request data including the Headers.

- Configure the HTTP path or MQTT topic using macros `BASE_PATH`, `CLIENT_ID` or `UPLOAD_PATH`.

- Configure the CA Certificate using the macro `CA_CERT`.

- Make note of the macro `MQTT_OUTPUT_RINGBUF_SIZE` in `smart_grid/include/lwipopts.h`. It is the maximum number of bytes of the MQTT packet data. As per documentation it should be a power of 2. It is set to 2048 for MQTT. The same should work for MQTTS too but it is working with 512.
