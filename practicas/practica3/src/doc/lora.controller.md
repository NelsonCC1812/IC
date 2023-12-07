# LoRa controller
* La API contiene el archivo `.h` tiene cosas de uso común
* Contiene una instancia de un objeto llamado `lora`, que contiene la API

- [LoRa controller](#lora-controller)
    - [Protocolo](#protocolo)
      - [Estructura del paquete](#estructura-del-paquete)
    - [Lora API](#lora-api)
      - [Métodos](#métodos)
      - [Códigos de operación](#códigos-de-operación)
    - [Variables de control](#variables-de-control)
      - [Códigos de error](#códigos-de-error)
    - [¿Que contiene el mensaje?](#que-contiene-el-mensaje)



### Protocolo
* Los paquetes son de 5 bytes de cabecera (1x recipient, 1x sender, 2x message_id, 1x operation_code, 1x payload_length)
* El tamaño del payload es de máximo 10 bytes ( es un valor arbitrario)

#### Estructura del paquete

| byte  | Field |
| :---: | :---: |#### Códigos de operación

|     7     |                    6                    |   5   |    4    |      3      |   2   |     1     |      0      |
| :-------: | :-------------------------------------: | :---: | :-----: | :---------: | :---: | :-------: | :---------: |
|     x     |                    x                    |   x   | txPower | Coding Rate |  SPF  | bandwidth | Config Mode |
|     1     |           destination address           |
|     2     |             sender address              |
|     3     |             Msg Counter HB              |
|     4     |             Msg Counter LB              |
|     5     | [Operation Code](#códigos-de-operación) |
|     6     |           Payload Length (n)            |
| 7 - (7+n) |                 Payload                 |
|   (8+n)   |               End Segment               |



### Lora API
#### Métodos
La API se usa llamando a métodos de la instancia `lora`:

```cpp
typedef struct {
    uint8_t bandwidth_index;
    uint8_t spreadingFactor;
    uint8_t codingRate;
    uint8_t txPower;
} LoRaConfig_t;

typedef struct {
    uint8_t rcpt, sender;
    uint16_t id;
    uint8_t opCode;

    uint8_t payloadLength;
    uint8_t payload[PAYLOAD_SIZE];

    uint8_t rssi, snr;

    bool endRecieved;
} LoraMessage_t;

struct {
    // fields
    uint8_t err = 0; // error code
    uint8_t localAddr, destAddr;
    uint16_t msgCount = 0;
    LoraMessage_t msg;

    // methods
    void (*init)(uint8_t _localAddr, uint8_t _destAddr) = init;
    LoRaConfig_t(*extractConfig)(byte configMask) = extractConfig;
    void (*applyConfig) (LoRaConfig_t config) = applyConfig;
    void (*receive) () = LoRa.receive;

} lora;
```

#### Códigos de operación

|   7   |   6   |   5   |    4    |      3      |   2   |     1     |      0      |
| :---: | :---: | :---: | :-----: | :---------: | :---: | :-------: | :---------: |
|   x   |   x   |   x   | txPower | Coding Rate |  SPF  | bandwidth | Config Mode |

### Variables de control

La API hace uso de ciertas variables de control que están built-in la instancia `lora`:
* LoraMessage_t msg; // Aquí se encuentran los datos del mensaje


#### Códigos de error

| lora.err | Meaning                                         |
| -------- | ----------------------------------------------- |
| 0        | no error                                        |
| 1        | no end of message received                      |
| 2        | payloadLength not coincides with actual payload |
| 3        | Noise excedes signal (percentage)               |
| 4        | Target message incorrect                        |
| 99       | payload excedes buffer                          |


### ¿Que contiene el mensaje?
El mensaje contiene el último mensaje que se ha recibido.
