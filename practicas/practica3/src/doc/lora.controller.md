# LoRa controller
* La API contiene el archivo `.h` tiene cosas de uso común
* Contiene una instancia de un objeto llamado `lora`, que contiene la API

- [LoRa controller](#lora-controller)
    - [Protocolo](#protocolo)
      - [Estructura del paquete](#estructura-del-paquete)
      - [Modo `discovered`](#modo-discovered)
    - [Lora API](#lora-api)
      - [Métodos](#métodos)
        - [Metodos](#metodos)
          - [Mandar mensajes](#mandar-mensajes)
      - [Parámetros de configuración (`LoRaConfig_t`)](#parámetros-de-configuración-loraconfig_t)
      - [Códigos de operación](#códigos-de-operación)
    - [Variables de control](#variables-de-control)
      - [Códigos de error](#códigos-de-error)
    - [¿Que contiene el mensaje?](#que-contiene-el-mensaje)
- [Links](#links)
- [ToDo](#todo)
  - [No tenemos conexion (**`discover`**)](#no-tenemos-conexion-discover)
  - [ignorar mensajes](#ignorar-mensajes)



### Protocolo
* Los paquetes son de 5 bytes de cabecera (1x recipient, 1x sender, 2x message_id, 1x operation_code, 1x payload_length)
* El tamaño del payload es de máximo 10 bytes ( es un valor arbitrario)

#### Estructura del paquete


|   byte    |                 Content                 |
| :-------: | :-------------------------------------: |
|     1     |             sender address              |
|     2     |           destination address           |
|     3     |             Msg Counter HB              |
|     4     |             Msg Counter LB              |
|     5     | [Operation Code](#códigos-de-operación) |
|     6     |           Payload Length (n)            |
| 7 - (7+n) |                 Payload                 |
|   (8+n)   |               End Segment               |

#### Modo `discovered`

Cuando no se tiene conexión con la placa

<!-- TODO: Hay que elegir si quien inicia la conexión es el slave o el master -->



### Lora API
#### Métodos
La API se usa llamando a métodos de la instancia `lora`:
<!-- TODO -->
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
    bool (*receive) () = LoRa.receive;

} lora;
```

##### Metodos

###### Mandar mensajes

> Cuando mandamos un mensaje a la dirección de broadcast, no esperamos ACK. Por lo que aún pidiendo ACK, no se devolverá.

#### Parámetros de configuración (`LoRaConfig_t`)

* `BandWith` [0-9]: Existe un array que contiene las variables reales de la configuración.
* `spreadingFactor` [6-12]: El 6 es un valor especial.
* `condingRate` [5-8]
* `txPower` [2-20]


#### Códigos de operación

> Un 1 en el bit 7 significa que nos encontramos en el modo de configuración:


|      7      |   6   |   5   |   4   |    3    |      2      |   1   |     0     |
| :---------: | :---: | :---: | :---: | :-----: | :---------: | :---: | :-------: |
| Config Mode |   x   |   x   |   x   | txPower | Coding Rate |  SPF  | bandwidth |

Fuera del modo de configuración, tenemos 64 valores posibles (0-63) para pasar:

| Operation Code |     Meaning     |
| :------------: | :-------------: |
|       0        | Control message |
|       1        |   Acknowledge   |
|       2        | No acknowledge  |
|       3        |  Data message   |
|       10       |    discover     |
|       20       | request config  |
|       21       |   send config   |
|       30       |      ping       |

> Un 1 en el bit 6 significa que el mensaje espera un ACK


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

# Links
[rssi & snr](https://www.thethingsnetwork.org/docs/lorawan/rssi-and-snr/)


# ToDo

## No tenemos conexion (**`discover`**)
* Aumentamos SPF al maximo
* Bajamos BW al minimo


* Iteramos por todo el espectro de potencias (2-20), reintentando 1-5 veces esperando por un ACK.
* Cada mensaje contiene la potencia a la que se mandó. Para así mandar el ACK a dicha potencia. 


## ignorar mensajes

* Cuando siguen llegando acks, y no se recive respuesta, ambos deben no aplicar los cambios e ignorar lo que se haya mandado, si es necesario, volver a mandar el mensaje o reconectar.