const express   = require('express');
const http      = require('http');
const mqtt      = require('mqtt');
const socketIo  = require('socket.io');
const cors      = require('cors');

require('dotenv').config();

//Configurar puerto serie para leer datos desde el MKR1310
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const serialPort = new SerialPort({ path: '/dev/cu.usbmodem101', baudRate: 9600 });

// let nodes = {"18": {"isFull": 0, "status": true}}
let nodes = {}
const TIMEOUT_NODES = 15000;
const UPDATE_TIME_SERVER = 20000;


// Crear una aplicación Express
const app = express();

// Crear un servidor HTTP utilizando Express
const server = http.createServer(app);

// Crear una instancia de Socket.IO y pasarle el servidor HTTP
const io = socketIo(server, {
  cors: {
    origin: 'http://localhost:5173',
    methods: ['GET', 'POST'],
  },
});

const port = process.env.WS_PORT || 3000;

// Configurar el broker MQTT y crear una instancia de cliente MQTT
const mqttBroker = process.env.MQTT_BROKER || 'mqtt://test.mosquitto.org';
const mqttClient = mqtt.connect(mqttBroker);

// Configurar middleware de Express para manejar datos JSON y CORS
app.use(express.json());
app.use(cors());

// Manejar conexiones de Socket.IO
io.on('connection', (socket) => {
  console.log('Cliente conectado a través de Socket.IO');

  // Manejar mensajes desde el cliente Socket.IO
  socket.on('messageFromReactWeb', (message) => {
    console.log(`Mensaje cliente: ${message}`);
    mqttClient.publish('emmartel', message);
  });


  // Enviar mensajes MQTT al cliente Socket.IO
  mqttClient.on('message', (topic, message) => {
    socket.emit('messageFromReactWeb', `${message}`);
  });

  // Manejar cierre de conexión
  socket.on('disconnect', () => {
    console.log('Cliente Socket.IO desconectado');
  });
});

// Manejar conexión MQTT
mqttClient.on('connect', () => {
  console.log(`Conectado al broker MQTT en ${mqttBroker}`);

  // Suscribirse a todos los temas de emmartel
  mqttClient.subscribe('emmartel/#', (err) => {
    if (!err) {
      console.log('Suscrito a todos los temas de emmartel');

    } else {
      console.error('Error al suscribirse:', err);
    }
  });
});



const parser = serialPort.pipe(new ReadlineParser({ delimiter: '\r\n' }));
parser.on('data', (data) => {

  mensajeJSON = JSON.parse(data);
  mensajeJSON.timestamp = Date.now()
  const {addr, ...props} = mensajeJSON
  console.log( mensajeJSON)
  // mensajeJSON = {...nodes[Object.keys(mensajeJSON)[0]], mensajeJSON}
  // nodes = {...nodes, mensajeJSON}

  nodes[addr] = props
  updateNodes();
  mqttClient.publish('emmartel', `${JSON.stringify(nodes)}`);
 
});

setInterval(() => {
  // Publicar datos en MQTT
  updateNodes()
  mqttClient.publish('emmartel', `${JSON.stringify(nodes)}`);
}, UPDATE_TIME_SERVER)

const updateNodes = () => Object.keys(nodes).forEach(key => {
  nodes[key].status = ( (Date.now() - nodes[key].timestamp) < TIMEOUT_NODES )

})


// Ruta para verificar que el servidor está en funcionamiento
app.get('/', (req, res) => {
  const message = 'Servidor con Socket.IO y MQTT en funcionamiento';
  
  mqttClient.publish('emmartel', nodes);
  //res.send(nodes);
});

// Iniciar el servidor y escuchar en el puerto especificado
server.listen(port, () => {
  console.log(`Servidor corriendo en http://localhost:${port}`);
});
