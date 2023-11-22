```
// void loop() {
//     snprintf(bff, 100, "%d", idx++);
//     oled.print(bff);
//     delay(1000);
// }

/**
 * => Leer
 * if( Serial1.available() > 0){
 * data = Serial.read()
 * }
 *
 * => Escribir
 * Serial1.write(cosas)
*/
```

Aunque lo ideal es hacer un sistema mediante interrupciones, como no nos han dado el uso de los timers
he optado por un sistema que compare intantes de tiempo