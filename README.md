# Ejemplo de semáforos en freeRTOS con arduino pro micro
El programa está compuesto por seis tareas 

* **TaskSendData** (Encargada de enviar los datos mediante puerto serie)
* **TaskAnalogRead1** (Lectura del ADC-0)
* **TaskAnalogRead2** (Lectura del ADC-1) 
* **TaskAnalogRead3** (Lectura del ADC-2)
* **TaskAnalogRead4** (Lectura del ADC-3)
* **filtro** (Butterworth de 4 órden pasa banda con fc= 50Hz fs=250Hz para la señal de la tarea **TaskAnalogRead4**)

¿Como compilar?
-
  ```
  cd carpeta/del/proyecto
  platformio run
  ```
¿Como compilar y cargar?
-
  ```
  platformio run -t upload
  ```
