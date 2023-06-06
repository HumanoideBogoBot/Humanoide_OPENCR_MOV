# Humanoide_OPENCR_MOV
Aqui esta el paquete que permite hacer trayectorias del las piernas y brazos del Bogobot

- El robor trabaja con el protocolo 1 de la librería de Dynamixel.
- El paquete contiene la funcion que hace el movimeinto sincronico de los servos del robot, ademas de que esta funcion movRobot solo solicita la pisicon X,Y,Z de los brazos y piernas y el tiempo en que queires completar la trayectoria, como su posicion actua y la deseada, esta funcion ya hace la cinematica inversa del robot de manera correcta, ademas de que tambien hace la interpolacion de quinto orden.

- En la caarpeta Bogobot esta el archivo pricipal Bogobot.ino que llama todas las funciones, en el archivo functions.h viene le funcion movRobot, y demas protocolos, en el archivo IK.h vienen las funciones que hacen el calculo de la cinematica inversa y el archivo poses.h viene la pose offset o base para haecr el cero ralitivo o posicio cero del robot que es cuandoe sta erguido.

- Este paquete esta diseñado para el robot Bogobot con el microcontrolador OpenCR.
