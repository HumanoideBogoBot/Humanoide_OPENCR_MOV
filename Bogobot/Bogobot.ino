#include "IK.h"
#include "functions.h"
//Leonardo Gracida Munoz A01379812
//Daniel Fuentes Castro A01750425
//Santiago Ortiz Suzarte A01750402

//Posiciones que vamos a iterar de una a la otra
double Pose[14] = {0,0,-30,0,0,0,-30,0,5,15,-30,-5,15,-30}; //Sentadilla
double Pose2[14] = {0,0,-37,0,0,0,-37,0,0,0,-39,0,0,-39}; //Erguido
double Vel[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};//Arreglo donde guardamos las velocidades de cada joint
double dxl_goal_position[20]; //Arreglo donde guardamos la posicion deseada de todos los joint
double PresentPose[19]; //Arreglo donde fuardamos la posicion actual

void setup() {
  //Iniciamos el codigo hasta que abramos el puerto serial
  while(!Serial);
  //Tanto las variables DEVICENAME,PROTOCOL_VERSION,BAUDRATE, como llamar a la libreria de dynamixel esta en el archivo functions.h
  //Creamos el por HANDLER
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  //Creamos el archivo Protocol version
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  //Abrimos el puerto
  portHandler->openPort();
  //Declaramos el Baudrate
  portHandler->setBaudRate(BAUDRATE);
  //Todas las funciones estan en el archivo functions.h
  //Habilitamos el torque en todos los servos
  enableAll();
}
bool activo;
void loop() {
  // put your main code here, to run repeatedly:
  //Definimos el tiempo en el que queremos completar la trayectoria
  float tf = 2.0;
  //Leemos posicion actual
  readPoseAll(PresentPose);
  //Movemos el robot a la posicion deseada
  activo = movRobot(PresentPose,Pose,dxl_goal_position,tf);
  //Leemos posicion actual
  readPoseAll(PresentPose);
  //Movemos hacia la segunda posicion
  activo = movRobot(PresentPose,Pose2,dxl_goal_position,tf);
  
}
