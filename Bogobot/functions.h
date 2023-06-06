#include <DynamixelSDK.h>
// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

#define BAUDRATE                        2000000
#define DEVICENAME                      "OpenCR_DXL_Port"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define ESC_ASCII_VALUE                 0x1b0x1b
void readPoseAll(double PresentPose[]) { // Lee la posición actual de todos los 18 servos Dynamixel y lo escribe en el arreglo PresentPose[]
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler("OpenCR_DXL_Port");
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_value;
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
    
  for (int dxl_id = 1; dxl_id <= 18; ++dxl_id) {
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, dxl_id, 36, &dxl_value, &dxl_error);
    PresentPose[dxl_id] = float(dxl_value);
  }
}
//Esta funcion mueve todo el robot a las cuatro posiciones de las dos piernas y los dos brazos
bool movRobot(double PresentPose[], double Pose[], double dxl_goal_position[], float tf){
    float startMillis;
    float currentMillis;
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    //Creamos el objeto de escritura sincronica de posicion
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
    dynamixel::GroupSyncWrite groupSyncWriteVel(portHandler, packetHandler, 32, LEN_MX_GOAL_POSITION);
    double dxl_goal_position2[20];
    uint8_t param_goal_position[2];
    uint8_t param_goal_vel[2];
    bool dxl_addparam_result = false;
    int dxl_comm_result = COMM_TX_FAIL; 
    //Obtenemos la cinematica inversa de las piernas
    IK_Feet(Pose, dxl_goal_position);
    //Obtenemos cinematica inversa de los brazos
    IK_arms(Pose, dxl_goal_position2);
    //Creamos el contador
    startMillis = millis()/1000.0;
    float t_ac = 0.0;
    float dt = 0.0;
    double q;
    //Mientras no pases el tiempo final
    while(t_ac <= tf){
      //Contamos
      currentMillis = millis()/1000.0;
      //Creamos un diferenciald e tiempo
      dt = currentMillis - startMillis;
      //Contamos el tiempo
      t_ac = t_ac + dt;
      float q;
      //Por cada parte del robot obtenemos la pequena trayectoria con la interpolacion de quinto orden
      for(int i = 1; i <= 6; i++){
        q = interpolation(t_ac ,tf,PresentPose[i],dxl_goal_position[i]);
        //Separamos la posicion de 16 bits en dos paquetes de 8 bits
        param_goal_position[0] = DXL_LOBYTE(q);
        param_goal_position[1] = DXL_HIBYTE(q);
        float vel = abs((dxl_goal_position[i]-PresentPose[i])/tf);
        //Separamos la velocidad de 16 bits en dos paquetes de 8 bits
        param_goal_vel[0] = DXL_LOBYTE(vel);
        param_goal_vel[1] = DXL_HIBYTE(vel);
        //Guardamos velocidad y posicion en la pila
        dxl_addparam_result = groupSyncWriteVel.addParam(i, param_goal_position);
        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);//
      }
      //Esto lo repetimos por cada parte del robot
      for(int i = 7; i <= 12; i++){
        q = interpolation(t_ac ,tf,PresentPose[i],dxl_goal_position[i]);
        //float val = millis()/1000.0;
        //Serial.println(q);
        param_goal_position[0] = DXL_LOBYTE(q);
        param_goal_position[1] = DXL_HIBYTE(q);
        float vel = abs((dxl_goal_position[i]-PresentPose[i])/tf);
        param_goal_vel[0] = DXL_LOBYTE(vel);
        param_goal_vel[1] = DXL_HIBYTE(vel);
        dxl_addparam_result = groupSyncWriteVel.addParam(i, param_goal_position);
        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);//
      }
      for(int i = 13; i <= 15; i++){
        q = interpolation(t_ac ,tf,PresentPose[i],dxl_goal_position2[i]);
        //float val = millis()/1000.0;
        //Serial.println(q);
        param_goal_position[0] = DXL_LOBYTE(q);
        param_goal_position[1] = DXL_HIBYTE(q);
        float vel = abs((dxl_goal_position[i]-PresentPose[i])/tf);
        param_goal_vel[0] = DXL_LOBYTE(vel);
        param_goal_vel[1] = DXL_HIBYTE(vel);
        dxl_addparam_result = groupSyncWriteVel.addParam(i, param_goal_position);
        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);//
      }
      for(int i = 16; i <= 18; i++){
        q = interpolation(t_ac ,tf,PresentPose[i],dxl_goal_position2[i]);
        //float val = millis()/1000.0;
        //Serial.println(q);
        param_goal_position[0] = DXL_LOBYTE(q);
        param_goal_position[1] = DXL_HIBYTE(q);
        float vel = abs((dxl_goal_position[i]-PresentPose[i])/tf);
        param_goal_vel[0] = DXL_LOBYTE(vel);
        param_goal_vel[1] = DXL_HIBYTE(vel);
        dxl_addparam_result = groupSyncWriteVel.addParam(i, param_goal_position);
        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);//
      }
      //Mandamos las posiciones para que se hagan al mismo tiempo
      dxl_comm_result = groupSyncWrite.txPacket();
      //Madamos las velocidades al mismo tiempo
      dxl_comm_result = groupSyncWriteVel.txPacket();
      //Limpiamos la pila
      groupSyncWrite.clearParam();
      //Actualziamos contador
      startMillis = currentMillis;  
    }
    return true;
  
}
void enableAll() { // Habilita la torca de todos los 18 sevos Dynamixel
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION); 
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
  uint8_t dxl_error = 0;                          // Dynamixel error
  portHandler->openPort();
  portHandler->setBaudRate(BAUDRATE);
  
  for (int dxl_id = 1; dxl_id <= 18; ++dxl_id) {    
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, dxl_id, 24, 1, &dxl_error);
  }
  portHandler->closePort();
}
