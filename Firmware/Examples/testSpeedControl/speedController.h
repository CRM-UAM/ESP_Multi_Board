

/***************************************************
  Developed by Victor Uceda for Club de Robotica of Universidad Autonoma de Madrid (CRM-UAM).
  Original code of Project Futura http://micromouseusa.com/?page_id=1342
 ****************************************************/

#ifndef _SPEED_CONTROLLER
#define _SPEED_CONTROLLER

  #include <stdlib.h>
  #include <ESP_Multi_Board.h>


  void speedProfile(ESP_Multi_board *robot);
  void resetSpeedProfile(ESP_Multi_board *robot);

  int needToDecelerate(int32_t dist, int16_t curSpd, int16_t endSpd);//speed are in encoder counts/ms, dist is in encoder counts


#endif
