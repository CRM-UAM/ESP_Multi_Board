

/***************************************************
  Developed by Victor Uceda for Club de Robotica of Universidad Autonoma de Madrid (CRM-UAM).
  Original code of Project Futura http://micromouseusa.com/?page_id=1342
 ****************************************************/

#ifndef _SPEED_CONTROLLER
#define _SPEED_CONTROLLER

 




  void speedProfile();
  void resetSpeedProfile();

  int needToDecelerate(long dist, int curSpd, int endSpd);//speed are in encoder counts/ms, dist is in encoder counts


#endif
