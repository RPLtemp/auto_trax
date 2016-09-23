#pragma once
#include <iostream>


class WheelBot{

public:
  WheelBot();
  float getX();
  float getY();
  float getTheta();

  void setX(float x);
  void setY(float y);
  void setTheta(float theta);


private:
  float x_,y_,theta_;
};