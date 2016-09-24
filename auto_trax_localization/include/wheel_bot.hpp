#pragma once
#include <iostream>


class WheelBot{

public:
  WheelBot();
  float getX();
  float getY();
  float getTheta();

  void addX(float x);
  void addY(float y);

  void setX(float x);
  void setY(float y);
  void setTheta(float theta);


  void setPose(float x, float y, float theta);


private:
  float x_,y_,theta_;
};
