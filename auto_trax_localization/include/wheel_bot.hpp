#pragma once
#include <iostream>
#include <Eigen/Core>


class WheelBot{

public:
  WheelBot();
  WheelBot(float initial_x, float initial_y, float initial_theta,
            float sigma_x, float sigma_y, float sigma_theta);
  float getX();
  float getY();
  float getTheta();

  void addX(float x);
  void addY(float y);

  void setX(float x);
  void setY(float y);
  void setTheta(float theta);


  void setPose(float x, float y, float theta);

  double getWeight();

  void setWeight(double w);

private:
  float x_,y_,theta_;

  double weight_;
};
