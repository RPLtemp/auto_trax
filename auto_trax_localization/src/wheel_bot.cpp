#include "wheel_bot.hpp"

WheelBot::WheelBot() {
  this->setTheta( (float) rand() / (RAND_MAX)  );
  this->setX(  (float) rand() / (RAND_MAX)  );
  this->setY( (float) rand() / (RAND_MAX)  );
}

float WheelBot::getX() {return x_;}
float WheelBot::getY() { return y_;}
float WheelBot::getTheta() {return theta_;}

void WheelBot::setX(float x) {x_ = x;}
void WheelBot::setY(float y) {y_ = y;}

void WheelBot::addX(float x) {x_ += x;}
void WheelBot::addY(float y) {y_ += y;}

void WheelBot::setTheta(float theta) {theta_ = theta;}
void WheelBot::setPose(float x, float y, float theta){x_ = x; y_ = y; theta_ = theta;}

double WheelBot::getWeight() {
  return weight_;
}

void WheelBot::setWeight(double w) {
  weight_ = w;
}
