#include "wheel_bot.hpp"

WheelBot::WheelBot() {
  this->setTheta( (float) rand() / (RAND_MAX)  );
  this->setX(  (float) rand() / (RAND_MAX)  );
  this->setY( (float) rand() / (RAND_MAX)  );
}

WheelBot::WheelBot(float initial_x, float initial_y, float initial_theta,
         float sigma_x, float sigma_y, float sigma_theta)
{
  this->setX( 2 * sigma_x * (float) rand() / (RAND_MAX) - sigma_x + initial_x );
  this->setY( 2 * sigma_y *  (float) rand() / (RAND_MAX) - sigma_y + initial_y );
  this->setTheta( 2 * sigma_theta *  (float) rand() / (RAND_MAX) - sigma_theta + initial_theta );

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
