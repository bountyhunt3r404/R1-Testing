#include <Arduino.h>

#include <PS4Controller.h>
#include <CytronMotorDriver.h>
#include <BasicLinearAlgebra.h>
#include <math.h>

// Pinouts
  //  ESP32       ->      Cytron
    

//Top Right Motor    [W1]
#define motorTR_dir 14
#define motorTR_pwm 27

//Top Left Motor     [W2]
#define motorTL_dir 12
#define motorTL_pwm 13

//Bottom Left Motor  [W3]
#define motorBL_dir 25
#define motorBL_pwm 26

//Bottom Right Motor [W4]
#define motorBR_dir 32
#define motorBR_pwm 33

//Left Gripper Motor
#define leftMotor_dir 5
#define leftMotor_pwm 18

//Right Gripper Motor
#define rightMotor_dir 19
#define rightMotor_pwm 21

const int const_limit = 8;
const float speed_limit = 30.0;
const double pi = 2*acos(0.0);
const double angle = pi/2;
const int R = 1;            //R -> is the radius of the robot from the center to the wheel
const int r = 1;            //r -> is the radius of the wheel

//Drive Motors
CytronMD motorTR(PWM_DIR, motorTR_pwm, motorTR_dir);  
CytronMD motorTL(PWM_DIR, motorTL_pwm, motorTL_dir);
CytronMD motorBL(PWM_DIR, motorBL_pwm, motorBL_dir);  
CytronMD motorBR(PWM_DIR, motorBR_pwm, motorBR_dir);

//Gripper Motors
CytronMD leftMotor(PWM_DIR, leftMotor_pwm, leftMotor_dir);
CytronMD rightMotor(PWM_DIR, rightMotor_pwm, rightMotor_dir);

using namespace BLA;

BLA::Matrix<4, 3, float_t> Inverse_Matrix;

BLA::Matrix<4,1,float> calculate_vel(float vx, float vy, float theta) {
                        Inverse_Matrix = {-sin(angle + (pi/4)), cos(angle +(pi/4)), R,
                                          -sin(angle + 3*(pi/4)), cos(angle + 3*(pi/4)), R,
                                          -sin(angle + 5*(pi/4)), cos(angle + 5*(pi/4)), R,
                                          -sin(angle + 7*(pi/4)), cos(angle + 7*(pi/4)), R};

                        BLA::Matrix<3, 1, float> Vel_matrix = {vx, -vy, theta};
                        BLA::Matrix<4, 1, float> V_W = Inverse_Matrix * Vel_matrix * float(1/r);

                        return V_W;
}

void print_ps4_data(){
      Serial.printf("Left Stick x at %d: ", PS4.LStickX());
      Serial.printf("Left Stick y at %d: ", PS4.LStickY());
      Serial.printf("Right Stick x at %d: ", PS4.RStickX());
      Serial.printf("Right Stick y at %d\n", PS4.RStickY());
}

int dead_zone(int val, int limit = const_limit) {
  if (val >= -limit && val <= limit) {
    return 0;
  }
  else {
    if (val>0){
      return val-limit;
    }

    else {
      return val+limit;
    }
  }
}

void send_vel(int w1, int w2, int w3, int w4) {
    motorTR.setSpeed(w1);
    motorTL.setSpeed(w2);
    motorBL.setSpeed(w3);
    motorBR.setSpeed(w4);

    // Serial.printf("W1 -> %d  W2 -> %d  W3 -> %d  W4 -> %d\n", w1, w2, w3, w4);

}

void setup() {
    Serial.begin(115200);
    send_vel(0, 0, 0, 0);
    // PS4.begin("48:e7:da:44:75:08");
    PS4.begin("e4:5f:01:3c:a6:23");
    Serial.println("Setup Complete");
}

void loop() {
    if (PS4.isConnected()) {
        //Left JoyStick
        int LY = dead_zone(PS4.LStickY());
        LY = LY * 2 * (speed_limit/100.0);

        //Right JoyStick
        int RX = dead_zone(PS4.RStickX());
        RX = RX * 2 * (speed_limit/100.0);

        int LT_ANALOG = PS4.L2Value() * (speed_limit/100.0);  //Left Trigger
        int RT_ANALOG = PS4.R2Value() * (speed_limit/100.0);  //Right Trigger

        int vx = LY;
        int vy = RX;
        float theta = 0.0;

        theta = map(RT_ANALOG-LT_ANALOG, -255*(speed_limit/100.0), 255*(speed_limit/100.0), -1000, 1000) * (pi/1000.0000);
        
        if (PS4.L2()) {
          leftMotor.setSpeed(100);
        }
        else if (PS4.L1()){
          leftMotor.setSpeed(-100);
        }
        else {
          leftMotor.setSpeed(0);
        }

        if (PS4.R1()) {
          rightMotor.setSpeed(100);
        }
        else if (PS4.R2()){
          rightMotor.setSpeed(-100);
        }
        else {
          rightMotor.setSpeed(0);
        }

        BLA::Matrix<4,1,float> V_W = calculate_vel(vx, vy, theta);
        // Serial.printf("%f ", V_W(0));
        // Serial.printf("%f ", V_W(1));
        // Serial.printf("%f ", V_W(2));
        // Serial.printf("%f\n", V_W(3));


        send_vel(int(V_W(0)), int(V_W(1)), int(V_W(2)), int(V_W(3)));

        Serial.printf("VX -> %d VY -> %d THETA -> %f  : LTRIG -> %d RTRIG -> %d\n", vx, vy, theta, LT_ANALOG, RT_ANALOG);
        // Serial.printf("W1 -> %d  W2 -> %d  W3 -> %d  W4 -> %d\n", int(V_W(0)), int(V_W(1)), int(V_W(2)), int(V_W(3)));
    }

    else {
      send_vel(0, 0, 0, 0);
      Serial.println("No PS4 Data");
    }
}
