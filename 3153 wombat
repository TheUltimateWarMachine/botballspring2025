#include <kipr/wombat.h>
#include <string.h>
#define MOTORl 1
#define MOTORr 0
#define ARM 0
#define CLAW 1
#define RIGHT 0
#define LEFT 1
#define MAX_MOTOR_SPEEDr -1500
#define MAX_MOTOR_SPEEDl 1500
#define GRAY 3500
#define WHITE 2400

#define LEFT_MOTOR_CORRECTION 1.0
#define RIGHT_MOTOR_CORRECTION 0.93
#define SENSORl 1 //left color sensor
#define SENSORr 0 //right color sensor
#define LEFT_TURN 1 //turn left mode
#define RIGHT_TURN 0 //turn right mode


void set_mav_speed(int motor_id, int speed) {
    if (motor_id == MOTORl) {
        mav(MOTORl, LEFT_MOTOR_CORRECTION * speed);
    } else if (motor_id == MOTORr) {
        mav(MOTORr, RIGHT_MOTOR_CORRECTION * speed);
    } else {
        printf("invalid motor_id: %d\n", motor_id);
    }
    
}

void check_line_sensors(){
    while(1){
        printf("left value = %d, right value = %d\n", analog(SENSORl), analog(SENSORr));
        msleep(1000);
    }
}




void line_follow(float distance) {
    //follow black line for set distance.
    int rhigh = MAX_MOTOR_SPEEDr;
    int rlow = 0.6 * MAX_MOTOR_SPEEDr;
    int lhigh = MAX_MOTOR_SPEEDl;
    int llow = 0.6* MAX_MOTOR_SPEEDl;
    cmpc(MOTORl);
    cmpc(MOTORr);
    while (gmpc(MOTORl) < distance && gmpc(MOTORr) < distance) {
        msleep(1);
        // if both sensors do not detect black, drive forward
        // if the left sensor detects black, turn left
        // if the right sensor detects black, turn right
        if (analog(SENSORl) <= GRAY && analog(SENSORr) <= GRAY) {  
            set_mav_speed(MOTORl, lhigh);
            set_mav_speed(MOTORr, rhigh);
           
        } else if (analog(SENSORl) >= GRAY) {
            set_mav_speed(MOTORl, llow);
            set_mav_speed(MOTORr, rhigh);

        } else if(analog(SENSORr) >= GRAY) {
            set_mav_speed(MOTORl, lhigh);
            set_mav_speed(MOTORr, rlow);
            
        }
    }
    set_mav_speed(MOTORl, 0);
    set_mav_speed(MOTORr, -10);
    msleep(50);
    ao();
    msleep(50);
}

    
   


void drive(int speed1,int speed2) {
    mav(0, speed2);
    mav(1,speed1);
}
void move_arm(int end_pos) {
    // speed discount = multiplier
    if (get_servo_position(ARM) < end_pos) {
        // If start position is higher, lower arm. 
        while (get_servo_position(ARM) < end_pos) {
            set_servo_position(ARM, get_servo_position(ARM) + 30);
            msleep(50); // try 30 and 20
        }
    } else {
        // If start position is lower, raise arm.
        while (get_servo_position(ARM) > end_pos) {
            set_servo_position(ARM, get_servo_position(ARM) - 30);
            msleep(50); // try 30 and 20
        } 
    }    
}
void right_turn() {
    mav(0,500);
    mav(1,500);
    msleep(2500);
}
void left_turn() {
    mav(0,-500);
    mav(1,-500);
    msleep(2500);
    
}
void left_45() {
    mav(0,-500);
    mav(1,-500);
    msleep(1250);
    
   
}
void left_60() {
    mav(0,-500);
    mav(1,-500);
    msleep(1850);
}
void move_claw(int end_pos) {
        if (get_servo_position(CLAW) < end_pos) {
        // If start position is higher, lower arm. 
        while (get_servo_position(CLAW) < end_pos) {
            set_servo_position(CLAW, get_servo_position(CLAW) + 30);
            msleep(50); // try 30 and 20
        }
    } else {
        // If start position is lower, raise arm.
        while (get_servo_position(CLAW) > end_pos) {
            set_servo_position(CLAW, get_servo_position(CLAW) - 30);
            msleep(50); // try 30 and 20
        } 
    }
    
}   
void drive_till_blk(){
    while (analog(SENSORl) <= GRAY && analog(SENSORr) <= GRAY) {
        drive(1000,-1000);
    }
}
/* 
- Go Straight for distance x
-90 degree
go distance y
-Claw
-reverse distance y
-90 degree left
-Move distance z
*/
int main() {
   enable_servos();
   set_servo_position(ARM,0);
   move_claw(183);
   drive_till_blk();
   drive(1000,-1000);
   msleep(900);
   right_turn();
   line_follow(6850);
   left_turn();
   drive(500,-500);
   msleep(250);
   move_arm(640);
   move_claw(1300);
   set_servo_position(0,0);
   drive(-500,500);
   msleep(250);
   left_turn();
   left_60();
   drive(500,-500);
   msleep(2800);
   move_arm(600);
   set_servo_position(1,200);

   /*set_servo_position(0,0);
   drive(-500,500);
   msleep(2800);
   left_turn();
   left_45();
   drive(-500,-500);
   msleep(500);
   drive(500,-500);
   msleep(900);
   move_arm(620);
   move_claw(1300);
   set_servo_position(0,0);
   drive(-500,500);
   msleep(2000);
   left_turn();
   left_turn();
   drive(1000,-1000);
   msleep(2000);
   move_arm(600);
   set_servo_position(1,0);

   ao();*/
    
}
