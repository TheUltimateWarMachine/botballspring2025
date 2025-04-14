#include <stdarg.h>
#include <kipr/wombat.h>
#include <stdbool.h>
//motor points
#define BL 2
#define FL 3
#define FR 0
#define BR 1
//tophat analog ports
#define TH_LEFT 1
#define TH_RIGHT 0
//servos and claw constants
#define CLAW_NO_CHANGE -1 //For claw_rotate - Do not change a servo's position
#define ARM_Z 0
#define ARM_Y 1
#define CLAW_SERVO 2 // Servo port for claw
#define CLAW_OPEN 00 // Open position for claw servo
#define CLAW_CLOSED 1600 // Closed position for claw servo
#define ARM_DOWN 500 // Lower position for arm motor
#define ARM_UP 1500 // Raised position for arm motor
#define abs(x) ((x) < 0 ? -(x) : (x))
//sign of x
#define sgn(x) ((x) > 0 ? 1 : ((x) == 0 ? 0 : -1))
#include "mmgyro.h"
void straight_square_up(int speed, int tick_guess);
bool is_black(int reading);
//Is reading above black threshold
bool is_black(int reading) {
    //1500 = black threshold
	return (reading > 1500);
}

bool conditional_is_black(void *np) {
	return is_black(analog(TH_LEFT)) || is_black(analog(TH_RIGHT));
}

//square up, move forward first (after tick guess, will move slowly to increase accuracy)
void straight_square_up(int speed, int tick_guess) {
    mmcmpc_all(FL, BL, FR, BR, mmend);
    //dynspeed is adjusted to increase accuracy
    int dynspeed = speed;
	while(true) {
        if(speed != dynspeed && tick_guess > mmgmpc_all(FL, BL, FR, BR, mmend))
        	dynspeed = (speed / 3);
        if(is_black(analog(TH_LEFT)) || is_black(analog(TH_RIGHT)))
            dynspeed = (speed / 4);
    	if(is_black(analog(TH_LEFT)) && is_black(analog(TH_RIGHT))) { break;}
        if(!is_black(analog(TH_LEFT)) && !is_black(analog(TH_RIGHT))) {
        	//forward
            mav(BL, -dynspeed);
            mav(FL, -dynspeed);
            mav(FR, dynspeed);
            mav(BR, dynspeed);
        }
		if(is_black(analog(TH_LEFT)) && !is_black(analog(TH_RIGHT))) {
        	mav(BL, dynspeed);
            mav(FL, dynspeed);
            mav(FR, dynspeed);
            mav(BR, dynspeed);
        }
        if(!is_black(analog(TH_LEFT)) && is_black(analog(TH_RIGHT))) {
        	mav(BL, -dynspeed);
            mav(FL, -dynspeed);
            mav(FR, -dynspeed);
            mav(BR, -dynspeed);
        }
        msleep(20);
    }
    mav(BL, 0);
	mav(FL, 0);
    mav(FR, 0);
    mav(BR, 0);
    msleep(100);
}

/*
void straight_square_up(int speed, int dist, int active_motor) {
    cmpc(active_motor);
    int iterations = 0;
    cmpc(active_motor);
	while(abs(gmpc(active_motor)) < dist) {
        iterations = (0);
    	while(is_black(analog(0))) {
        	mav(BR, speed);
            mav(FR, speed);
            mav(BL, -speed);
            mav(FL, -speed);
            msleep(20);
        }
        msleep(20);
    }
}
*/

//Set Z {0 - 180} or Y {0 - 180}to CLAW_NO_CHANGE to not change their position
//smoothness must be from 0 to 7; 7 is highest smoothness
void claw_rotate(int Z, int Y, int smoothness) {
    //++ to avoid divide by 0 errors
    if(++smoothness > 8) return;
    //2047 = max position, on a scale of 180 though
    int initialZ = get_servo_position(ARM_Z);
    int initialY = get_servo_position(ARM_Y);
    int dZ = (Z == CLAW_NO_CHANGE) ? 0 : Z - initialZ;
    int dY = (Y == CLAW_NO_CHANGE) ? 0 : Y - initialY;
    //if Z = 90 i = 85
    for(int i = 0; i < smoothness; i++) {
        printf("%d %d\n", initialZ + (dZ / smoothness) * i, initialY + (dY / smoothness) * i);
        set_servo_position(ARM_Z, initialZ + (dZ / smoothness) * i);
        set_servo_position(ARM_Y, initialY + (dY / smoothness) * i);
        msleep(100);
    }
    set_servo_position(ARM_Z, Z);
    set_servo_position(ARM_Y, Y);
    
}

void claw_power(int power) {
	set_servo_position(CLAW_SERVO, power);
    msleep(500);
}

int main() {
    //Get the pink cup
    puts("\t[+]Getting cup1...");
    enable_servos();
    double bias;
    claw_power(CLAW_OPEN);
    calibrate_gyro_axis(&bias, 25, gyro_z);
    int ticks = 2100;
    printf("driving\n");
    straight_square_up(1500, 1000);
    printf("rotating\n");
	claw_rotate(455, 800, 2);
    printf("opening claw\n");
    ticks = 900;
    mmdrive_with_gyro(gyro_z, 750, mmticks_reached, (void*)&ticks, bias);
    claw_power(CLAW_CLOSED);
    puts("\t[+]Retrieved cup1. Moving to beverage station...");
    //Bring pink cup back
    printf("claw rotating2\n");
    claw_rotate(455, 870, 0);
    printf("square up 2\n");
    straight_square_up(-1500, 400);
    calibrate_gyro_axis(&bias, 15, gyro_z);
    ticks = 4050;
    printf("driving3\n");
    msleep(50);
    mmdrive_with_gyro(gyro_z, -1500, mmticks_reached, (void*)&ticks, bias);
    claw_rotate(0, 1400, 0);
    ticks = 600;
    msleep(100);
    mm_sidedrive(MM_SIDEDRIVE_RIGHT, 1500, &mmticks_reached, &ticks);
    mmdrive_rotate_deg(92);
    ticks = 1215;
    calibrate_gyro_axis(&bias, 15, gyro_z);
    mmdrive_with_gyro(gyro_z, 750, &mmticks_reached, (void*)&ticks, bias);
    puts("Putting cup down");
    claw_rotate(115, 775, 2);
    msleep(200);
    claw_power(1300);
    msleep(30);
    claw_power(CLAW_OPEN);
    puts("\t[+]Done with cup1. Moving to cup2...");
    puts("ram squaring up");
    claw_rotate(115, 2047, 5);
    mmdrive_rotate_deg(-2);
    ticks = 1000;
    mmdrive_with_gyro(gyro_z, -1500, &mmticks_reached, (void*)&ticks, bias);
    claw_rotate(0, 1400, 2);
    mmdrive(1500, 500);
    mmdrive(500, 500);
    msleep(200);
    mmdrive(-400, 210);
    ticks = 200;
    mm_sidedrive(MM_SIDEDRIVE_LEFT, 1500, &mmticks_reached, (void*)&ticks);
    mmdrive_rotate_deg(-90);
    puts("line squaring up");
    ticks = 2000;
    mmdrive_with_gyro(gyro_z, 1500, &mmticks_reached, (void*)&ticks, bias);
    claw_rotate(300, 800, 3);
    straight_square_up(500, 500);
	ticks = 1500;
    mmdrive_with_gyro(gyro_z, 750, &mmticks_reached, (void*)&ticks, bias);
    claw_power(1300);
    claw_rotate(300, 870, 1);
    ticks = 1100;
    mmdrive_with_gyro(gyro_z, -750, &mmticks_reached, (void*)&ticks, bias);
    ticks = 1600;
    mm_sidedrive(MM_SIDEDRIVE_RIGHT, 1500, &mmticks_reached, (void*)&ticks);
}
