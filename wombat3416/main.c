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
#define CLAW_CLOSED 675 // Closed position for claw servo
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
        	mav(BL, -dynspeed);
            mav(FL, -dynspeed);
            mav(FR, -dynspeed);
            mav(BR, -dynspeed);
        }
        if(!is_black(analog(TH_LEFT)) && is_black(analog(TH_RIGHT))) {
        	mav(BL, dynspeed);
            mav(FL, dynspeed);
            mav(FR, dynspeed);
            mav(BR, dynspeed);
        }
        msleep(10);
    }
    ao();
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
    msleep(150);
}

void claw_power(int power) {
	set_servo_position(CLAW_SERVO, power);
    msleep(500);
}

int main() {
    claw_power(CLAW_OPEN);
    claw_rotate(315, 800, 1);
    puts("\t[+]Getting cup1...");
    enable_servos();
    double bias;
    claw_power(CLAW_OPEN);
    calibrate_gyro_axis(&bias, 25, gyro_z);
    int ticks = 600;
    //Get the 1st cup
    mmdrive(-1500, 3000);
    mmdrive(750, 620);
    mm_sidedrive(MM_SIDEDRIVE_RIGHT, 500, &mmticks_reached, &ticks);
    mmdrive_rotate_deg(90);
    printf("driving to 1st cup\n");
    straight_square_up(1500, 2000);
    ticks = 1000;
    mm_sidedrive(MM_SIDEDRIVE_LEFT, 900, &mmticks_reached, &ticks);
    ticks = 80;
    mm_sidedrive(MM_SIDEDRIVE_RIGHT, 500, &mmticks_reached, &ticks);
    printf("rotating\n");
	claw_rotate(480, 800, 2);
    printf("opening claw\n");
    ticks = 1100;
    mmdrive_with_gyro(gyro_z, 750, mmticks_reached, (void*)&ticks, bias);
    claw_power(CLAW_CLOSED);
    puts("\t[+]Retrieved cup1. Moving to beverage station...");
    //Bring pink cup back
    printf("claw rotating2\n");
    claw_rotate(455, 870, 0);
    printf("square up 2 against the black line\n");
    straight_square_up(-1000, 400);
    calibrate_gyro_axis(&bias, 15, gyro_z);
    puts("rotate claw up with cup 1");
    ticks = 3850;
    printf("driving back to the wall\n");
    mmdrive_with_gyro(gyro_z, -1500, mmticks_reached, (void*)&ticks, bias);
    claw_rotate(115, 1700, 0);
    /*claw_rotate(0, 1400, 0);
    msleep(100);
    mmdrive(750, 400);
    mmdrive(-500, 80);
    */
    puts("Rotating to align laterally towards the beverage station");
    ticks = 600;
    mm_sidedrive(MM_SIDEDRIVE_RIGHT, 1500, &mmticks_reached, (void*)&ticks);
    calibrate_gyro_axis(&bias, 15, gyro_z);
    mmdrive_rotate_deg(90);
    puts("square up right against the pipe");
    ticks = 1100;
    mm_sidedrive(MM_SIDEDRIVE_RIGHT, 1500, &mmticks_reached, (void*)&ticks);
    ticks = 50;
    mm_sidedrive(MM_SIDEDRIVE_LEFT, 500, &mmticks_reached, (void*)&ticks);
    puts("square up bev station");
    ticks = 3800;
   	mmdrive_with_gyro(gyro_z, 1500, &mmticks_reached, (void*)&ticks, bias);
    ticks = 950;
    mmdrive_with_gyro(gyro_z, -1000, &mmticks_reached, (void*)&ticks, bias);
    puts("rotate to put cup1 in the corner of the bev station");
    claw_rotate(115, 795, 7);
    msleep(200);
    mmdrive_rotate_deg(1);
    puts("Putting cup down");
    claw_power(400);
    msleep(30);
    claw_power(CLAW_OPEN);
    puts("Disengaging from cup1 to retract the claw");
    mmdrive_rotate_deg(4);
    mmdrive(1500, 850);
    mmdrive_rotate_deg(-5);
    puts("sliding left to get away from wall to give space for turning");
    ticks = 80;
    mm_sidedrive(MM_SIDEDRIVE_LEFT, 1500, &mmticks_reached, (void*)&ticks);
    puts("\t[+]Done with cup1. Moving to cup2...");
    claw_rotate(115, 2047, 5);
    ticks = 1000;
    mmdrive_with_gyro(gyro_z, -1500, &mmticks_reached, (void*)&ticks, bias);
    claw_rotate(0, 1400, 2);
//  mmdrive(1500, 500);
    msleep(200);
    mmdrive(-400, 210);
    ticks = 200;
    mm_sidedrive(MM_SIDEDRIVE_LEFT, 1500, &mmticks_reached, (void*)&ticks);
    mmdrive_rotate_deg(-90);
    puts("line squaring up to get 2nd cup");
    ticks = 2200;
    mmdrive_with_gyro(gyro_z, 1500, &mmticks_reached, (void*)&ticks, bias);
    claw_rotate(300, 800, 3);
    straight_square_up(1500, 800);
    puts("driving forward to get 2nd cup");
	ticks = 780;
    mmdrive_with_gyro(gyro_z, 1500, &mmticks_reached, (void*)&ticks, bias);
    claw_rotate(315, 870, 1);
    claw_power(CLAW_CLOSED);
    puts("\t[+] Got cup2, driving backwards.");
    claw_rotate(315, 910, 1);
    straight_square_up(-1000, 1750);
    puts("completed backing up after picking up 2nd cup");
    ticks = 400;
    mmdrive_with_gyro(gyro_z, -1500, &mmticks_reached, (void*)&ticks, bias);
    puts("sliding right to the black line");
    int sensor = 0;
    mm_sidedrive(MM_SIDEDRIVE_RIGHT, 1500, &mmblack_reached, (void*)&sensor);
    ticks = 480;
    mm_sidedrive(MM_SIDEDRIVE_RIGHT, 1500, &mmticks_reached, (void*)&ticks);
    puts("sliding right to align with the drink");
    straight_square_up(400, 500);
    puts("Driving forward tape width");
    ticks = 500;
    mmdrive_with_gyro(gyro_z, 900, &mmticks_reached, (void*)&ticks, bias);
    straight_square_up(-300, 80);
    puts("rotating claw to 90 degrees and 80 degrees");
    puts("Depositing cup 2");
    claw_rotate(1024, 1100, 7);
    claw_rotate(1024, 1000, 2);
    msleep(300);
    claw_power(300);
    msleep(300);
    claw_power(CLAW_OPEN);
    for(int i = 0; i < 2; i++) {
    if(i > 0) {
        puts("rotating up");
		claw_rotate(1024, 2047, 0);
    }
	mmdrive(-500, 5);
    puts("rotating up");
	claw_rotate(1024, 2047, 0);
    msleep(500);
    puts("Doing half-square up on the left");
    sensor = 0;
	mm_sidedrive(MM_SIDEDRIVE_LEFT, 200, &mmblack_reached, (void*)&sensor);
    ao();
    ticks = 110;
    mm_sidedrive(MM_SIDEDRIVE_RIGHT, 500, &mmticks_reached, (void*)&ticks);
    straight_square_up(-200, 100);
    puts("slightly closing the claw");
    claw_power(400);
    puts("rotating forward");
    claw_rotate(0, 2047, 0);
    msleep(500);
    puts("Going forward into drinks");
    ticks = 490;
    mmdrive_with_gyro(gyro_z, 1500, &mmticks_reached, (void*)&ticks, bias);
    puts("rotating claw to align with the drink");
    int drink_pickup_angle = 610;
    claw_rotate(300, drink_pickup_angle, 0);
    msleep(500);
    puts("Driving forward to drink1");
    ticks = 625;
    mmdrive_with_gyro(gyro_z, 1500, &mmticks_reached, (void*)&ticks, bias);
    puts("Closing claw to get drink1");
    claw_power(1020);
    msleep(500);
    puts("Driving back w/ drink1");
    claw_rotate(300, drink_pickup_angle + 130, 0);
    straight_square_up(-750, 300);
    puts("turning claw to drop drink1 into cup2");
    claw_rotate(280, 2047, 3);
    claw_rotate(1050, 1800, 3);
    puts("turning claw down to tocuh the cup2 for stability");
    claw_rotate(1050, 1280, 6);
	msleep(5000);
    claw_power(CLAW_OPEN);
    }
    return 1;
}
