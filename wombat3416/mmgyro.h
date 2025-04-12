#include <kipr/wombat.h>
#include <math.h>
#define R_MOTOR 0
#define L_MOTOR 1
#define mmend -1
#define ushort unsigned short
#define MM_SIDEDRIVE_LEFT 0
#define MM_SIDEDRIVE_RIGHT 1
//void calibrate_gyro_z(double* bias, ushort precision);
void mmdrive(int speed, int dist);
double calibrate_gyro_z();
void mmdrive_with_gyro(signed short (*axis)(void), int speed, bool (*condition)(void*), void* condition_arg, double bias);

//regular drive forward
void mmdrive(int speed, int dist) {
    cmpc(BL);
    while(abs(gmpc(BL)) < dist) {
    	mav(BL, speed);
    	mav(FL, speed);
        //-speed because right motors are installed backward
        mav(FR, -speed);
        mav(BR, -speed);
        msleep(200);
    }
    ao();
    msleep(150);
}

int doublesgn(double n) {
	if(n > 0) return 1;
    else if (n < 0) return -1;
    return 0;
}

//variadic; averages the motor positions of all inputs. Last argument should be mmend
int mmgmpc_all(int m1, ...) {
    va_list args;
    va_start(args, m1);
    if(m1 == mmend) return 0;
    int sum = abs(gmpc(m1));
    int count = 1;
    int motor;
    while ((motor = va_arg(args, int)) != mmend) {
        sum += abs(gmpc(motor));
        count++;
    }
    va_end(args);
    return sum / count;
}

//variadic; clears all motor positions. Last argument should be mmend
void mmcmpc_all(int m1, ...) {
	va_list args;
    va_start(args, m1);
    if(m1 == mmend) return;
    cmpc(m1);
    int motor;
    while ((motor = va_arg(args, int)) != mmend)
        cmpc(motor);
    va_end(args);
}

//Drive sideways using the mechanum wheels
// lr: left = 0; right = 1, condition_arg will be passed to condition(). Use SIDEDRIVE_LEFT/RIGHT for 1st arg
void mm_sidedrive(bool leftright, int speed, bool (*condition)(void*), void* condition_arg) {
    mmcmpc_all(BR, FR, BL, FL, mmend);
     while (condition(condition_arg)) {
        if(leftright == MM_SIDEDRIVE_RIGHT) {
			mav(FL, -speed);
			mav(FR, -speed); // Sideways Right
			motor(BL, speed);
        	motor(BR, speed);
            msleep(10);
        }
        if(leftright == MM_SIDEDRIVE_LEFT) {
            mav(FL, speed);
			mav(FR, speed); // Sideways Right
			motor(BL, -speed);
        	motor(BR, -speed);
            msleep(10);
        }
    }
    
}

//self-explanatory, precision is number of times gyro is queried to get the average bias
void calibrate_gyro_axis(double* bias, int precision, signed short (*axis)(void)) {
	*bias = 0;
    int i = 0;
    while(i < precision) {
    	*bias += axis();
        i++;
        msleep(5);
    }
    *bias /= (double)precision;
}

bool mmticks_reached(void* ticks) {
    int rn = mmgmpc_all(FL, BL, BR, FR, mmend);
    //printf("%d\n", rn);
	return rn < *((int*)ticks);
}

//Drive until condition is met
//condition_arg will be passed into conditional function e.g. (&mmticks_reached, 1000)
void mmdrive_with_gyro(signed short (*axis)(void), int speed, bool (*condition)(void*), void* condition_arg, double bias) {
    mmcmpc_all(BR, FR, BL, FL, mmend);
    double delta = 0, dev = 0;
    while (condition(condition_arg)) {
        delta = dev / 5;
        mav(BR, speed + delta);
        mav(FR, speed + delta);
        mav(FL, -1 * (speed + delta) );
        mav(BL, -1 * (speed + delta) );
        msleep(10);
        dev = axis() - bias;
    }
    ao();
}

//deg > 0 == clockwise
void mmdrive_rotate_deg(int deg) {
	int ticks_per_deg = 25;
    int speed = 1000;
    mmcmpc_all(BR, FR, BL, FL, mmend);
    //+40 accounts for momentum
    int pos = mmgmpc_all(BR, FR, BL, FL, mmend);
    while( pos + 40 < abs(ticks_per_deg * deg)) {
        mav(FR, -sgn(deg) * speed);
        mav(FL, -sgn(deg) * speed);
        mav(BR, -sgn(deg) * speed);
        mav(BL, -sgn(deg) * speed);
        pos = mmgmpc_all(BR, FR, BL, FL, mmend);
        msleep(15);
    }
	ao();
}
