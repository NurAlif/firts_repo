/*
    PROJECT A
    
    SEDITIOUS 3-DIMENSIONAL DRIFTING OBJECT
    
    DATE STARTED : MAY 6 2017
  
 */
 
/******************************* Includes *****************************/

#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <pthread.h>

#include    "appweb.h"
#include "MotionSensor.h"



/******************************* Defines ******************************/

unsigned int throttle;

float throttleSpeed, throttleMin, turnSpeed;
float rori, role, right, left, ward, back;

const double pidInterval = 0.01;

int m1,m2,ma,mb;


/********************************* Code *******************************/

static void myaction(HttpConn *conn)
{
    HttpQueue   *q;

    q = conn->writeq;

    httpSetStatus(conn, 200);

    httpAddHeaderString(conn, "Content-Type", "text/html");
    httpAddHeaderString(conn, "Access-Control-Allow-Origin", "http://localhost");
    httpSetHeaderString(conn, "Cache-Control", "no-cache");

    if(httpGetParam(conn, "asc", "-")){throttle += throttleSpeed;}
    else if(httpGetParam(conn, "des", "-")){throttle -= throttleSpeed;}
    else if(httpGetParam(conn, "drp", "-")){throttle = throttleMin;}

    if(httpGetParam(conn, "rori", "-")){rori=turnSpeed;}else{rori=0;}
    if(httpGetParam(conn, "role", "-")){role=turnSpeed;}else{role=0;}
    if(httpGetParam(conn, "right", "-")){right=turnSpeed;}else{right=0;}
    if(httpGetParam(conn, "left", "-")){left=turnSpeed;}else{left=0;}
    if(httpGetParam(conn, "ward", "-")){ward=turnSpeed;}else{ward=0;}
    if(httpGetParam(conn, "back", "-")){back=turnSpeed;
			httpWrite(q, "%lf__%lf__%lf__%d__%d__%d__%d", ypr[ROLL], ypr[PITCH], ypr[YAW], m1, m2, ma, mb);
	}else{back=0;}

    httpFinalize(conn);

#if POSSIBLE

    httpRedirect(conn, 302, "/other-uri");
    httpError(conn, 409, "My message : %d", 5);
#endif
}

int main(int argc, char **argv, char **envp)
{
	ms_open();
	// INNER DEFINES
	double last_x, last_y, last_z;

	double vx, vy, vz;

	double setpointX = 0;
	double setpointY = 0;
	double setpointZ = 0;
	
	double prevErrorX, prevErrorY, prevErrorZ;
	double integralX, integralY, integralZ;
	
	const float pidInterval = 0.1;
	const float pidLimit = 20;
	const float tilt = 3;

	const float p_gain = 0.6;
	const float i_gain = 0.01;
	const float d_gain = 10.0;
	
	
	// ANGLE CORRECTION
	
	double mid(double val, float limit){
		if(val < -limit){
			return -limit;
		}else if(val > limit){
			return limit;}
		else{return val;}
	}
	
	double pid(float present, double past, float target, double prevError, double prevIntergral){
		double error = present - target;
		double derivative = (error - prevError) * d_gain;
		prevError = error;

		double integral = prevIntergral + (error * i_gain);
		prevIntergral = integral;

		return mid((error * p_gain) + integral + derivative, pidLimit);
	}
	
	pthread_t inc_x_thread;
	
	void *inc_x(void *x_void_ptr)
	{
		
		while (1) {
			ms_update();
			
			vx = pid(ypr[ROLL], last_x, setpointX, prevErrorX, integralX);
			vy = pid(ypr[PITCH], last_y, setpointY, prevErrorY, integralY);
			vz = pid(ypr[YAW], last_z, setpointZ, prevErrorZ, integralZ);
			
			m1 = throttle;
			m2 = throttle;
			ma = throttle;
			mb = throttle;

			m1 -= vx;
			m2 -= vx;
			ma += vx;
			mb += vx;
			
			mb -= vy;
			m2 -= vy;
			ma += vy;
			m1 += vy;
			
			m2 -= vz;
			ma -= vz;
			m1 += vz;
			mb += vz;

			if(m1 < 1000){
				m1 = 1000;}
			if(m2 < 1000){
				m2 = 1000;}
			if(ma < 1000){
				ma = 1000;}
			if(mb < 1000){
				mb = 1000;}
			
			
			
			sleep(pidInterval);
		}
		
		return NULL;

	}
	
	
	// START THREAD
	
	int x = 0;
	if(pthread_create(&inc_x_thread, NULL, inc_x, &x)) {

		fprintf(stderr, "Error creating thread\n");
		return 1;

	}
	
	
    Mpr         *mpr;
    int         rc;

    rc = MPR_ERR_CANT_CREATE;
    if ((mpr = mprCreate(0, NULL, MPR_USER_EVENTS_THREAD)) == 0) {
        mprError("Cannot create runtime");
        return -1;
    }
    if (httpCreate(HTTP_CLIENT_SIDE | HTTP_SERVER_SIDE) == 0) {
        mprError("Cannot create the HTTP services");
        return -1;
    }
    mprStart();

    if (maParseConfig("appweb.conf") < 0) {
        mprError("Cannot parse the config file %s", "appweb.conf");
        return -1;
    }
    httpDefineAction("/action/myaction", myaction);

    if (httpStartEndpoints() < 0) {
        mprError("Cannot start the web server");
        return -1;
    }
    mprServiceEvents(-1, 0);
    mprDestroy();
    return 0;
}
