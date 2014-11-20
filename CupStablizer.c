#pragma config(Sensor, S1,HTAC,sensorI2CCustom)
#include "drivers/hitechnic-accelerometer.h"

float speedLR(float ang, float & angP, float & lastTime, float & dAngPrev, float & sumErrorLR) //Big angle
{
	//Calculate Derivative value
	 float dAng;
	 float now = time10[T1]/100.0;
	 if (now != lastTime)						//Prevents nul values
	 	{
	 		dAng = (ang - angP)/(now - lastTime);
	 		sumErrorLR += (ang + angP)*(now - lastTime)/2.0;
	 		//Save for next call
		 	angP = ang;
		 	lastTime = now;
	   	dAngPrev = dAng;
	   }
	 else
	 {
	 		dAng = dAngPrev;
	 		ang = angP;
	 	}


	 //Display
	 float p = 0.85*ang; //0.75 and 0.01
	 float d = 0.019*dAng;
	 float in = 0.1*sumErrorLR; //0.1
	 //nxtDisplayString(0, "LR");
	 //nxtDisplayString(1, "P=%.2f",p);
	 //nxtDisplayString(2, "D=%.2f",d);
	 //nxtDisplayString(3, "T=%.2f",now);
	 //since balancing to 0 ang is error value
	 return p+d+in;
}

float speedFB(float ang, float & angP, float & lastTime, float & dAngPrev) //Small angle
{
	 //Calculate Derivative
	 float dAng;
   float now = time10[T1]/100.0;
   if(now != lastTime)						//Prevents null value
   {
     dAng = (ang - angP)/(now - lastTime);
     //Save Previous
     dAngPrev = dAng;
	   angP = ang;
	   lastTime = now;
   }
   else
   {
     //Set returns to previous
     dAng = dAngPrev;
     ang = angP;
   }
   float d = 0.00975*dAng;
   float p = 1.05*ang;
   //Display
   //nxtDisplayString(4, "FB");
   //nxtDisplayString(5, "P=%.2f",p);
   //nxtDisplayString(6, "D=%.2f",d);
   //nxtDisplayString(7, "T=%.2f",now);
   return  p + d;
}

void getValues(float & angleLR,float & angleFB)
{
	int _x_axis = 0;
  int _y_axis = 0;
  int _z_axis = 0;
	if (!HTACreadAllAxes(HTAC,  _x_axis, _y_axis, _z_axis))	//Taken from sample programs from here till
	{																												//
    nxtDisplayTextLine(4, "ERROR!!");											//
    wait1Msec(2000);																			//
    StopAllTasks();																				//here
  }
	  float x = _x_axis;																		//RobotC being weird
    float y = _y_axis;
    float z = _z_axis;
    float sqt = sqrt((x*x)+(y*y)+(z*z));

    angleFB = 180.0/PI*acos(_x_axis/sqt)-90;							//Angle cosines
    angleLR = 180.0/PI*acos(_y_axis/sqt)-90;							//Change angle so zero is desired angle
}

int grabCup() { // function that clamps the cup

	while (SensorValue[S2] == 0){
	wait10Msec(100);
}
	nMotorEncoder[motorC] = 0;
	motor[motorC] = -15;

	while (SensorValue[S2] == 1){}

	motor[motorC] = 0;

	return nMotorEncoder[motorC];

}

void release(int distClamp) { //function that releases the cup back to its original value

//	nxtDisplayString(0,"%d",distClamp);
	nMotorEncoder[motorC] = 0;
	motor[motorC] = 10;
	while (nMotorEncoder[motorC] <= abs(distClamp)){}
	//	nxtDisplayString(1,"%d",nMotorEncoder[motorC]);}
	motor[motorC] = 0;
}

int start()
{
	int ultDist = 30;
	bool cond = true;
//	while(true){
	//	nxtDisplayString(0, "Light %d ",SensorValue[S3]);
//nxtDisplayString(2, "Ultra %d ",SensorValue[S4]);}
	while(SensorValue[S4] > ultDist)
	{
	nxtDisplayCenteredBigTextLine(0, "Cup Bal");
  	nxtDisplayCenteredTextLine(2, "PLEASE PLACE");
  	nxtDisplayCenteredTextLine(3, "PLACE CUP");
  	nxtDisplayCenteredTextLine(4, "IN HOLDER");
  	nxtDisplayCenteredTextLine(7, "Good Cup/Bad Cup");
		cond = true;
		while(SensorValue[S2] == 0 && SensorValue[S4] < ultDist)
		{
			if(cond == true)
			{
				eraseDisplay();
				cond = false;
			}
			PlaySound(soundDownwardTones);
			while(bSoundActive) EndTimeSlice();
			nxtDisplayCenteredTextLine(0, "Cup Stablizer");
	  	nxtDisplayCenteredTextLine(2, "HOLD BUTTON");
	  	nxtDisplayCenteredTextLine(3, "TILL CLAMPS");
	  	nxtDisplayCenteredTextLine(4, "HOLD CUP TIGHTLY");
		}

	}
	return grabCup();

}

task main()
{
	SensorType[S4] = sensorSONAR;
	SensorType[S3] = sensorCOLORFULL;
	SensorType[S2] = sensorTouch;

	wait1Msec(200);
	int distance = start();
	nxtDisplayCenteredTextLine(2, "REMOVE");
	nxtDisplayCenteredTextLine(4, "UJOINT");
	wait1Msec(500);
	while (SensorValue[S2] != 1){}
	while (SensorValue[S2] != 0){}
	PlaySound(soundBlip);
	while(bSoundActive) EndTimeSlice();
	//Initialising variables
	float lastTimeLR = 0;
	float lastTimeFB = 0;
	float angleLR = 0;		//LR is left right in refrence to accelerometer 	//Large angle
	float angleLRPrev = 0;
	float dAngPrevLR = 0;
	float angleFB = 0;		//FB is forward back in reference to accelerometer //small angle
	float angleFBPrev = 0;
	float dAngPrevFB = 0;
	float sumErrorLR = 0;
	//Done variable Initialisation

	eraseDisplay();
	nxtDisplayCenteredBigTextLine(3,"BALANCED");
	time10[T1] = 0;
	wait1Msec(500);
  do//Keep looping //will be while(sensorValue[touch location] == 0){
  {
		getValues(angleLR, angleFB);
		motor[motorA] = speedLR(angleLR, angleLRPrev, lastTimeLR, dAngPrevLR,sumErrorLR); //Change direction to outside speed if does not work
		motor[motorB] = speedFB(angleFB, angleFBPrev, lastTimeFB, dAngPrevFB);
	}while(SensorValue[S2] == 0);
	motor[motorA] = 0;
	motor[motorB] = 0;
		eraseDisplay();

	nxtDisplayCenteredBigTextLine(2, "ADD");
	nxtDisplayCenteredBigTextLine(4, "UJOINT");
	nxtDisplayCenteredTextLine(7, "Press to release");

	PlaySound(soundBlip);
	while(bSoundActive) EndTimeSlice();

	wait1Msec(500);
	while (SensorValue[S2] != 1){}
	while (SensorValue[S2] != 0){}
	eraseDisplay();
	nxtDisplayCenteredBigTextLine(2, "RELEASE");
	nxtDisplayCenteredBigTextLine(4, "U-JOINT");
	release(distance);
	PlaySound(soundDownwardTones);
	while(bSoundActive) EndTimeSlice();
	eraseDisplay();
	nxtDisplayCenteredBigTextLine(2, "HAVE");
	nxtDisplayCenteredBigTextLine(4, "A GOOD");
	nxtDisplayCenteredBigTextLine(6, "DAY");
	wait10Msec(500);
}
