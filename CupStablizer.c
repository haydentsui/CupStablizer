#pragma config(Sensor, S1,HTAC,sensorI2CCustom)
#include "drivers/hitechnic-accelerometer.h"


//Nick and Hayden
float speedLR(float ang, float & angP, float & lastTime, float & dAngPrev, float & sumErrorLR) //Big  axis
{
	 float dAng, now = time10[T1]/100.0;

	 if (now != lastTime)	//Prevents nul values
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
	 return 0.85*ang + 0.019*dAng + 0.1*sumErrorLR;		//Return Speed value from PID
}

//Adriano and Arun
float speedFB(float ang, float & angP, float & lastTime, float & dAngPrev) //Integral not needed, no const error
{
	 float dAng, now = time10[T1]/100.0;
   if(now != lastTime)						//Prevents null value
   {
     dAng = (ang - angP)/(now - lastTime);			 //Calculate Derivative

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
   return  0.00975*dAng + 1.05*ang; //Returns PD values
}

//Hayden
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
    float y = _y_axis;																		//
    float z = _z_axis;																		//

    float sqt = sqrt((x*x)+(y*y)+(z*z));

    angleFB = 180.0/PI*acos(_x_axis/sqt)-90;							//Angle cosines
    angleLR = 180.0/PI*acos(_y_axis/sqt)-90;							//Change angle so zero is desired angle
}

//Arun
int grabCup() // function that clamps the cup
{
	while (SensorValue[S2] == 0)	//Wait for button press
			wait10Msec(100);					//To prevent user error

	nMotorEncoder[motorC] = 0;
	motor[motorC] = -15;

	while (SensorValue[S2] == 1){}

	motor[motorC] = 0;

	return nMotorEncoder[motorC];	//Will be sent to release cup program

}

//Nick
void release(int distClamp) 	//function that releases the cup back to its original value
{
	nMotorEncoder[motorC] = 0;
	motor[motorC] = 10;
	while (nMotorEncoder[motorC] <= abs(distClamp)){}
	motor[motorC] = 0;
}

//Adriano
int start()
{
	int ultDist = 30;
	bool cond = true;

	while(SensorValue[S4] > ultDist)	//No cup in holder
	{
		nxtDisplayCenteredBigTextLine(0, "Cup Bal");
  	nxtDisplayCenteredTextLine(2, "PLEASE PLACE");
  	nxtDisplayCenteredTextLine(3, "PLACE CUP");
  	nxtDisplayCenteredTextLine(4, "IN HOLDER");
  	nxtDisplayCenteredTextLine(7, "Good Cup/Bad Cup");
		cond = true;
		while(SensorValue[S2] == 0 && SensorValue[S4] < ultDist)	//Cup in holder and no button press
		{
			if(cond == true)
			{
				eraseDisplay();
				cond = false;		//To stop screen flashing
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

//All
task main()
{
	SensorType[S4] = sensorSONAR;
	SensorType[S3] = sensorCOLORFULL;
	SensorType[S2] = sensorTouch;

	//LR is left right in refrence to accelerometer
	//FB is forward back in reference to accelerometer //small angle
	float lastTimeLR = 0, angleLR = 0, angleLRPrev = 0, dAngPrevLR = 0, sumErrorLR = 0,
				lastTimeFB = 0, angleFB = 0, angleFBPrev = 0, dAngPrevFB = 0;


	wait1Msec(200);
	int distance = start();
	nxtDisplayCenteredTextLine(2, "REMOVE");
	nxtDisplayCenteredTextLine(4, "UJOINT");
	wait1Msec(500);
	while (SensorValue[S2] != 1){}
	while (SensorValue[S2] != 0){} //Wait for push and release
	PlaySound(soundBlip);
	while(bSoundActive) EndTimeSlice(); //From sample code

	eraseDisplay();
	wait1Msec(500);
	nxtDisplayCenteredBigTextLine(3,"BALANCED");
	time10[T1] = 0;
  do
  {
		getValues(angleLR, angleFB);
		motor[motorA] = speedLR(angleLR, angleLRPrev, lastTimeLR, dAngPrevLR,sumErrorLR);
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
