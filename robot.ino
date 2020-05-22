#include <SoftwareSerial.h>
#include <tuple>

using namespace std;

SoftwareSerial bluetooth(51, 50); //rx, tx

tuple<int, int> motorFL (1, 2); // Change
tuple<int, int> motorBL (3, 4); // Change
tuple<int, int> motorFR (5, 6); // Change
tuple<int, int> motorBR (7, 8); // Change
tuple<int, int> intakeMotor(9, 10); // Change
tuple<int, int> indexerMotor(11, 12); // Change
tuple<int, int> feederMotor(13, 14); // Change
tuple<int, int> shooterMotorA(15, 16); // Change
tuple<int, int> shooterMotorB(17, 18); // Change

float leftXAxis;
float leftYAxis;
float rightYAxis;
float rightXAxis;
int aButton;
int bButton;
int xButton;
int yButton;

void setup(){
	bluetooth.begin(9600);

	pinMode(get<0>(motorFL), OUTPUT);
	pinMode(get<1>(motorFL), OUTPUT);
	pinMode(get<0>(motorBL), OUTPUT);
	pinMode(get<1>(motorBL), OUTPUT);
	pinMode(get<0>(motorFR), OUTPUT);
	pinMode(get<1>(motorFR), OUTPUT);
	pinMode(get<0>(motorBR), OUTPUT);
	pinMode(get<1>(motorBR), OUTPUT);

	digitalWrite(get<0>(motorFL), LOW);
	digitalWrite(get<1>(motorFL), LOW);
	digitalWrite(get<0>(motorBL), LOW);
	digitalWrite(get<1>(motorBL), LOW);
	digitalWrite(get<0>(motorFR), LOW);
	digitalWrite(get<1>(motorFR), LOW);
	digitalWrite(get<0>(motorBR), LOW);
	digitalWrite(get<1>(motorBR), LOW);
}

void setMotor(int power, tuple motor){
	pin1 = get<0>(motor);
	pin2 = get<1>(motor);

	if (power < 0) {
		analogWrite(pin1, -power);
		digitalWrite(pin2, LOW);
	} else {
		analogWrite(pin2, power);
		digitalWrite(pin1, LOW);	
}

void drive(float xPower, float yPower) {
    xPower *= 100;
    yPower *= -100;
    float v = (100 - abs(yPower)) * (xPower/100) + xPower;
    float w = (100 - abs(xPower)) * (yPower/100) + yPower;
    float velocityL = ((((v - w) / 2) / 100) * 255);
    float velocityR = ((((v + w) / 2) / 100) * 255);

// Left motors
    setMotor(velocityL, motorFL); 
    setMotor(velocityL, motorBL);

    // Right motors
    setMotor(velocityR, motorFR);
    setMotor(velocityL, motorBR);
}

void intakeControl(float power){
    // Set intake
    // 1x N20 motor

    setMotor(power, intakeMotor);
}

void indexerControl(float power){
    // Set indexer
    // 1x TT motor for indexer

    setMotor(power, indexerMotor);
}

void shooterControl(float power){
    // Set shooter
    // 1x TT motor for feeder; 2x Micro motor for shooter
    
    setMotor(power, feederMotor);
    setMotor(power, shooterMotorA);
    setMotor(power, shooterMotorB);
}

// void turretControl(float power){
    // Set turret 
    // 1x TT motor; 1x KY-040 encoder; 1x SG90 servo for hood
// }

//void climbControl(float power){
    // Set climb
    // MG995 servo
//}

void loop() {
    while (bluetooth.available() > 0) {
        if ((bluetooth.read()) == 'z') {
            leftXAxis = bluetooth.parseFloat();
            leftYAxis = bluetooth.parseFloat();
            rightYAxis = bluetooth.parseFloat();
            rightXAxis = bluetooth.parseFloat();
            aButton = bluetooth.parseInt();
            bButton = bluetooth.parseInt();
            xButton = bluetooth.parseInt();
            yButton = bluetooth.parseInt();

//          drive(leftXAxis, -leftYAxis);
            setMotor(250, motorFR);
            // Run subsystems
            if (aButton){
               intakeControl(160);
            }

            if (bButton){
                indexerControl(100);
            }

            if (xButton){
                shooterControl(250);
            }
        }
    }
}
