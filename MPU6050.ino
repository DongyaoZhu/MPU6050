#include <Wire.h>
#include <Math.h>
#include <Kalman.h>

float error[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const float radTOdeg = 57.295779513;
unsigned long lastTime = 0;
Kalman kalmanRoll;
Kalman kalmanPitch;

void startMPU6050(){
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

//set acceleration ratio: a = 0x1C, 2g, 4g, 8g, 16g - 0 1 2 3
//set gyro ratio ratio: a = 0x1B, 250d/s, 500, 1000, 2000 - 0 1 2 3
void setRatio(int a, int n){
    Wire.beginTransmission(0x68);
	Wire.write(a);
	Wire.requestFrom(0x68, 1, true);//get old one
	unsigned char conf = Wire.read();
	conf &= 0xE7;
    conf |= (n << 3);
	Wire.write(conf);	
    Wire.endTransmission(true);
}

float getTemperature(float raw){
    return raw / 340 + 36.53;
}

/*
0x3B: x axis acceleration(3B is MS byte, 3C is LS byte)
0x3D: y axis acceleration
0x3F: z axis acceleration
0x41: temperature
0x43: x axis gyro
0x45: y axis gyro
0x47: z axis gyro
*/
void readAccGyro(float * values){
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.requestFrom(0x68, 14, true);
    for(int i = 0; i < 7; ++i){
        values[i] = Wire.read() << 8 | Wire.read();
        //values[i] |= 0xFFFC;//don't need that accurate
    }
    Wire.endTransmission(true);
}

void calibrate(){
    float readVal[7];
    for(int i = 0; i < 100; i++){
        readAccGyro(readVal);
        for(int j = 0; j < 3; j++){//ignore temperature
            error[j] += readVal[j] / 100;//acceleration
            error[j + 4] += readVal[j + 4] / 100;//angular speed
        }
    }
    //when still, z axis is 1g + some error
    error[2] -= 16383.5;
}

void correctAndConvert(float * values){
    for(int i = 0; i < 3; i++){
        //acceleration
        //note: when still, z axis = +1g; when free fall, z axis = -1g
        //when accelerating to right, ball is on left(suppose +x), x axis < 0
        values[i] -= error[i];
        //if(values[i] >= 0) values[i] += 1;
        values[i] /= 16384;//ratio is 2g
        //angular speed
        values[i + 4] -= error[i + 4];
        //if(values[i + 4] >= 0) values[i + 4] += 1;
        values[i + 4] /= 131.072;//ratio is 250d/s
    }
    //temperature
    values[3] = values[3] / 340 + 36.53;
}

void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
	Serial.begin(9600);
	Wire.begin();
    startMPU6050();
    setRatio(0x1C, 0);
    setRatio(0x1B, 0);
    calibrate();
}

void getRollAndPitch(float * values, float * rp){
    float normYZ = sqrt(values[1] * values[1] + values[2] * values[2]);
    float normXZ = sqrt(values[0] * values[0] + values[2] * values[2]);
    //pitch and roll
    rp[0] = atan(values[0] / normYZ) * radTOdeg;
    rp[1] = atan(values[1] / normXZ) * radTOdeg;
}

void rawOut(float * values){
    Serial.println("temperature in C: ");
    Serial.println(values[3]);
    Serial.println("\nacceleration in g: ");
    char axis[] = {'x', 'y', 'z'};
    for(int i = 0; i < 3; ++i){
        Serial.print(axis[i]);
        Serial.print(" ");
        Serial.println(values[i]);
    }
    Serial.println("\nangular speed in d/s: ");
    for(int i = 4; i < 7; ++i){
        Serial.print(axis[i - 4]);
        Serial.print(" ");
        Serial.println(values[i]);
    }
}

void loop(){
    float values[7];
    readAccGyro(values);
    correctAndConvert(values);
    //rawOut(values);
    float rowPitch[2];
    getRollAndPitch(values, rowPitch);
    unsigned long currentTime = micros();
    float dt = (double)(currentTime - lastTime) / 1000000.0;

        //rowPitch[i] = kalman.getAngle(rowPitch[i], values[4+i], dt);
        
        rowPitch[0] = kalmanRoll.getAngle
            (values[4], rowPitch[0], dt);
        rowPitch[1] = kalmanPitch.getAngle
            (values[5], rowPitch[1], dt);
            

    lastTime = currentTime;

    Serial.print("roll: ");
    Serial.print(rowPitch[0]);
    Serial.print(" --- pitch: ");
    Serial.println(rowPitch[1]);
    

    //delay(2000);
}








