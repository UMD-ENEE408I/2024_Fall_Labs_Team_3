#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Encoder.h>
#include <Adafruit_MPU6050.h>

#include <Adafruit_Sensor.h>

// ADC (line sensor)
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

int adc1_buf[8];
int adc2_buf[8];

uint8_t lineArray[13]; 

// Encoders
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

// Motors
const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const unsigned int PWM_MAX = 255;
const int freq = 5000;
const int resolution = 8; // 8-bit resolution -> PWM values go from 0-255

// LED
const int ledChannel = 0;

void M2_stop();
void M1_stop();

Adafruit_MPU6050 mpu;

// PID
const int base_pid = 80; // Base speed for robot
const float mid = 5.33;

float e;
float d_e;
float total_e;
float prev_e;

// Assign values to the following feedback constants:
float Kp = 5;
float Kd = 5;
float Ki = .001;

/*
 *  Line sensor functions
 */
void readADC() {
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i); //read from one side
    adc2_buf[i] = adc2.readADC(i);
  }
}

void rotateNDegrees2(int degrees) {
  float angleZ = 0;
  unsigned long previousTime = millis();

  //digitalWrite(M1_IN_1, HIGH);
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, 100);
  ledcWrite(M2_IN_1_CHANNEL, 100);
  ledcWrite(M2_IN_2_CHANNEL, 0);
  while (abs(angleZ) < degrees) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - previousTime) / 1000.0;
    
    angleZ += g.gyro.z * elapsedTime * (180.0 / PI);
    
    //Serial.print("Angle: ");
    //Serial.println(angleZ);
    
    previousTime = currentTime;
    delay(10);
  }
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, 0);
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, 0);
  Serial.print(degrees);
  Serial.println("-degree rotation complete!");
}
void rotateNDegrees1(int degrees) {
  float angleZ = 0;
  unsigned long previousTime = millis();

  ledcWrite(M1_IN_1_CHANNEL, 100);
  ledcWrite(M1_IN_2_CHANNEL, 0);
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, 100);
  while (abs(angleZ) < degrees) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    float elapsedTime = (currentTime - previousTime) / 1000.0;
    
    angleZ += g.gyro.z * elapsedTime * (180.0 / PI);
    
    // Serial.print("Angle: ");
    // Serial.println(angleZ);
    
    previousTime = currentTime;
    delay(10);
  }
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, 0);
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, 0);
  Serial.print(degrees);
  Serial.println("-degree rotation complete!");
}

// Converts ADC readings to binary array lineArray[] (Check threshold for your robot) 
void digitalConvert() {
  int threshold = 700;
  for (int i = 0; i < 7; i++) {
    if (adc1_buf[i]>threshold) {
      lineArray[2*i] = 0; 
    } else {
      lineArray[2*i] = 1;
    }

    if (i<6) {
      if (adc2_buf[i]>threshold){
        lineArray[2*i+1] = 0;
      } else {
        lineArray[2*i+1] = 1;
      }
    }
  }
}

// Calculate robot's position on the line 
float getPosition(/* Arguments */) {
  int position = 6;
  /* Using lineArray[], which is an array of 13 Boolean values representing 1 
   * if the line sensor reads a white surface and 0 for a dark surface, 
   * this function returns a value between 0-12 for where the sensor thinks 
   * the center of line is (6 being the middle)
   */
  readADC();
  digitalConvert();
  int sum = 0; 
  int numBlck = 0;
  for(int i = 0; i < 12; i++) {
    if (lineArray[i] == 0) {
      sum += i;
      numBlck++;
    }
  }

  if (numBlck == 0) {
    return 0;
  } else if (numBlck >= 10) {
    return 13;
  } else {
    return (float)sum/(float)numBlck;
  }
}

/*
 *  Movement functions
 */
void M1_forward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, pwm_value);
}
void M2_forward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, pwm_value);
  delay(100);
  M1_stop();
  M2_stop();
}

void M1_backward(int pwm_value) {
  ledcWrite(M1_IN_1_CHANNEL, pwm_value);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}
void M2_backward(int pwm_value) {
  ledcWrite(M2_IN_1_CHANNEL, pwm_value);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, PWM_MAX);
  ledcWrite(M1_IN_2_CHANNEL, PWM_MAX);
}
void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, PWM_MAX);
  ledcWrite(M2_IN_2_CHANNEL, PWM_MAX);
}

bool allWhite(){
  for (int i = 0; i < 13; i++) {
    if (lineArray[i] != 0) {
      return false;
    }
  }
  return true;
}

bool allBlack() {
  for (int i = 0; i < 13; i++) {
    if (lineArray[i] != 1) {
      return false;
    }
  }
  return true;
}

int isCorner() {
  digitalConvert();
  if (allWhite()){
    Serial.print(", all white, ");
    return 3;
  } else if(allBlack()) {
    Serial.print(", all black, ");
    return 4;
  } else if (lineArray[8] == 1 && lineArray[9] == 1 && lineArray[10] == 1 && lineArray[11] == 1 && lineArray[12] == 1) {
    Serial.print(", left corner, ");
    return 1; //left corner
  } else if (lineArray[4] == 1 && lineArray[3] == 1 && lineArray[2] == 1 && lineArray[1] == 1 && lineArray[0] == 1) {
    Serial.print(", right corner, ");
    return 2; //right
  } else {
    Serial.print(", no corner, ");
    return 0; 
  }
}

void turnCorner(int direction) {
  /* 
   * Use the encoder readings to turn the robot 90 degrees clockwise or 
   * counterclockwise depending on the argument. You can calculate when the 
   * robot has turned 90 degrees using either the IMU or the encoders + wheel measurements
   */
  //direction = 1 -> turn right, direction = 0 -> turn left 
  if (direction == 1) {
    M1_forward(80);
    M2_backward(80);
    //check robot position
  } else {
    M1_backward(80);
    M2_forward(80);
  }

}

/*
 *  setup and loop
 */
void setup() {
  Serial.begin(115200);

  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);

  M1_stop();
  M2_stop();

  delay(100);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

void loop() {
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  prev_e = 0;
  while(true) {
    int u;
    int rightWheelPWM;
    int leftWheelPWM;
    float pos;

    readADC();
    digitalConvert();
    delay(1);
    pos = getPosition(/* Arguments */);
    Serial.print("pos:");
    Serial.print(pos);
    // Define the PID errors
    e = mid - pos;
    if (pos == 13) {
      e = 6;
    }
    d_e = (e - prev_e); 
    total_e= total_e + e;
    prev_e = e;
    Serial.print(", e:");
    Serial.print(e);
    Serial.print(", prev_e:");
    Serial.print(prev_e);
    Serial.print(", d_e:");
    Serial.print(d_e);
    Serial.print(", total_e:");
    Serial.print(total_e);
    Serial.print(", u:");
    Serial.print(u);
    // Implement PID control (include safeguards for when the PWM values go below 0 or exceed maximum)
    u = Kp*e + Ki*total_e + Kd*d_e;
    int basePWM = 80;
   
     
    Serial.print(", rightWheelPWM:");
    Serial.print(rightWheelPWM);
    Serial.print(", leftWheelPWM:");
    Serial.println(leftWheelPWM);
  
    
    // Check for corners
    int side = isCorner();
    Serial.print("side");
    Serial.print(side);
    if(side == 1) {
      M1_forward(150);
      M2_forward(150);
      delay(500);
      rotateNDegrees1(60);
      delay(1000);
    } else if (side == 2) {
      M1_forward(150);
      M2_forward(150);
      delay(500);
      rotateNDegrees2(60);
      delay(1000);
    } else if (side == 3) {
      //all white
      M1_forward(150);
      M2_forward(150);

    } else if(side == 4) {
      //all black 
      rotateNDegrees2(150);
      delay(1000);
    } else {
      if (basePWM -u < 0|| basePWM + u > 255 || pos == 13) {
        rightWheelPWM = 0;
        leftWheelPWM = 0;
      } else {
        rightWheelPWM = basePWM + u; //positive error
        leftWheelPWM = basePWM - u;
      }
    }
    
    M1_forward(leftWheelPWM);
    M2_forward(rightWheelPWM);
  }
}
