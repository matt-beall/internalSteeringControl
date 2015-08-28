
const float adc12BitToVolts = 3.3 / 4096.0;
const float voltsTo12Bit = 4096.0 / 3.3;

#define DAC_PIN A14
#define DIRECTION_SELECT_PIN 14
#define STEERING_POSITION_PIN A1
#define DESIRED_POSITION_PIN A2

#define LED_PIN 13

int dacVal;
float dacVolts;

//1 for right, 0 for left
bool steeringDirection;

int steeringPositionVal;
float steeringPositionVolts;

int desiredPositionVal;
float desiredPositionVolts;

float integralError;

void clamp(float minimum, float maximum, float& value)
{
    value = max(value, minimum);
    value = min(value, maximum);
}

void readValues()
{
    steeringPositionVal = analogRead(STEERING_POSITION_PIN);
    steeringPositionVolts = steeringPositionVal * adc12BitToVolts;
    //flip because analog sensor is inverse of desired logic
    steeringPositionVolts = 3.3 - steeringPositionVolts;
    
    desiredPositionVal = analogRead(DESIRED_POSITION_PIN);
    desiredPositionVolts = desiredPositionVal * adc12BitToVolts;
}

void steeringControl()
{
//  float scaleFactorP = 1.5;
  float gainP = 0.5;
  float error = desiredPositionVolts - steeringPositionVolts;

//  float scaleFactorI = 0.1 * scaleFactorP;
  float gainI = 0.1*gainP;
  
  integralError += error;

  clamp(-0.2, 0.2, integralError);

  float steeringEffortI = integralError * gainI;

  float steeringEffortP = error * gainP;

  float steeringEffortVolts = steeringEffortP + steeringEffortI;

  clamp(-0.75, 0.75, steeringEffortVolts);

  steeringDirection = steeringEffortVolts > 0;

  dacVolts = abs(steeringEffortVolts);
  dacVal = dacVolts * voltsTo12Bit;
}

void writeValues()
{
  digitalWrite(DIRECTION_SELECT_PIN, steeringDirection);
  analogWrite(DAC_PIN, dacVal);
}

void setup() {
  // put your setup code here, to run once:
  analogReadResolution(12);
  analogWriteResolution(12);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  pinMode(DAC_PIN, OUTPUT);
  analogWrite(DAC_PIN, 0);

  pinMode(DIRECTION_SELECT_PIN, OUTPUT);
  digitalWrite(DIRECTION_SELECT_PIN, HIGH);

  integralError = 0;
  delay(3000);
  digitalWrite(LED_PIN, LOW);
}

void loop() {

  readValues();

  steeringControl();

  writeValues();

}
