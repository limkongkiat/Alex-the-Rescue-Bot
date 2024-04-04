#define S0 22
#define S1 23
#define S2 24
#define S3 25
#define sensorOut A8
 
// Variables for Color Pulse Width Measurements

double white = 0;
double red = 0;
double green = 0;
double blue = 0;
 
void setup() {
 
  // Set S0 - S3 as outputs
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // Set Sensor output as input
  pinMode(sensorOut, INPUT);
  
  // Set Pulse Width scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  // Setup Serial Monitor
  Serial.begin(9600);
}

void identifyColour(int Red, int Green, int Blue)
{
  if (Red < 100) //see what values it gives, not sure about this
  {
    Serial.println("Red");
  } else {
    Serial.println("Green");
  }
}
 
void loop() {
 //read white values 
 //clear filter
 digitalWrite(S2,HIGH);
 digitalWrite(S3,LOW);
 white = (double)pulseIn(sensorOut,LOW);
 delay(20);
 
  // Read Red Pulse Width
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  red = (white/(double)pulseIn(sensorOut,LOW));
  // Delay to stabilize sensor
  delay(20);
  
  // Read Green Pulse Width
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  greenPW = (white/(double)pulseIn(sensorOut,LOW));
  // Delay to stabilize sensor
  delay(20);
  
  // Read Blue Pulse Width
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  bluePW = (white/(double)pulseIn(sensorOut,LOW));
  // Delay to stabilize sensor
  delay(20);

  Serial.print(red);
  Serial.print(green);
  Serial.print(blue);
  
  // Print output to Serial Monitor
identifyColour(red, green, blue);
}
