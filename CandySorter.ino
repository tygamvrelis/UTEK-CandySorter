/* UTEK 2018 Senior Design -- Starburst candy sorter
 *  
 * Ann Yun, Tyler Gamvrelis
 */

/**** COLOR SENSOR ****/
// Sensor pins
const int S0 = 2;
const int S1 = 3;
const int S2 = 4;
const int S3 = 5;
const int OE = 6;
const int OUT = A0;
#define NUMSAMPLES 1024
enum sensorColors {C_RED, C_GREEN, C_BLUE, C_CLEAR};
enum scaling {POWER_OFF, TWO, TWENTY, FULL};
long colors[4]; // Holds color data at runtime

/**** ULTRASONIC SENSOR ****/
#include <NewPing.h>
const int trig = 7;
const int echo = 8;
unsigned int max_cm_dis = 200;
unsigned long echo_time_micro;
NewPing sonar(trig, echo, max_cm_dis);

/**** GENERAL ****/
const int NUM_CANDIES = 7;
int numRed; // Number of red candies sorted
int numYellow; // Number of yellow candies sorted
int numPink; // Number of pink candies sorted
enum candyColors {RED_CANDY, YELLOW_CANDY, PINK_CANDY};
long CANDY_COLORS[3][4] = {
  {110, 147, 49, 194},// Red candy reference color vector
  {94, 140, 41, 140},// Yellow candy reference color vector
  {122, 122, 46, 168},// Pink candy reference color vector
}; // Color vector matrix. Each row corresponds to a different candy. Columns are red, green, blue, clear
   // New sensor data acquired uses the Euclidean norm to compare acquisitions to these reference vectors
   // The most similar vector (minimum distance) is selected as the most likely color for the candy

/**** MOTORS ****/
#include <Servo.h>
Servo Servo1; // Servo 1
int enable_pwm = 11;
int in1 = 12;
int in2 = 9;





/**************** FUNCTIONS ****************/
// Color sensor
int getColor(uint8_t color){
  /* Returns the analog color data acquired from the
   * sensor.
   *  
   *  Inputs: color, the specification of red, green 
   *          blue, or clear filter to be used on the
   *          sensor (min: 0, max: 3)
   *  
   *  Outputs: int corresponding to the color
   */

  switch(color){
    case C_RED: digitalWrite(S2, LOW);
                digitalWrite(S3, LOW);
                break;
    case C_GREEN: digitalWrite(S2, LOW);
                  digitalWrite(S3, HIGH);
                  break;
    case C_BLUE: digitalWrite(S2, HIGH);
                 digitalWrite(S3, LOW);
                 break;
    case C_CLEAR: digitalWrite(S2, HIGH);
                  digitalWrite(S3, HIGH);
                  break;
  }
  
  return pulseIn(OUT, LOW);
}

void setFrequencyScaling(uint8_t scaling){
  /* Sets the frequency scaling parameter on the color sensor.
   *  
   *  Inputs: scaling, a value corresponding to the possible
   *          frequency scale settings for the sensor (min: 0, max: 3)
   *  
   *  Outputs: none
   */
   
  switch(scaling){
    case POWER_OFF: digitalWrite(S0, LOW);
                digitalWrite(S1, LOW);
                break;
    case TWO: digitalWrite(S0, LOW);
                  digitalWrite(S1, HIGH);
                  break;
    case TWENTY: digitalWrite(S0, HIGH);
                 digitalWrite(S1, LOW);
                 break;
    case FULL: digitalWrite(S0, HIGH);
                  digitalWrite(S1, HIGH);
                  break;
  }
}

void sampleColors(void){
  /* Samples data from color sensor
   *  
   *  Inputs: none
   *  
   *  Outputs: none
   */

    for(int i = 0; i < 4; i++){
      colors[i]= 0;
    }
   
    for(int i = 0; i < NUMSAMPLES; i++){
      colors[0] += getColor(C_RED);
      colors[1] += getColor(C_GREEN);
      colors[2] += getColor(C_BLUE);
      colors[3] += getColor(C_CLEAR);
    }
    colors[0] /= NUMSAMPLES;
    colors[1] /= NUMSAMPLES;
    colors[2] /= NUMSAMPLES;
    colors[3] /= NUMSAMPLES;
}

void calibrateColors(void){
  /* Prints colors data to the screen for each candy.
   * Prompts users for input.
   * 
   * Inputs: none
   * 
   * Outputs: none
   */

    int dummy  = 0;
    
    Serial.println("Please align with RED candy. Press enter to continue.");
    while(Serial.available() == 0);
    dummy = Serial.read();
    sampleColors();
    Serial.print("--> RED CANDY: ");
    Serial.print(" | AVERAGES | ");
    Serial.print("Red: ");
    Serial.print(colors[0]);
    Serial.print(" | Green: ");
    Serial.print(colors[1]);
    Serial.print(" | Blue: ");
    Serial.print(colors[2]);
    Serial.print(" | Clear: ");
    Serial.println(colors[3]);

    Serial.println("Please align with YELLOW candy. Press enter to continue.");
    while(Serial.available() == 0);
    dummy = Serial.read();
    sampleColors();
    Serial.print("--> YELLOW CANDY: ");
    Serial.print(" | AVERAGES | ");
    Serial.print("Red: ");
    Serial.print(colors[0]);
    Serial.print(" | Green: ");
    Serial.print(colors[1]);
    Serial.print(" | Blue: ");
    Serial.print(colors[2]);
    Serial.print(" | Clear: ");
    Serial.println(colors[3]);

    Serial.println("Please align with PINK candy. Press enter to continue.");
    while(Serial.available() == 0);
    dummy = Serial.read();
    sampleColors();
    Serial.print("--> PINK CANDY: ");
    Serial.print(" | AVERAGES | ");
    Serial.print("Red: ");
    Serial.print(colors[0]);
    Serial.print(" | Green: ");
    Serial.print(colors[1]);
    Serial.print(" | Blue: ");
    Serial.print(colors[2]);
    Serial.print(" | Clear: ");
    Serial.println(colors[3]);
}

long norm(long* v1, long* v2, int size){
  /* Euclidean distance between v1 and v2.
   *  
   *  Inputs: v1, v2 (vectors), size of vector
   *  
   *  Output: sqrt(sum((v1_i - v2_i)^2))
   */
   
  long sum = 0;
  for(int i = 0; i < size; i++){
    sum += (v1[i] - v2[i])*(v1[i] - v2[i]);
  }
  return sqrt(sum);
}

int detectColor(void){
  /* Uses the color sensor to communicate light information.
   * 
   * Acquires data over NUMSAMPLES samples and uses the Euclidean
   * distance for comparing data.
   * 
   * Inputs: none
   * 
   * Outputs: int corresponding to color (RED_CANDY, YELLOW_CANDY, PINK_CANDY);
   */
   
   sampleColors();
   long arr[3]; // arr[0] = red, arr[1] = yellow, arr[2] = pink;
   arr[0] = norm(colors, CANDY_COLORS[RED_CANDY], 4);
   arr[1] = norm(colors, CANDY_COLORS[YELLOW_CANDY], 4);
   arr[2] = norm(colors, CANDY_COLORS[PINK_CANDY], 4);

   Serial.print("Red: ");
   Serial.print(arr[0]);
   Serial.print("Yellow: ");
   Serial.print(arr[1]);
   Serial.print("Pink: ");
   Serial.println(arr[2]);
   
   // Search array for smallest element
   int index_min = -1;
   long min = 2147483647;
   for(int i = 0; i < 3; i++){
    if(arr[i] < min){
      index_min = i;
      min = arr[i];
    }
   }
  
   return index_min;
}

// Ultrasonic sensor
unsigned int getDistance(void){
  /* Gets the distance from the ultrasonic sensor.
   *  
   *  Inputs: none
   *  
   *  Output: unsigned in equal to the distance sensed 
   *          by the ultrasonic sensor, in cm
   */
  
  return sonar.convert_cm((unsigned int)(echo_time_micro));
}

void setup() {
  /* Color sensor configuration */
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OE, OUTPUT);
  digitalWrite(OE, LOW);
  pinMode(OUT, INPUT);
  setFrequencyScaling(TWENTY);

  /* Ultrasonic sensor configuration */
  echo_time_micro = sonar.ping_median (5);

  /* Open serial port with baud rate = 9600. */
  Serial.begin(9600, SERIAL_8N1);

  /* Servo motor configuration */
  Servo1.attach(9); // Attach servo to pin 9

  /* DC motor configuration */
  pinMode (enable_pwm, OUTPUT);
  pinMode (in1, OUTPUT);
  pinMode (in2, OUTPUT);
}

void loop() {
#if 0 // set this to 1 to enter the color calibration loop (generate reference vectors for candies)
  while(1){
    calibrateColors();
  }
#endif

  numRed = 0;
  numYellow = 0;
  numPink = 0;

  int distance_init = getDistance();
  do{
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH); // Move the robot
  }while(distance_init - getDistance() < 12);
  
  int colorGuess = -1;
  for(int i = 0; i < NUM_CANDIES; i++){
    analogWrite(enable_pwm, 0);
    colorGuess = detectColor();
    switch(colorGuess){
      case RED_CANDY: numRed += 1;
                      break;
      case YELLOW_CANDY: numYellow += 1;
                         break;
      case PINK_CANDY: numPink += 1;
                       break;
    }
    distance_init = getDistance();
    do{
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH); // Move the robot
    }while(distance_init - getDistance() < 12);
  }

  int pos = 0;
  while(1){
    for (pos = 0; pos <= 180; pos += 10) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      Servo1.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    for (pos = 180; pos >= 0; pos -= 10) { // goes from 180 degrees to 0 degrees
      Servo1.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
  }
}

/* UART receive interrupt triggers the sending of our classification logs. */
void serialEvent() {
  Serial.print("Red: ");
  Serial.print(numRed);
  Serial.print("Yellow: ");
  Serial.print(numYellow);
  Serial.print("Pink: ");
  Serial.print(numPink);
}
