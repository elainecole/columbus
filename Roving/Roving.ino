/*
 * Roving
 * 
 * BOE Shield-Bot application roams with Ping))) mounted on servo turret.
 * 
 * 2017 Elaine Cole
 * Original sturcture taken from: http://learn.parallax.com/BoeShield/RoamingPingShieldBot
 *      Created May 17th, 2012
 *      by Andy Lindsay
 *      v0.82
 * 
 */
                                                                                
#include <Servo.h>                                 // Include servo library

Servo servoLeft;                                   // Servo object instances
Servo servoRight;
Servo servoTurret;  

const int servoLeftPin = 13;                       // I/O Pin constants
const int servoRightPin = 12;
const int servoTurretPin = 11;
const int pingPin = 7;

const int msPerTurnDegree = 6;                     // For maneuvers  
const int tooCloseCm = 30;                         // For distance decisions
const int bumpedCm = 6;
int ccwLim = 1400;                                 // For turret servo control
int rtAngle = 900;
const int usTocm = 29;                             // Ping))) conversion constant

// Sequence of turret positions.
int sequence[] = {0, 2, 4, 6, 8, 10, 9, 7, 5, 3, 1};

// array of values for turret distance from obstacle @ pos 
int obstacleTracker[5] = {0, 0, 0, 0, 0};

void look(int values[]); 
void printObstacles(int values[]); 
void nextMove(int values[]);

// Declare array to store that many cm distance elements using pre-calculated
// number of elements in array.
const int elements = sizeof(sequence);
int cm[elements];

// Pre-calculate degrees per adjustment of turret servo. 
const int degreesTurret = 180/(sizeof(sequence)/sizeof(int)-1);

int i = -1;                                        // Global index
int sign = 1;                                      // Sign of increments
int theta = -degreesTurret;                        // Set up initial turret angle

void setup() { 
  
  Serial.begin(9600);                              // Open serial connection

  servoLeft.attach(servoLeftPin);                  // Attach left signal to pin 13
  servoRight.attach(servoRightPin);                // Attach right signal to pin 12
  servoTurret.attach(servoTurretPin);              // Attach turret signal to pin 7
  maneuver(0, 0, 1000);                            // Stay still for 1 second
  setLook();
}  
 
void loop() {
  
  look(obstacleTracker); 
  nextMove(obstacleTracker); 
  // every 9 samples, print out distance array 
    i++;                                             // Increment turret index
  if (i % 9 == 0) {
    printObstacles(obstacleTracker);
  }
  return;
  
  maneuver(200, 200);                              // Go full speed forward UFN
  i++;                                             // Increment turret index

  // Advance turret servo to next position in sequence and wait for it to get there.
  theta = sequence[i] * degreesTurret;       
  turret(theta);
  delay(100);                                

  int response = cmDistance();                            // Measure cm from this turret angle
  Serial.print("response is ");
  Serial.println(response); 
  Serial.print("i = ");
  Serial.println(i);
  delay(100);
  return; 
  // If object in +/- 36 degrees from center is within tooCloseCm threshold... 
  if ((sequence[i]>=3) && (sequence[i]<=7) && (cm[i] < tooCloseCm))
  {
    maneuver(0, 0);                                // Stop moving 
    int theta = findOpening();                     // Get opening (in terms of sequence element
    theta *= degreesTurret;                        // Convert sequence element to degree angle
    
    // Convert degree angle to time the BOE Shield-Bot wheels will have to turn to face
    // direction of turret.
    int turnAngleTime = (90 - theta) * msPerTurnDegree;
    
    if(turnAngleTime < 0)                          // If negative turning angle, 
    {
      maneuver(-200, 200, -turnAngleTime);         // then rotate CCW for turningAngleTime ms
    }
    else                                           // If positive turning angle, 
    {
      maneuver(200, -200, turnAngleTime);          // then rotate CW for turningAngleTime ms 
    }
    maneuver(200, 200);                            // Start going forward again
  }   

  if(i == 10)                                      // If turret at max, go back to zero 
  {
    i = -1;
  }  
}

/*
 * Control BOE Shield-Bot servo direction, speed, set and forget version.
 * Parameters: speedLeft - left servo speed
 *             speedRight - right servo speed
 *             Backward  Linear  Stop  Linear   Forward
 *             -200      -100......0......100       200
 */ 
void maneuver(int speedLeft, int speedRight)
{
  // Call maneuver with just 1 ms blocking; servos will keep going indefinitely.
  maneuver(speedLeft, speedRight, 1);              
}

/*
 * Control BOE Shield-Bot servo direction, speed and maneuver duration.   
 * Parameters: speedLeft - left servo speed
 *             speedRight - right servo speed
 *             Backward  Linear  Stop  Linear   Forward
 *             -200      -100......0......100       200
 *             msTime - time to block code execution before another maneuver
 * Source:     http://learn.parallax.com/ManeuverFunction
 */ 
void maneuver(int speedLeft, int speedRight, int msTime)
{
  servoLeft.writeMicroseconds(1500 + speedLeft);   // Set Left servo speed
  servoRight.writeMicroseconds(1500 - speedRight); // Set right servo speed
  if(msTime==-1)                                   // if msTime = -1
  {                                  
    servoLeft.detach();                            // Stop servo signals
    servoRight.detach();   
  }
  delay(msTime);                                   // Delay for msTime
}

/*
 * Position the horn of a Parallax Standard Servo
 * Parameter: degreeVal in a range from 90 to -90 degrees. 
 */ 
void turret(int degreeVal)
{ 
  servoTurret.writeMicroseconds(ccwLim - rtAngle + (degreeVal * 10));
}
/*
 * returns calculation for time (some small sec) required to adjust given degree
 */
int turretTime(int degreeVal) {
  if (degreeVal < 0) {
    degreeVal = -degreeVal; 
  }
  return ccwLim - rtAngle + (degreeVal * 10); 
}
/*
 * Get cm distance measurment from Ping Ultrasonic Distance Sensor
 * Returns: distance measurement in cm.   
 */ 
int cmDistance()
{
  int distance = 0;                                // Initialize distance to zero    
  do                                               // Loop in case of zero measurement
  {
    long us = ping(pingPin);                        // Get Ping))) microsecond measurement
    distance = convert(us, usTocm);                // Convert to cm measurement
    delay(3);                                      // Pause before retry (if needed)
  }
  while(distance == 0);                           
  return distance;                                 // Return distance measurement
}

/*
 * Converts microsecond Ping))) round trip measurement to a useful value.
 * Parameters: us - microsecond value from Ping))) echo time measurement.
 *             scalar - 29 for us to cm, or 74 for us to in.
 * Returns:    distance measurement dictated by the scalar.   
 */ 
long convert(long us, int scalar)
{
    return us / scalar / 2;                        // Echo round trip time -> cm
}

/*
 * Initiate and capture Ping))) Ultrasonic Distance Sensor's round trip echo time.
 * Parameter: pin - Digital I/O pin connected to Ping)))
 * Returns:   duration - The echo time in microseconds 
 * Source:    Ping by David A. Mellis, located in File -> Examples -> Sensors
 * To-Do:     Double check timing against datasheet
 */ 
long ping(int pin)
{
  long duration;                                   // Variables for calculating distance
  pinMode(pin, OUTPUT);                            // I/O pin -> output
  digitalWrite(pin, LOW);                          // Start low
  delayMicroseconds(2);                            // Stay low for 2 us
  digitalWrite(pin, HIGH);                         // Send 5 us high pulse
  delayMicroseconds(5);                            
  digitalWrite(pin, LOW);                          
  pinMode(pin, INPUT);                             // Set I/O pin to input 
  duration = pulseIn(pin, HIGH);            // Measure echo time pulse from Ping)))
  return duration;                                 // Return pulse duration
}    
          
/*
 * Find an opening in system's 180-degree field of distance detection.
 * Returns:    Distance measurement dictated by the scalar
 * To-do:      Clean up and modularize
 *             Incorporate constants
 */ 
int findOpening()
{
                                                    
  int Ai;                                          // Initial turret angle   
  int Af;                                          // Final turret angle
  int k = sequence[i];                             // Copy sequence[i] to local variable
  int ki = k;                                      // Second copy of current sequence[i] 
  int inc;                                         // Increment/decrement variable 
  int dt;                                          // Time increment
  int repcnt = 0;                                  // Repetitions count
  int sMin;                                        // Minimum distance measurement

  // Increment or decrement depending on where turret is pointing
  if(k <= 5)                                       
  {
    inc = 1;
  }  
  else
  {
    inc = -1;
  }

  // Rotate turret until an opening becomes visible.  If it reaches servo limit, 
  // turn back to first angle of detection and try scanning toward other servo limit.
  // If still no opening, rotate robot 90 degrees and try again.
  do
  {
    repcnt++;                                        // Increment repetition count 
    if(repcnt > ((sizeof(sequence) / sizeof(int)))*2)// If no opening after two scans
    {
      maneuver(-200, -200, 100);                     // Back up, turn, and stop to try again
      maneuver(-200, 200, 90*6);
      maneuver(0, 0, 1);
    }
    k += inc;                                        // Increment/decrement k
    if(k == -1)                                      // Change inc/dec value if limit reached
    {
      k = ki;
      inc = -inc;
      dt = 250;                                      // Turret will take longer to get there
    }
    if(k == 11)
    {
      k = ki;
      inc = -inc;
      dt = 250;
    }  
    // Look for index of next turret position
    i = findIn(k, sequence, sizeof(sequence)/sizeof(int));
    
    // Set turret to next position
    int theta = sequence[i] * degreesTurret;
    turret(theta);                                   // Position turret      
    delay(dt);                                       // Wait for it to get there
    dt = 100;                                        // Reset for small increment turret movement

    cm[i] = cmDistance();                            // Take Ping))) measurement
  }
  while(cm[i] < 30);                                 // Keep checking to edge of obstacle
  
  sMin = 1000;                                       // Initialize minimum distance to impossibly large value
  for(int t = 0; t <= 10; t++)
  {  
    if(sMin > cm[t]) sMin = cm[t];                   // Use sMin to track smallest distance
  }
  if(sMin < 6)                                       // If less than 6 cm, back up a little
  {
    maneuver(-200, -200, 350);
    k = -1;                                          // Get turret ready to start over
  }  
  maneuver(0, 0);                                    // Stay still indefinitely
  
  // Keep rotating turret until another obstacle that's under 30 cm is detected or the turret
  // reaches the servo limit.  Keep track of the maximum distance and the angle when this portion
  // of the scan started and stopped.

  Ai = sequence[i];                                  // Save initial angle when obstacle disappeared from view
 
  k = sequence[i];                                   // Make a copy of the turret position again
  
  int aMax = -2;                                     // Initialize maximum distance measurements to impossibly small values
  int cmMax = -2;

  do                                                 // Loop for scan
  {
    k += inc;                                        // Inc/dec turret position
    // Look up index for turret position
    i = findIn(k, sequence, sizeof(sequence)/sizeof(int));
    int theta = sequence[i] * degreesTurret;         // Position turret
    turret(theta);
    delay(100);
    
    cm[i] = cmDistance();                            // Measure distance

    if(cm[i]>cmMax)                                  // Keep track of max distance and angle(max distance)
    {
      cmMax = cm[i];
      aMax = sequence[i]; 
    }  
  }
  while((cm[i] > 30)&&(sequence[i]!=0)&&(sequence[i]!=10)); 
  // Keep repeating while the distance measurement > 30 cm, and the turret is not near its 
  // mechanical limits.
  
  Af = sequence[i];                                  // Record final turret position  
  int A = Ai + ((Af-Ai)/2);                          // Calculate middle of opening

  // Set turret index for straight ahead in prep for turn.  
  i = findIn(5, sequence, sizeof(sequence)/sizeof(int));
  int theta = sequence[i] * degreesTurret;           
  turret(theta);
  
  if(sMin < 7)                                       // Turn further if too close   
  {
    if (A < aMax) return aMax; else return A;
  }
  else
  {
    if (A < aMax) return A; else return aMax;
  }    
}  
 
/*
 * Finds the first instance of a value in an array.
 * Parameters: value to search for
 *             array[] to search in
 *             elements - number of elements to search
 * Returns:    index where the matching element was found    
 */ 
int findIn(int value, int array[], int elements)
{
  for(int i = 0; i < elements; i++)
  {
    if(value == array[i]) return i;
  }
  return -1;
}  

/*
 * Look(array)
 * look will turn the distance sensor to a new position and collect a distance measurement.
 * it updates the array of distances passed in.
 * the array has 5 values:90 left, 45 left, straight ahead, 45 right, and 90 right. 0, 1, 2, 3, 4
 */
 int current = 0;
 bool clockwise = true;  // if moving to right, true; else, false
 
void look(int values[]) {
  
  if (current == 0) {
    current = 1; 
    clockwise = true; 
  } else if (current == 4) {
    current = 3; 
    clockwise = false; 
  } else if (clockwise) {
    current++;
  } else { // !clockwise
    current --; 
  }
  // wait for movement.
  turret(180 - (current * 45)); 
  delay(250);
  
  values[current] = cmDistance(); // call Ping, collect obstacle distance and store
}

void setLook() {
  turret(180); 
  delay(2000); 
  turret(90); 
  delay (1000);  // Set turret to 0 degrees
  current = 2;  // now looking straight ahead
}

void printObstacles(int values[]) {
   Serial.print("obstacle at: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(values[i]);
    Serial.print(" ");
  }
  Serial.println();
}

/* 
 *  determines direction to go next
 *  tooCloseCm = 30
 */
void nextMove(int values[]) {
  
  if (values[1] > tooCloseCm && values[2] > tooCloseCm && values[3] > tooCloseCm) { // straight is clear
    // keep going straight
    maneuver(200, 200);
  } else if (values[0] > tooCloseCm && values[1] > tooCloseCm) { // left is clear and straight is not
    // go left 
    maneuver(100, 0);
    Serial.println("want to go left!"); 
  } else if (values[3] > tooCloseCm && values[4] > tooCloseCm) { // right is clear and left + straight aren't 
    // go right
    maneuver(0, 100);
    Serial.println("want to go right!"); 
  } else { // no where within checked distances is clear
    // backup 
    maneuver(-200, 200);
    Serial.println("want to go back!"); 
  }
}

