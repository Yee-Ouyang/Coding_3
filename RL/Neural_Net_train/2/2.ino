//Machine learnng crawling Robot

//Reinforcement using Neural Net

//by: jim demello November 2018 at Shangluo University

//Adapted neural net from here: http://www.the-diy-life.com/running-an-artifical-...

// Servo setup: the servos must be oriented so that if the arm is rotating counter-clockwise to the left of the servo, then up is 0 degrees

// and down is 160(servoMax) degrees, for both servos. Then when the arm is in it's highest postion, servo 1 (the servo closest to the

// body of the robot, will be at 0 degrees and servo 2 will be at 0 degrees.)

// Sonar: the ultrasonic module should be placed facing the rear of the robot as it measures movement of the robot away from some

// solid structure like a wall.

// goal: move between two arm positions that produce the greatest distance, without using arrays to store results, only NN

// algorithm: this robot uses a NN to train on training data, then practice moving with the NN and then repeating the most successful movement (learned behavior)

// this algorithm is better than my previous RL algorithms because once the NN is trained, then any random servo positions between

// 0 and servoMax can be used.

// To make this NN work, it is all in setting up the input and training arrays. There may be better ways of doing than I have done here.

// Also you can play with the various NN settings.

// Note: while this algorithm is very entertaining, it seems that a simple Reinforment Learning algorithm is just as accurate without all the complexity of the NN. But it

// it is a fun application of a NN. Perhaps someone can find a better way to fit the application to a NN - perhaps by applying the Distance reading to the back-propagation

// rather than as an input to NN.

#include

Servo servo1,servo2;

#ifdef __arm__

// should use uinstd.h to define sbrk but Due causes a conflict

extern "C" char* sbrk(int incr);

#else // __ARM__

extern char *__brkval;

#endif // __arm__

int freeMemory() { // this routine reports on free ram space

char top;

#ifdef __arm__

return &top - reinterpret_cast(sbrk(0));

#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)

return &top - __brkval;

#else // __arm__

return __brkval ? &top - __brkval : &top - __malloc_heap_start;

#endif // __arm__

}

float distance;

float sonarTime;

int TRIGGER=7,ECHO=8; // ultrasonic sensor pins

int spos1 = 0; // servo1 position

int spos2 = 0; // servo2 position

int spos3 = 0; // servo1 position

int spos4 =0; // servo2 position

int spos1High =0; // servo1 highest position - which means these arm positions achieved the biggest distance reading and so will repeat them in final execution routine

// I know, I am terrible at variable names

int spos2High =0;

int spos3High =0;

int spos4High =0;

int distanceHigh=0; // save highest distance reading

float highOutput = 0.0;

int numberTrainingCycles = 100; // number of times to train robot on NN after the NN has trained on the Input and Target arrays

int servoMin = 0;

int servoMax = 160;

float distDifference=0,distPrevious=0,distCurrent=0;

#include "math.h"

/******************************************************************

Network Configuration - customized per network

******************************************************************/

const int PatternCount = 16;

const int InputNodes = 5;

const int HiddenNodes = 7;

const int OutputNodes = 1;

const float LearningRate = 0.2;

const float Momentum = 0.9;

const float InitialWeightMax = 0.5;

const float Success = 0.0015;

float Target[PatternCount][OutputNodes] = { // these are the successful outputs corresponding to arm positions in Input array

// 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16

{ 0 }, { 0 }, { 0 }, { 1 }, { 0 }, { 0 }, { 0,}, { 1 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 1 } };

const float Input [PatternCount][InputNodes] {

{ 0,0,0,0,0}, // colum 1 and 2 holds servo1 positions and column 3 and 4 holds servo 2 positions - col 5 holds distance

{ 0,0,0,1,0},

{ 0,0,1,0,0},

{ 0,0,1,1,1},

{ 1,0,0,0,0},

{ 1,0,0,1,0},

{ 1,0,1,0,0},

{ 1,0,1,1,1},

{ 1,1,0,0,0},

{ 1,1,0,1,0},

{ 1,1,1,0,0},

{ 1,1,1,1,0},

{ 0,1,0,0,0},

{ 0,1,0,1,0},

{ 0,1,1,0,0},

{ 0,1,1,1,1}};

/******************************************************************

End Network Configuration

******************************************************************/

int i, j, p, q, r;

int ReportEvery1000;

int RandomizedIndex[PatternCount];

long TrainingCycle;

float Rando;

float Error = 2;

float Accum;

float Hidden[HiddenNodes];

float Output[OutputNodes];

float HiddenWeights[InputNodes + 1][HiddenNodes];

float OutputWeights[HiddenNodes + 1][OutputNodes];

float HiddenDelta[HiddenNodes];

float OutputDelta[OutputNodes];

float ChangeHiddenWeights[InputNodes + 1][HiddenNodes];

float ChangeOutputWeights[HiddenNodes + 1][OutputNodes];

long previousMillis = 0;

unsigned long currentMillis;

long loopTimer = 10; // do the main processing every 10 milliseconds (probably not necessary to use this, I just copied it in from another program)

void setup (){

Serial.begin(115200);

Serial.println("Starting program");

randomSeed(analogRead(A1)); //Collect a random ADC sample for Randomization.

ReportEvery1000 = 1;

for ( p = 0 ; p < PatternCount ; p++ ) {

RandomizedIndex[p] = p ;

}

Serial.println("do train_nn"); // do NN training

train_nn();

delay(1000);

servo1.attach( 9, 600, 2400 );

servo2.attach( 6, 600, 2400 );

myServo(servo1,0,1,8,1); // set servos to zero position

delay(1000);

myServo(servo2,0,1,8,1);

delay(1000);

pinMode(TRIGGER, OUTPUT); // setup ultrasonic sensor

pinMode(ECHO, INPUT);

distPrevious = getDistance(); //get initial distance

Serial.print("Initial distance= ");Serial.println(distPrevious);

delay(1000);

// exit(0); // exit here to just test sonar

} // end setup

float getDistance() { // routine to measure distance = call and average it 5 times

int numberTriggers = 5;

int average = 0;

for(int i=0;i

digitalWrite(TRIGGER, LOW);

delayMicroseconds(5);

digitalWrite(TRIGGER, HIGH);

delayMicroseconds(10);

digitalWrite(TRIGGER, LOW);

sonarTime = pulseIn(ECHO, HIGH);

distance = sonarTime / 58.00;

average = average + distance;

delay(100);

} // end for i

average = average / numberTriggers;

Serial.print("Distance = ");Serial.println(average);

return average;

}// end get sonar distance routine

void doLearnedBehavior() {

Serial.println("Do Learned behavior... ");

myServo(servo1,0,1,8,1);

myServo(servo2,0,1,8,1);

delay(2000);

for (int i=0;i<30;i++) {

Serial.print(" spos1High= "); Serial.print(spos1High);

Serial.print(" spos2High = ");Serial.print(spos2High);

Serial.print(" spos3High = ");Serial.print(spos3High);

Serial.print(" spos4High = ");Serial.println(spos4High);

myServo(servo1,spos1High,1,7,1);

myServo(servo2,spos2High,1,7,1);

myServo(servo1,spos3High,1,7,1);

myServo(servo2,spos4High,1,7,1);

} // doLearned

} // end loop

void loop(){ // main loop reads success table and performs actions

int freespace = freeMemory(); Serial.print("free memory= "); Serial.println(freespace); // just wanted to see if memory ever became a problem

drive_nn();

freespace = freeMemory(); Serial.print("free memory= "); Serial.println(freespace);

doLearnedBehavior();

myServo(servo1,0,1,8,1);

myServo(servo2,0,1,8,1);

Serial.print("end program ");

delay(2000);

exit(0); // quit program

} // end main loop

void myServo(Servo servo,int newAngle,int angleInc,int incDelay,int servoNum) {// routine to read current servo angle, advance it to newAngle and control speed

int curAngle = 0;

curAngle = servo.read();

//Serial.print("curAngle = "); Serial.println(curAngle);

if (curAngle < newAngle) {

for(int angle=curAngle;angle < newAngle;angle += angleInc) {

servo.write(angle);

delay(incDelay); }

}

else if (curAngle > newAngle) {

for(int angle=curAngle;angle > newAngle;angle -= angleInc) {

servo.write(angle);

delay(incDelay); }

}

} // end of myServo function

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/after having trained the NN, now drive the robot on the NN, and store the highest distance yielding servo positions

void drive_nn()

{

Serial.println("Running NN Drive ");

numberTrainingCycles = 20; // number of times to try random servo positions and get distances, then store the highest for final walking

for (int x=0;x < numberTrainingCycles;x++) {

currentMillis = millis();

float TestInput[] = {0, 0};

if(currentMillis - previousMillis > loopTimer) { //do calculation every 5 or more milliseconds

// Serial.print("currentMillis= ");Serial.println(currentMillis);

int randomNo = random(servoMax);

float pos1 = map(randomNo,0,servoMax,0,100); // randomly get servo 1 position between 0 and servoMax

pos1 = pos1/100;

randomNo = random(servoMax);

float pos2 = map(randomNo,0,servoMax,0,100);

pos2 = pos2/100;

randomNo = random(servoMax);

float pos3 = map(randomNo,0,servoMax,0,100);

pos3 = pos3/100;

randomNo = random(servoMax);

float pos4= map(randomNo,0,servoMax,0,100);

pos4 = pos4/100;

// move robot with new random positions

myServo(servo1,pos1 * servoMax,1,7,1);

myServo(servo2,pos2 * servoMax,1,7,1);

myServo(servo1,pos3 * servoMax,1,7,1);

myServo(servo2,pos4 * servoMax,1,7,1);

/////////////////////////

// get distance for pos5

/////////////////////////

//float temp = getDistance(); // get distance

distCurrent = getDistance(); // get distance

distDifference = distCurrent - distPrevious;

distPrevious = distCurrent;

Serial.print("===> distDifference = ");Serial.println(distDifference);

// if ((pos1 < .70) && (pos3 > .8)) temp = 3; //testing distance

// else temp = 0;

//////////////////////////////////////////////////////////////

// may have to increase range to make distance more powerful

/////////////////////////////////////////////////////////////

float temp = map(distDifference,0,10,0,100);

float pos5 = temp/100;

InputToOutput(pos1,pos2,pos3,pos4,pos5); //input to NN to get Output[]

pos1 = pos1 * servoMax; pos2 = pos2 * servoMax; pos3 = pos3 * servoMax; pos4 = pos4 * servoMax; //pos5 = pos5 * servoMax;

Serial.print(" pos1= ");Serial.print(pos1);Serial.print(" pos2= ");Serial.print(pos2);Serial.print(" pos3= ");Serial.print(pos3);

Serial.print(" pos4= ");Serial.print(pos4);Serial.print(" pos5= ");Serial.print(pos5);

Serial.print("Output from NN =");Serial.println(Output[0]);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// IF output is greater than .10 then use those positions to move robot and store the highest output positions achieved

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

if (Output[0] > .10) {

if (Output[0] > highOutput) {

highOutput = Output[0];

spos1High = pos1;

spos2High = pos2;

spos3High = pos3;

spos4High = pos4;

Serial.print(" --------> spos1High= ");Serial.print(spos1High);Serial.print(" spos2High= ");Serial.print(spos2High);Serial.print(" spos3High= ");Serial.print(spos3High);

Serial.print(" spos4High= ");Serial.print(spos4High);Serial.print(" Output= ");Serial.println(Output[0]);

}

}

previousMillis = currentMillis;

} // end millis loop

}

} //end of drive_nn() function

//DISPLAYS INFORMATION WHILE TRAINING

void toTerminal()

{

for ( p = 0 ; p < PatternCount ; p++ ) {

Serial.println();

Serial.print (" Training Pattern: ");

Serial.println (p);

Serial.print (" Input ");

for ( i = 0 ; i < InputNodes ; i++ ) {

Serial.print (Input[p][i], DEC);

Serial.print (" ");

}

Serial.print (" Target ");

for ( i = 0 ; i < OutputNodes ; i++ ) {

Serial.print (Target[p][i], DEC);

Serial.print (" ");

}

/******************************************************************

Compute hidden layer activations

******************************************************************/

for ( i = 0 ; i < HiddenNodes ; i++ ) {

Accum = HiddenWeights[InputNodes][i] ;

for ( j = 0 ; j < InputNodes ; j++ ) {

Accum += Input[p][j] * HiddenWeights[j][i] ;

}

Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ; // activation function

}

/******************************************************************

Compute output layer activations and calculate errors

******************************************************************/

for ( i = 0 ; i < OutputNodes ; i++ ) {

Accum = OutputWeights[HiddenNodes][i] ;

for ( j = 0 ; j < HiddenNodes ; j++ ) {

Accum += Hidden[j] * OutputWeights[j][i] ;

}

Output[i] = 1.0 / (1.0 + exp(-Accum)) ;

}

Serial.print (" Output ");

for ( i = 0 ; i < OutputNodes ; i++ ) {

Serial.print (Output[i], 5);

Serial.print (" ");

}

}

}

void InputToOutput(float In1,float In2, float In3, float In4, float In5)

{

float TestInput[] = {0,0,0,0,0};

// Serial.print("In1 = ");Serial.println(In1);

TestInput[0] = In1; //first servo arm position - servo 1

TestInput[1] = In2; //first servo arm position - servo 2

TestInput[2] = In3; // 2nd servo arm position - servo 1

TestInput[3] = In4; // 2nd servo arm position - servo 2

TestInput[4] = In5; // distance

/******************************************************************

Compute hidden layer activations

******************************************************************/

for ( i = 0 ; i < HiddenNodes ; i++ ) {

Accum = HiddenWeights[InputNodes][i] ;

for ( j = 0 ; j < InputNodes ; j++ ) {

Accum += TestInput[j] * HiddenWeights[j][i] ;

}

Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;

}

/******************************************************************

Compute output layer activations and calculate errors

******************************************************************/

for ( i = 0 ; i < OutputNodes ; i++ ) {

Accum = OutputWeights[HiddenNodes][i] ;

for ( j = 0 ; j < HiddenNodes ; j++ ) {

Accum += Hidden[j] * OutputWeights[j][i] ;

}

Output[i] = 1.0 / (1.0 + exp(-Accum)) ;

}

//#ifdef DEBUG

Serial.print (" Output ");

for ( i = 0 ; i < OutputNodes ; i++ ) {

Serial.print (Output[i], 5);

Serial.print (" ");

}

//#endif

}

//TRAINS THE NEURAL NETWORK

void train_nn() {

/******************************************************************

Initialize HiddenWeights and ChangeHiddenWeights

******************************************************************/

int prog_start = 0;

Serial.println("start training...");

//digitalWrite(LEDYEL, LOW);

for ( i = 0 ; i < HiddenNodes ; i++ ) {

for ( j = 0 ; j <= InputNodes ; j++ ) {

ChangeHiddenWeights[j][i] = 0.0 ;

Rando = float(random(100)) / 100;

HiddenWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;

}

}

//digitalWrite(LEDYEL, HIGH);

/******************************************************************

Initialize OutputWeights and ChangeOutputWeights

******************************************************************/

//digitalWrite(LEDRED, LOW);

for ( i = 0 ; i < OutputNodes ; i ++ ) {

for ( j = 0 ; j <= HiddenNodes ; j++ ) {

ChangeOutputWeights[j][i] = 0.0 ;

Rando = float(random(100)) / 100;

OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;

}

}

//digitalWrite(LEDRED, HIGH);

//SerialUSB.println("Initial/Untrained Outputs: ");

//toTerminal();

/******************************************************************

Begin training

******************************************************************/

for ( TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++) {

/******************************************************************

Randomize order of training patterns

******************************************************************/

for ( p = 0 ; p < PatternCount ; p++) {

q = random(PatternCount);

r = RandomizedIndex[p] ;

RandomizedIndex[p] = RandomizedIndex[q] ;

RandomizedIndex[q] = r ;

}

Error = 0.0 ;

/******************************************************************

Cycle through each training pattern in the randomized order

******************************************************************/

for ( q = 0 ; q < PatternCount ; q++ ) {

p = RandomizedIndex[q];

/******************************************************************

Compute hidden layer activations

******************************************************************/

//digitalWrite(LEDYEL, LOW);

for ( i = 0 ; i < HiddenNodes ; i++ ) {

Accum = HiddenWeights[InputNodes][i] ;

for ( j = 0 ; j < InputNodes ; j++ ) {

Accum += Input[p][j] * HiddenWeights[j][i] ;

}

Hidden[i] = 1.0 / (1.0 + exp(-Accum)) ;

}

//digitalWrite(LEDYEL, HIGH);

/******************************************************************

Compute output layer activations and calculate errors

******************************************************************/

//digitalWrite(LEDRED, LOW);

for ( i = 0 ; i < OutputNodes ; i++ ) {

Accum = OutputWeights[HiddenNodes][i] ;

for ( j = 0 ; j < HiddenNodes ; j++ ) {

Accum += Hidden[j] * OutputWeights[j][i] ;

}

Output[i] = 1.0 / (1.0 + exp(-Accum)) ;

OutputDelta[i] = (Target[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;

Error += 0.5 * (Target[p][i] - Output[i]) * (Target[p][i] - Output[i]) ;

}

// Serial.println(Output[0]*100);

//digitalWrite(LEDRED, HIGH);

/******************************************************************

Backpropagate errors to hidden layer

******************************************************************/

//digitalWrite(LEDYEL, LOW);

for ( i = 0 ; i < HiddenNodes ; i++ ) {

Accum = 0.0 ;

for ( j = 0 ; j < OutputNodes ; j++ ) {

Accum += OutputWeights[i][j] * OutputDelta[j] ;

}

HiddenDelta[i] = Accum * Hidden[i] * (1.0 - Hidden[i]) ;

}

//digitalWrite(LEDYEL, HIGH);

/******************************************************************

Update Inner-->Hidden Weights

******************************************************************/

//digitalWrite(LEDRED, LOW);

for ( i = 0 ; i < HiddenNodes ; i++ ) {

ChangeHiddenWeights[InputNodes][i] = LearningRate * HiddenDelta[i] + Momentum * ChangeHiddenWeights[InputNodes][i] ;

HiddenWeights[InputNodes][i] += ChangeHiddenWeights[InputNodes][i] ;

for ( j = 0 ; j < InputNodes ; j++ ) {

ChangeHiddenWeights[j][i] = LearningRate * Input[p][j] * HiddenDelta[i] + Momentum * ChangeHiddenWeights[j][i];

HiddenWeights[j][i] += ChangeHiddenWeights[j][i] ;

}

}

//digitalWrite(LEDRED, HIGH);

/******************************************************************

Update Hidden-->Output Weights

******************************************************************/

//digitalWrite(LEDYEL, LOW);

for ( i = 0 ; i < OutputNodes ; i ++ ) {

ChangeOutputWeights[HiddenNodes][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes][i] ;

OutputWeights[HiddenNodes][i] += ChangeOutputWeights[HiddenNodes][i] ;

for ( j = 0 ; j < HiddenNodes ; j++ ) {

ChangeOutputWeights[j][i] = LearningRate * Hidden[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;

OutputWeights[j][i] += ChangeOutputWeights[j][i] ;

}

}

//digitalWrite(LEDYEL, HIGH);

}

/******************************************************************

Every 100 cycles send data to terminal for display and draws the graph on OLED

******************************************************************/

ReportEvery1000 = ReportEvery1000 - 1;

if (ReportEvery1000 == 0)

{

int graphNum = TrainingCycle / 100;

int graphE1 = Error * 1000;

int graphE = map(graphE1, 3, 80, 47, 0);

Serial.print ("TrainingCycle: ");

Serial.print (TrainingCycle);

Serial.print (" Error = ");

Serial.println (Error, 5);

toTerminal();

if (TrainingCycle == 1)

{

ReportEvery1000 = 99;

}

else

{

ReportEvery1000 = 100;

}

}

/******************************************************************

If error rate is less than pre-determined threshold then end

******************************************************************/

if ( Error < Success ) break ;

}

Serial.println("End training.");

}
