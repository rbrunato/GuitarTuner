// Arduino Pitch Detection on A0 with autocorrelation and peak detection
// Original author(s): akellyirl, revised by robtillaart, MrMark, barthulsen
// http://forum.arduino.cc/index.php?topic=540969.15
// Continuous ADC ala http://www.instructables.com/id/Arduino-Audio-Input/ "Step 7"
// Further revisions by Rob Brunato - LCD and Motor Additions for COMP444 Project.

#include <Arduino.h>
#include <LiquidCrystal.h>

#define sampleFrequency 9615.4
#define bufferSize 1024

//root frequencies for all notes in A 440Hz scaling
//https://www.seventhstring.com/resources/notefrequencies.html
float C = 16.35;
float Db = 17.32;
float D = 18.35;
float Eb = 19.45;
float E = 20.60;
float F = 21.83;
float Gb = 23.12;
float G = 24.50;
float Ab = 25.96;
float A = 27.50;
float Bb = 29.14;
float B = 30.87;

LiquidCrystal lcd(12, 11, 10, 9, 8, 7);

const int AIN1 = 6; //control pin 1 on the motor driver for the right motor
const int AIN2 = 5; //control pin 2 on the motor driver for the right motor
const int PWMA = 4; //speed control pin on the motor driver for the right motor
const int SW = 2;   //input for the on/off switch
const int turnTimeBase = 150;

volatile byte  rawData[bufferSize] ;  // Buffer for ADC capture
volatile int sampleCnt ;                    // Pointer to ADC capture buffer
long currentSum, previousSum, twoPreviousSum;
int threshold = 0, octaveCount;
String noteName;
float frequency = 0, note, timeOut;
float fundamentalFrequency = 0;
byte pdState = 0;

void setup() {
  //Serial.begin(115200);
  lcd.begin(16, 2); //tell the lcd library that we are using a display that is 16 characters wide and 2 characters high
  lcd.clear();
  pinMode(SW, INPUT_PULLUP);
  cli();//disable interrupts

  //set up continuous sampling of analog pin 0
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0 ;
  ADCSRB = 0 ;

  ADMUX |= (1 << REFS0) ; //set reference voltage
  ADMUX |= (1 << ADLAR) ; //left align the ADC value- so we can read highest 8 bits from ADCH register only

  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // ADC clock 128 prescaler- 16mHz/128=125kHz->9615 samples/sec
  ADCSRA |= (1 << ADATE); //enable auto trigger
  ADCSRA |= (1 << ADIE) ; //enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN) ; //enable ADC
  ADCSRA |= (1 << ADSC) ; //start ADC measurements
}

//interrupts when there is a reading ready on A0
ISR(ADC_vect) {     // When ADC sample ready, put in buffer if not full
  if (sampleCnt < bufferSize)
  {
    rawData[sampleCnt] = ADCH ;
    sampleCnt++ ;
  }
}

void spinMotor(int motorSpeed) //function for driving the right motor
{
  if (motorSpeed > 0) //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH); //set pin 1 to high
    digitalWrite(AIN2, LOW);  //set pin 2 to low
  }
  else if (motorSpeed < 0) //if the motor should drive backwar (negative speed)
  {
    digitalWrite(AIN1, LOW);  //set pin 1 to low
    digitalWrite(AIN2, HIGH); //set pin 2 to high
  }
  else //if the motor should stop
  {
    digitalWrite(AIN1, LOW); //set pin 1 to low
    digitalWrite(AIN2, LOW); //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed)); //now that the motor direction is set, drive it at the entered speed
}

//collect data into an array for the autocorrelation process. 
//Nothing happens until the buffer is full
void readData() {
  sampleCnt = 0 ;
  sei() ;                           // Enable interrupts, samples placed in buffer by ISR
  while (sampleCnt < bufferSize) ;  // Spin until buffer is full
  cli() ;                           // Disable interrupts
}

//this is a function to break down the reading to it's fundamental frequency
//this frequency should be between 15 and 35Hz when it is broken down. 
//the octave count is used to break out of a never ending loop if something goes wrong
//and to allow for us to determin what octave the note was originally in.
void findFundamental(){
  fundamentalFrequency = frequency / 2.0;
  octaveCount = 1;
  while (fundamentalFrequency > 35){
    fundamentalFrequency = fundamentalFrequency / 2.0;
    octaveCount++;
    if (octaveCount > 6){
      break;
    }
  }
}

void findFrequency() {
  // Calculate mean to remove DC offset
  long meanSum = 0 ;
  for (int k = 0; k < bufferSize; k++) {
    meanSum += rawData[k] ;
  }
  char mean = meanSum / bufferSize ;
  // Remove mean
  for (int k = 0; k < bufferSize; k++) {
    rawData[k] -= mean ;
  }

  // Autocorrelation
  currentSum = 0 ;
  pdState = 0 ;
  for (int i = 0; i < bufferSize && (pdState != 3); i++) {
    // Autocorrelation
    float period = 0 ;
    twoPreviousSum = previousSum ;
    previousSum = currentSum ;
    currentSum = 0 ;
    for (int k = 0; k < bufferSize - i; k++) {
      currentSum += char(rawData[k]) * char(rawData[k + i]) ;
    }
    // Peak detection
    switch (pdState) {
      case 0:   // Set threshold based on zero lag autocorrelation
        threshold = currentSum / 2 ;
        pdState = 1 ;
        break ;
      case 1:   // Look for over threshold and increasing
        if ((currentSum > threshold) && (currentSum - previousSum) > 0) pdState = 2 ;
        break ;
      case 2:   // Look for decreasing (past peak over threshold)
        if ((currentSum - previousSum) <= 0) {
          // quadratic interpolation
          float interpolationValue = 0.5 * (currentSum - twoPreviousSum) / (2 * previousSum - twoPreviousSum - currentSum) ;
          period = i - 1 + interpolationValue ;
          pdState = 3 ;
        }
        break ;
      default:
        pdState = 3 ;
        break ;
    }

    // Frequency identified in Hz
    if (threshold > 100) {
      frequency = sampleFrequency / period;
      if (frequency < 1000){
        findFundamental();
        timeOut = millis();
      }
    }
    else{
      frequency = -1;
    }
  }
}

void displayResults(){
  if(frequency != -1)  
  { //start determining what note the freqency is close to and set some parameters. 
    if((15.895<=fundamentalFrequency) & (fundamentalFrequency<16.835))
    { 
        note = C;
        noteName = "C ";
    }
    else if((16.835<=fundamentalFrequency) & (fundamentalFrequency<17.835))
    { 
        note = Db;
        noteName = "Db";
    }
    else if((17.335<=fundamentalFrequency) & (fundamentalFrequency<18.9))
    { 
        note = D; 
        noteName = "D ";
    }
    else if((18.9<=fundamentalFrequency) & (fundamentalFrequency<20.025))
    { 
        note = Eb;
        noteName = "Eb";
    }
    else if((20.025<=fundamentalFrequency) & (fundamentalFrequency<21.215))
    { 
        note = E;
        noteName = "E ";
    } 
    else if((21.215<=fundamentalFrequency) & (fundamentalFrequency<22.475))
    { 
        note = F;
        noteName = "F ";
    }
    else if((22.475<=fundamentalFrequency) & (fundamentalFrequency<23.81))
    { 
        note = Gb;
        noteName = "Gb";
    }
    else if((23.81<=fundamentalFrequency) & (fundamentalFrequency<25.23))
    { 
        note = G;
        noteName = "G ";
    }
    else if((25.23<=fundamentalFrequency) & (fundamentalFrequency<26.73))
    { 
        note = Ab;
        noteName = "Ab";
    }
    else if((26.73<=fundamentalFrequency) & (fundamentalFrequency<28.32))
    {
        note = A;
        noteName = "A ";
    }
    else if((28.32<=fundamentalFrequency) & (fundamentalFrequency<30.005))
    { 
        note = Bb;
        noteName = "Bb";
    }
    else if((30.005<=fundamentalFrequency) & (fundamentalFrequency<31.785))
    { 
        note = B;
        noteName = "B ";
    }
    //display the note and the octave
    lcd.setCursor(0,0);
    lcd.print("Note: ");
    lcd.setCursor(6,0);
    lcd.print(noteName);
    lcd.setCursor(9,0);
    lcd.print("Oct: ");
    lcd.setCursor(14,0);
    lcd.print(octaveCount);

    //how close are we to pitch? 
    String output;
    int turnTime;
    int turnSpeed;
    if (fundamentalFrequency == note)   //bang on!
    {
      output = "b------[]------#";
      turnTime = 0;
      turnSpeed = 0;
    }
    else
    {
      //a fancy equation to set some thresholds
      float tuning = ((fundamentalFrequency/note)-1)*100;
      if (tuning >= -0.2 && tuning <= 0.2){   //close enough!
        output = "b------[]------#";
        turnTime = 0;
        turnSpeed = 0;
      }
      // the flat side
      else if (tuning >= -0.5 && tuning < -0.2){   
        output = "b----->[]------#";
        turnTime = 5;
        turnSpeed = -200;
      }
      else if (tuning >= -1.0 && tuning < -0.5){   
        output = "b---->>[]------#";
        turnTime = 10;
        turnSpeed = -200;
      }
      else if (tuning >= -1.5 && tuning < -1.0){   
        output = "b--->>>[]------#";
        turnTime = 15;
        turnSpeed = -200;
      }
      else if (tuning >= -2.0 && tuning < -1.5){   
        output = "b-->>>>[]------#";
        turnTime = 20;
        turnSpeed = -200;
      }
      else if (tuning >= -2.5 && tuning < -2.0){   
        output = "b->>>>>[]------#";
        turnTime = 25;
        turnSpeed = -200;
      }
      else if (tuning < -2.5){   
        output = "b>>>>>>[]------#";
        turnTime = 30;
        turnSpeed = -200;
      }
      // the sharp side
      else if (tuning > 0.2 && tuning <= 0.5){   
        output = "b------[]<-----#";
        turnTime = 5;
        turnSpeed = 200;
      }
      else if (tuning > 0.5 && tuning <= 1.0){   
        output = "b------[]<<----#";
        turnTime = 10;
        turnSpeed = 200;
      }
      else if (tuning > 1.0 && tuning <= 1.5){   
        output = "b------[]<<<---#";
        turnTime = 15;
        turnSpeed = 200;
      }
      else if (tuning > 1.5 && tuning <= 2.0){   
        output = "b------[]<<<<--#";
        turnTime = 20;
        turnSpeed = 200;
      }
      else if (tuning > 2.0 && tuning <= 2.5){   
        output = "b------[]<<<<<-#";
        turnTime = 25;
        turnSpeed = 200;
      }
      else if (tuning > 2.5){   
        output = "b------[]<<<<<<#";
        turnTime = 30;
        turnSpeed = 200;
      }
    }
    lcd.setCursor(0,1);
    lcd.print(output);

    //spin motor - this section has been removed due to an issue with the motor I own
    //Sadly, tt is not geared with enough torque to turn the guitar tuners. 
    //I tried on a number of guitars all with the same result. 
    //spinMotor(turnSpeed);
    //delay(turnTime * turnTimeBase);
    //spinMotor(0);
  }
  else
  { // we didnt get a reading within threshold... 
    //lets wait 1000 ms before clearing the screen since the last good reading
    if (millis() - timeOut > 800){
      lcd.clear(); 
    }
  }
}

void loop() {
  if (digitalRead(SW) == LOW) //if the switch is on
  {
    readData();
    findFrequency();
    displayResults();
  }
  else{
    cli();
    lcd.clear();
  }
}