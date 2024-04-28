// RoombaMain with classes 
// Alex Johnson
// For CS 241
//
// Started: 4/6/2024
// Updated: 4/27/2024

#include "SoftwareSerial.h"
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

#define ROOMBA_START_OPCODE                 128
#define ROOMBA_CONTROL_OPCODE               130
#define ROOMBA_FULL_OPCODE                  132
#define ROOMBA_POWER_OPCODE                 133
#define ROOMBA_DRIVE_OPCODE                 137
#define ROOMBA_SONG_DEFINITION_OPCODE       140
#define ROOMBA_SONG_PLAY_OPCODE             141
#define ROOMBA_FORWARD_OPCODE               32768

//************************************
// class ardRoomba - Class definition 
//************************************

class ardRoomba {
private:
    // ********************** private member variables **********************
    int buttonPin; 

    int rxPin;

    int txPin;

    SoftwareSerial Roomba = SoftwareSerial(rxPin, txPin);   

    bool spin = 0;   

    int temperaturethresh = 172;  // default value 

    Adafruit_AMG88xx amg;

    float pixels[AMG88xx_PIXEL_ARRAY_SIZE];

    float columnSum[AMG88xx_PIXEL_ARRAY_SIZE/8] = {0,0,0,0,0,0,0,0};



    // ********************** private member functions **********************
    // startTones
    // defines a song on the roomba and plays it, song is defined in buffer mysong,
    // and buffer to play song is playSong
    void startTone(){
        const uint8_t songBuffSize = 9;
        const uint8_t songBuff[songBuffSize] =
            {ROOMBA_SONG_DEFINITION_OPCODE, 0, 3,         
            68, 16,                                          
            72, 16,
            75, 16};
        const uint8_t playSongSize = 2; 
        const uint8_t playSong[playSongSize] = {ROOMBA_SONG_PLAY_OPCODE, 0};

        Roomba.write(songBuff, songBuffSize);
        Roomba.write(playSong, playSongSize);
        delay(750);
    }

    // roombaFullStart
    //    defines a buffer that contains the roomba start opcode and full mode opcode,
    //    sets up tx and rx pins, aswell as sending the start buffer to the roomba
    //    CONTAINS three seconds of hard coded delaly including that within startTones()
    void roombaFullStart(){
        const int startBuffSize = 2;
        uint8_t startbuff[startBuffSize] = {ROOMBA_START_OPCODE, ROOMBA_FULL_OPCODE};
        delay(1000); // time at start for some reason
        Roomba.begin(19200);
        delay(1000);
        Roomba.write(startbuff, startBuffSize); 
        delay(100);
        startTone();
    }
    // Roomba drive bounds 
    //    Velocity (-500 – 500 mm/s)
    //    Radius (-2000 – 2000 mm)
    //    
    //    Special cases:
    //    Straight = 32768 : defined as ROOMBA_FORWARD_OPCODE
    //    Turn in place clockwise = -1
    //    Turn in place counter-clockwise = 1
    // roombaDrive
    //    given a velocity and a radius to drive at,
    //    converts 16 bit velocity and 16 bit radius into four 8 bit unsigned integers,
    //    and sends them to roomba along with the drive opcode
    void roombaDrive(uint16_t Velocity, uint16_t Radius)
    {
        uint8_t highVel = (Velocity>>8) & 0xff;
        uint8_t lowVel = (Velocity>>0) & 0xff;
        uint8_t highRad = (Radius>>8) & 0xff;
        uint8_t lowRad = (Radius>>0) & 0xff;
        const int driveBuffSize = 5;
        uint8_t drivebuff[driveBuffSize] = {ROOMBA_DRIVE_OPCODE, highVel, lowVel, highRad, lowRad};
        Roomba.write(drivebuff, driveBuffSize);
    }

    // readColumnSum
    //    reads the values from thermal sensor,
    //    sums the columns and places the sums in float array
    //    columnSum
    void readColumnSum()
    {
        for(int i = 0; i < 8; i++)
        {
            columnSum[i] = 0;
        }
      
        amg.readPixels(pixels);
        for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
          // i-1
          columnSum[(i-1)%8] += pixels[i-1];    
        }
    }


    // maxColumn
    //    returns the number of the column with the highest sum,
    //    or -1 if none of the columns are warmer than the defined temperature
    //    threshhold, which is approx (room temp * 8) 
    int maxColumn()
    {
        readColumnSum();
        int maxColumn = -1;
        float columnMax = 0;
        //const int temperaturethresh = 172; // degrees celsius
        for(int i = 0; i < 8; i++)
        {      
            if( columnSum[i] >= columnMax && columnSum[i] > temperaturethresh)
            {
              columnMax = columnSum[i];
              maxColumn = i;
            }
          
        }
        return maxColumn;
    }
    // turnAngle
    //    given a column number 0-7, will return
    //    a value for the roomba to turn at
    uint16_t turnAngle(uint8_t column) {
        const int turnRate = 100; 
        if (column > 4) {
            return turnRate;
        } 
        else if (column < 3) {
            return -turnRate;
        }
        else {
            return ROOMBA_FORWARD_OPCODE;
        }
    }


    // roombaON
    //    puleses roomba device detect pin for 500 ms, 
    //    this will turn on the roomba if it is turned off
    void roombaOn()
    {
        const int deviceDetectPin = 2;
        pinMode(deviceDetectPin, OUTPUT);
        digitalWrite(deviceDetectPin, HIGH);
        delay(500);
        digitalWrite(deviceDetectPin, LOW);
        delay(500);
        digitalWrite(deviceDetectPin, HIGH);
        delay(500);
    }

    // findThresh
    //    returns the average temperature across thermal sensor at startup
    //    for good performace point away from thermal object at startup 
    int findThresh()
    {
      readColumnSum();
      int thresh = 0; 
      for( int i = 0; i< 8; i++)
      {
        thresh += columnSum[i];
      }
      return ((thresh/8) + 15);
    }
    // startButton
    //    hangs code in while loop until pin 8 goes to zero 
    void startButton()
    {
        int buttonValue = 1; 
        while(buttonValue == 1)
        {
          buttonValue = digitalRead(8);
        }
    }

    // checkButton
    //    checks the button pin, if it's low halts roomba movement, 
    //    will resume once button pin is low again 
    void checkButton(){
    int buttonState = digitalRead(buttonPin); 
        if(buttonState != 1){
            roombaDrive(0, 0);
            spin = 0;   
            while(buttonState == 0)
            {
                buttonState = digitalRead(buttonPin);
            }   
            startButton(); 
        }
    } 
    public:

    // ********************** public member functions **********************
    // ardRoomba Constructor 
    //    takes three integers, constructs an ardRoomba
    //    with specified pins
    ardRoomba(int tx, int rx, int button)
        : txPin{tx},
          rxPin{rx},
          buttonPin{button}
    {
        pinMode(rxPin, INPUT);
        pinMode(txPin, OUTPUT);
        pinMode(buttonPin, INPUT_PULLUP);
    }

    // startup 
    //    waits for start button,
    //    then turns on roomba,
    //    then finds temp threshhold
    //    then puts roomba in controlable state
    void startup() 
    {   
        startButton();
        roombaOn();
    
        bool status;
        status = amg.begin();  // will get stuck here if thermal sensor isn't connected
        if (!status) {
            while (1);
        }

        temperaturethresh = findThresh(); 
        roombaFullStart();
    }
    //  runloop
    //    checks pause button
    //    checks thermal sensor
    //    sets roomba to drive accordingly  
    //          either turn toward warmest column
    //          or spin until a warm enough column is found 
    void runLoop() {
        const uint16_t vel = 150;
        checkButton(); 
        int column = maxColumn();  
        if(column != -1) 
        {
            uint16_t driveangle = turnAngle(column);              
            roombaDrive(vel, driveangle);  
        }
        else if(!(spin))                            
        {
            spin = !(spin);
            roombaDrive(vel, -1);
      }
    }
    //  turnOFF
    //      sends roomba the Power opcode, 
    //      equivalent to pushing power button on roomba 
    void turnOff()
    {
        Roomba.write(ROOMBA_POWER_OPCODE);
    }
};  //************************ End of Class ardRoomba**********************************

// define object of time ardRoomba 
ardRoomba roomba{14, 15, 8};
// global timer start
long startTime = 0; 

void setup()
{
roomba.startup();
startTime = millis();
}

void loop()
{
    roomba.runLoop();
    long currentTime = millis();
    long elapsedTime = currentTime - startTime;
    if(elapsedTime > 30000)  // 30 second timer
    {
        roomba.turnOff();
        roomba.startup();
    }
    delay(200); // delay to slow down how fast the loop runs in in accordance with ag8338 refresh rate
}