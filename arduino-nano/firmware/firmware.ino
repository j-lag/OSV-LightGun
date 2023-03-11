#define BUTTONCOUNT 6
int buttons[BUTTONCOUNT]     = {2,3,4,5,6,7};
int buttonstate[BUTTONCOUNT] = {0,0,0,0,0,0};

#define AXISCOUNT 2
int axis[AXISCOUNT]     = {0,1};
int axisstate[AXISCOUNT] = {0,0};

void setup() 
{
  // open the serial port at 9600 bps:
  Serial.begin(9600); 
  // make the pushbutton's pin an input:
  for(int i = 0; i < BUTTONCOUNT; ++i)
    pinMode(buttons[i], INPUT_PULLUP);  
}

void loop() 
{
  char buffer[64];
  
  // read the input pin:
  for(int i = 0; i < BUTTONCOUNT; ++i)
  {
    buttonstate[i] = digitalRead(buttons[i]);
  }
  for(int i = 0; i < AXISCOUNT; ++i)
  {
    axisstate[i] = analogRead(axis[i]);
  }
  
  //report to host
  sprintf(buffer, "D %d %d %d %d %d %d %04d %04d",
  buttonstate[0],
  buttonstate[1],
  buttonstate[2],
  buttonstate[3],
  buttonstate[4],
  buttonstate[5],
  axisstate[0],
  axisstate[1]
  );  
  Serial.print(buffer);
  
  // delay in between reads for stability
  delay(15);
}
