/*   ▄████████ ███▄▄▄▄    ▄████████  ▄██████▄  ████████▄     ▄████████    ▄████████ 
  ███    ███ ███▀▀▀██▄ ███    ███ ███    ███ ███   ▀███   ███    ███   ███    ███ 
  ███    █▀  ███   ███ ███    █▀  ███    ███ ███    ███   ███    █▀    ███    ███ 
 ▄███▄▄▄     ███   ███ ███        ███    ███ ███    ███  ▄███▄▄▄      ▄███▄▄▄▄██▀ 
▀▀███▀▀▀     ███   ███ ███        ███    ███ ███    ███ ▀▀███▀▀▀     ▀▀███▀▀▀▀▀   
  ███    █▄  ███   ███ ███    █▄  ███    ███ ███    ███   ███    █▄  ▀███████████ 
  ███    ███ ███   ███ ███    ███ ███    ███ ███   ▄███   ███    ███   ███    ███ 
  ██████████  ▀█   █▀  ████████▀   ▀██████▀  ████████▀    ██████████   ███    ███ 
    ███        ▄████████   ▄▄▄▄███▄▄▄▄    ▄█  ▄██   ▄      ▄████████   ███    ███ 
▀█████████▄   ███    ███ ▄██▀▀▀███▀▀▀██▄ ███  ███   ██▄   ███    ███              
   ▀███▀▀██   ███    ███ ███   ███   ███ ███▌ ███▄▄▄███   ███    ███              
    ███   ▀   ███    ███ ███   ███   ███ ███▌ ▀▀▀▀▀▀███   ███    ███              
    ███     ▀███████████ ███   ███   ███ ███▌ ▄██   ███ ▀███████████              
    ███       ███    ███ ███   ███   ███ ███  ███   ███   ███    ███              
    ███       ███    ███ ███   ███   ███ ███  ███   ███   ███    ███              
   ▄████▀     ███    █▀   ▀█   ███   █▀  █▀    ▀█████▀    ███    █▀               
                                                                                  

Based on code from: http://forum.arduino.cc/index.php?topic=16618.0
Encoder model E4P from US Digital.

Pins: 
  RED 5V
  BLACK GND
  GREEN A Channel
  YELLOW B Channel
*/

#define PIN_A_ENCODER   2                 //  A channel, using Interrupt 0
#define PIN_B_ENCODER   4                 //  B channel

int encoderDir = 1;                      //  1 Forward, -1 Backwards, 0 Stopped
volatile unsigned int encoderPos = 0;
unsigned int lastPos = encoderPos;
byte ticksPerCM = 40;
int travelledTicks = 0;

void setup() {
  //  Setup Encoder
  pinMode(PIN_A_ENCODER, INPUT);
  pinMode(PIN_B_ENCODER, INPUT);
  digitalWrite(PIN_A_ENCODER, HIGH);      // activate soft pull-ups
  digitalWrite(PIN_B_ENCODER, HIGH);
  
  //  Enable Interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_A_ENCODER), processEncoder, FALLING); //  FALLING=128 steps/rev CHANGE=256 steps/rev
  Serial.begin(9600);
  
  Serial.println("Encoder ready");
}

void loop() {
  if(encoderPos != lastPos){
    travelledTicks = encoderPos - lastPos;
    lastPos = encoderPos;
  }
  else{
    encoderDir = 0;
  }

  Serial.print("Encoder:");
  Serial.print(travelledTicks);
  Serial.print(" Direction:");
  //Serial.println(encoderDir);
  if(encoderDir == 1) Serial.println("Forward");
  else if(encoderDir == -1) Serial.println("Backward");
  else Serial.println("Stop");
}

void processEncoder() {
  // low-to-high on channel A
  // check channel B to see which way encoder is turning
  if(digitalRead(PIN_A_ENCODER) == HIGH){         
    if(digitalRead(PIN_B_ENCODER) == LOW){        
      // CW rotation, forward
      encoderPos++;
      encoderDir = 1;
    }
    else{
      // CCW rotation, reverse
      encoderPos--;
      encoderDir = -1;
    }
  }
  else{                                            
    //  high-to-low on channel A
    // check channel B to see which way encoder is turning
    if(digitalRead(PIN_B_ENCODER) == HIGH){       
      // CW rotation, forward
      encoderPos++;
      encoderDir = 1;
    }
    else {
      // CCW rotation, reverse
      encoderPos = --encoderPos;
      encoderDir = -1;
    }
  }
}

