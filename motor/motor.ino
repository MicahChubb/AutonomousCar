// Motor A
int enA = 10;
int in1 = 9;
int in2 = 8;
 
// Motor B
int enB = 5;
int in3 = 7;
int in4 = 6;

// Lower is 170, upper is 255
int Speed = 255;

void setup()
{
  Serial.begin(9600);
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}
 
void straight()
{
  //motor A
  analogWrite(enA, Speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  //motor B
  analogWrite(enB, Speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void left()
{
  // Now turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW); 
  //motor B
  analogWrite(enB, Speed);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void right()
{
  // Now turn off motors 
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  //motor A
  analogWrite(enA, Speed);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void loop(){

  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    int moves = data.toInt();
    
    switch (moves){
      case 1:
        left();
        break;
      case 2:
        straight();
        break;
      case 3:
        right();
        break;
       case 4:
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        break;
        
    }
    //delay(1000);
  }
}
