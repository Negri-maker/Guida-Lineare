#define DIR1 4
#define PWM1 5
#define read2Pin A1 // lettura asse Y Joystik 1 se funzionante con Joystick

// definizione pin Encoder
#define encoder1PinA 2
#define encoder1PinB 3

// definizione End Stop
#define Min_01_Motor 6
#define Max_01_Motor 7

//contatore encoder
volatile long encoder1Count = 0;

//variabili controllo PID
long previous1Time = 0;
float e1Previous = 0;
float e1Integral = 0;
int Motore1 = 0;
int target1 = 0;

// Stringhe usate comunicazione seriale
String stringa = "";
String stringa1 = "";

void setup() {
Serial.begin(9600);
// setp pin mode
pinMode(DIR1, OUTPUT);
pinMode(PWM1, OUTPUT);
pinMode(encoder1PinA, INPUT);
pinMode(encoder1PinB, INPUT);
pinMode(A0, INPUT); // in caso si voglia comandare la slitta con Joystick
pinMode(encoder1PinA, INPUT);
pinMode(encoder1PinB, INPUT);
pinMode(6, INPUT);
pinMode(7, INPUT); 

// interrupts per encoder
attachInterrupt(digitalPinToInterrupt(encoder1PinA),handle1Encoder,RISING);

//Cerca Zero Motore 1
while (digitalRead(Min_01_Motor)) {
digitalWrite(4, 0);
analogWrite(5, 50);
}
digitalWrite(4, 0);
analogWrite(5, 0);
delay(80);
digitalWrite(4, 1);
analogWrite(5, 30);
delay(500);
analogWrite(5, 1);
encoder1Count = 0;
}

void loop() {
if (digitalRead(Max_01_Motor)){
 if (Serial.available() > 0) {     // controlliamo che ci siano dati disponibili sulla seriale
  
    String command = Serial.readStringUntil('\n');  // leggo la stringa ricevuta via seriale fino al ritorno a capo
  
    // Trova l'indice del comando Motore
    
    int indexX = command.indexOf('X');  // creo la variabile indice con all'intero la posizione delle lettere X nella stringa ricevuta via seriale
    int indexE = command.indexOf('E'); // comando emergenza
 


    // Controllo Motore1
    if (indexX != -1 && isDigit(command.charAt(indexX + 1)) && isDigit(command.charAt(indexX + 2)) && isDigit(command.charAt(indexX + 3))) {  //verifichiamo che dopo X ci siano 3 numeri consecuitivi X000 - X180
      Motore1 = (command.substring(indexX + 1, indexX + 4).toInt());    // assegno al Motore1 il valore di spostamento X000-X999
    }
    
    while (indexE != -1 ) {  
    analogWrite(5, 0);  // ferma Motore
     } 
  }

// timer che manda la stringa di controllo ogni 200 ms
if (mytimer(200)) {
//Serial.flush();
stringa = 'A' + String(target1)  + ('F');
stringa1 = "a" + String(encoder1Count);
stringa = stringa + stringa1;
Serial.println(stringa);
stringa = "";
stringa1 = "";
}
//setpoint
target1 =  (10 * (Motore1)); // questo è il motivo per cui nella stringa di risposta il valore è moltiplicato per 10

//guadagno PID e calcolo
float kp1 = 1;
float kd1 = 0.1;
float ki1 = 0;
float u1 = pid1Controller(target1,kp1,kd1,ki1);

//muovi motore
moveMotor1(DIR1, PWM1, u1);
}
else {
analogWrite(5, 0); // stoppa il motore se finecorsa Max_01_Motor attivato
 }
}

// muovi motori
void moveMotor1(int dir1Pin, int pwm1Pin, float u1){
// velocità massima motore 1
float speed1 = fabs(u1); // valore assoluto di un numero float
if (speed1 > 250){
  speed1 = 250;
}
// set direzione
int direction1 = 1;
if (u1 < 0){
  direction1 = 0;
}
digitalWrite(dir1Pin, direction1);
analogWrite(pwm1Pin, speed1);
}

float pid1Controller(int target1,float kp1, float kd1, float ki1){
  // misuro il tempo trascorso dall'ultima iterazione
  long current1Time = micros();
  float deltaT = ((float)(current1Time - previous1Time)) / 1.0e3;
  // calcolo errore,derivata e integrale
  int e1 = + target1 - encoder1Count;
  float e1Derivative =(e1 - e1Previous) / deltaT;
  e1Integral =  e1Integral + e1 * deltaT;
  // calcola segnale PID
  float u1 = (kp1 * e1) + (kd1 * e1Derivative) + (ki1 * e1Integral);
  //aggiorna variabili per la prossima interazione
  previous1Time = current1Time;
  e1Previous = e1;
  return u1;
}

void handle1Encoder(){
  if(digitalRead(encoder1PinA)>digitalRead(encoder1PinB)){
    encoder1Count ++;
  }
  else{
    encoder1Count--;
  }
}

int mytimer (int timer1) {
  static unsigned long t1, dt;
  int ret = 0;
  dt = millis() - t1;
  if (dt > 200) {
      t1 = millis();
      ret = 1;
  }
  return ret;
}
