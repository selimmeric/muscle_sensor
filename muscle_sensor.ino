// A0 - Muscle sensörün
#include <Wire.h>
// Adafruit_PWMServoDriver.h sürücüsü burda yazmasada https://github.com/adafruit/Adafruit_BusIO adresinden Adafruit_BusIO nunda kurulmasını ister.
#include "src/Adafruit_PWM_Servo_Driver/Adafruit_PWMServoDriver.h" // https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
#include "src/SimpleKalmanFilter/src/SimpleKalmanFilter.h" // https://github.com/denyssene/SimpleKalmanFilter
#include "src/tinyCommand/src/tinyCommand.hpp" // https://github.com/chrmlinux/tinyCommand

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOCOUNT 5 
#define SERVOMIN  50 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  520 // This is the 'maximum' pulse length count (out of 4096)
int PRINTINTERVAL = 50; // Ekrana basmak için beklenecek döngü sayısı
// servoInterval * PRINTINTERVAL kadar mili saniye bekliyor

//------- VARIABLES (will change)

int servoPosition[] = { 0, 0, 0, 0, 0 };
int servoTargetPosition[] = { 0, 0, 0, 0, 0 };

int servoMinPosition[] = { 0, 0, 0, 0, 0 };
int servoMaxPosition[] = { 180, 180, 180, 180, 180 };
int servoPositionInv[] = { 0, 0, 1, 0, 0 }; // posizyon tersleme olacakmı ? 0 ise normal, 1 ise motorun yönü ters olacak.
// şu anki durum
// motor indis 0 - baş parmak
// motor indis 1 - işaret parmagı
// motor indis 2 - küçük parmak
// motor indis 3 - Orta  parmak
// motor indis 4 - yüzük parmagı
int servoSlowInterval = 80;  // millisecs between servo moves
int servoFastInterval = 15;

int servoInterval = servoFastInterval;  // initial millisecs between servo moves
int servoDegrees = 2;                   // amount servo moves at each step
                                        //    will be changed to negative value for movement in the other direction

unsigned long currentMillis = 0;        // stores the value of millis() in each iteration of loop()
unsigned long previousServoMillis = 0;  // the time when the servo was last moved

static tinyCommand cmd(Serial);
// Kalman filtresi
long muscle_raw_value=0; 
float estimated_muscle_value=0;

 SimpleKalmanFilter KalmanFilter(1, 1, 0.01);
// Global command variable.
int analogSeriOut = 0;
int servoSerialOut = 1;



void setup() {
  Serial.begin(9600);
  Serial.println("Robot El");  // so we know what sketch is running

  // PWM sürücü devreye alınıyor
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency

  Wire.setClock(400000); // I2C hızı ayarlanıyor

  for (int n = 0; n < SERVOCOUNT; n++) {
      pwm.setPWM(n, 0, SERVOMIN);
  }

  cmd.begin();
  cmd.setCmd("spos", set_pos);
  cmd.setCmd("gpos", get_pos);
  cmd.setCmd("param", set_param);
  cmd.setCmd("help", help);

  help();
  delay(1000);
}
void loop() {
  cmd.scan();
  currentMillis = millis();  // capture the latest value of millis()
  servoSweep();
  muscle_raw_value = analogRead(A0); // sensör verisi
  estimated_muscle_value = KalmanFilter.updateEstimate(muscle_raw_value); // filtrelenmiş sensör verisi
}
void help(){
    Serial.println("Terminal Komut Listesi");
    Serial.println("**********************");
    Serial.println("help : bu listeyi görürsün");
    Serial.println("spos p1 p2 p3 p4 p5 : servo pozisyonlama komutu");
    Serial.println("     ----------------");
    Serial.println("                      p1 : Birinci  motor açısal degeri 0 - 180");
    Serial.println("                      p2 : İkinci   motor açısal degeri 0 - 180");
    Serial.println("                      p3 : Üçüncü   motor açısal degeri 0 - 180");
    Serial.println("                      p4 : Dördüncü motor açısal degeri 0 - 180");
    Serial.println("                      p5 : Beşinci  motor açısal degeri 0 - 180");
    Serial.println("gpos : servo pozisyonlarını listeler");
    Serial.println("param p1 p2 : parametreleri degiştirir");
    Serial.println("      -------");
    Serial.println("              p1 : a,s veya i verilebilir a = analog veri, s = servo verisi");
    Serial.println("                   i = interval verisi, ekrana gelecek veri hızı");
    Serial.println("              p2 : 0 dan farklı bir deger olursa aktif olur. 0 kapatir");
    Serial.println("                   interval verisinde çarpan olarak kullanılır.");
    Serial.println("**********************");
}
void serialOut(){
    Serial.print("256,100,");
    Serial.print(muscle_raw_value);
    Serial.print(",");
    Serial.println(estimated_muscle_value);
}
int16_t set_param(int argc, char **argv) {
  int stat = 1;
    if (argc > 2){
      if (strncmp("a", argv[1], sizeof(argv[1])) == 0) {
        stat = atoi(argv[2]);
        analogSeriOut = stat;
      }
      if (strncmp("s", argv[1], sizeof(argv[1])) == 0) {
        stat = atoi(argv[2]);
        servoSerialOut = stat;
      }
      if (strncmp("i", argv[1], sizeof(argv[1])) == 0) {
        stat = atoi(argv[2]);
        PRINTINTERVAL = stat;
      }
    }
  return 0;
}

int16_t set_pos(int argc, char **argv) {

  int stat = 1;
  for (int n = 1; n <= SERVOCOUNT; n++) {
    if (argc > n) {
      stat = atoi(argv[n]);
      servoTargetPosition[n-1] = stat;
    }
  }
  return 0;
}
int16_t get_pos(int argc, char **argv) {
   infox();
   return 0;
}
void infox(){
  Serial.print("positions : ");
  for (int n = 0; n < SERVOCOUNT; n++) {
    Serial.print(servoPosition[n]);
    Serial.print(" - ");
    Serial.print(servoTargetPosition[n]);
    Serial.print("\t");
  }
  Serial.println();
}
int cntTm = 0;
void servoSweep() {
  if (currentMillis - previousServoMillis >= servoInterval || currentMillis < previousServoMillis) {
    previousServoMillis += servoInterval;
    for (int n = 0; n < SERVOCOUNT; n++) {
      if(servoTargetPosition[n]!=servoPosition[n]){
        if(servoTargetPosition[n]>servoPosition[n]){
          servoPosition[n]+=servoDegrees;
          if(servoTargetPosition[n]<servoPosition[n])
            servoPosition[n] = servoTargetPosition[n];
        }
        else {
          servoPosition[n]-=servoDegrees;
          if(servoTargetPosition[n]>servoPosition[n])
            servoPosition[n] = servoTargetPosition[n];
        }
      }
      // motorların alt üst limitleri dereceye çevriliyor
      long output = map(servoPositionInv[n]==0?servoPosition[n]:180-servoPosition[n], servoMinPosition[n], servoMaxPosition[n],SERVOMIN,SERVOMAX);
      pwm.setPWM(n, 0, output );
    }
    if(cntTm > PRINTINTERVAL){
      if(analogSeriOut>0){
        serialOut();
      }
      if(servoSerialOut>0){
        infox(); // durum pozisyonu ekrana bastırılıyor
      }
     cntTm=0;
    }else{
      cntTm++;
    }
  }
}