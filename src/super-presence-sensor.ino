
#include "Radar/Radar.h"
MR24HPB1 radar(Serial1,18,19);
// // some example callbacks
// void unocc(){Serial.println("UNOCCUPIED");}
// void occ(){Serial.println("OCCUPIED");}
// void moti(){Serial.println("MOTION");}
// void stat(){Serial.println("STATIONARY");}
// void away(uint8_t state){
//   switch(state){
//     case 0x01:
//       Serial.println("NONE"); break;
//     case 0x02:
//       Serial.println("CLOSE_TO"); break;
//     case 0x03:
//       Serial.println("STAY_AWAY"); break;
//     default:
//       Serial.println("INVALID"); break;
//   }
// }
// void env(uint8_t state){
//   Serial.print("ENVIRONMENT: ");
//   Serial.println(state);
// }
// void mot(float s){
//     Serial.print("MOTOR: ");
//   Serial.println(s);
// }

// // simple test commands to set the sensitivity and scene setting t# will set the Threshold to the #, s# will set the scene setting. e.g t7 will set the threshold to 7 (which is the sensor default)
// void parseserial(){
//   if(Serial1.available() >0){
   
//         Serial.print("c");
//         char cmd = Serial1.read();
//         switch(cmd){
//           case 't': {
//             Serial.print("t");
//             uint8_t dat = Serial1.parseInt();
//             Serial.println(dat);
//             radar.setThreshold(dat);
//             return;
//           }
//          case 's':{
//             Serial.print("s");
//             uint8_t dat = Serial1.parseInt();
//             Serial.println(dat);
//             radar.setSceneSetting((scene_setting_t)dat);
//             return;
//           }
         
//         }
    
//   }
// }

// long prev =0;

#include "Arduino.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  pinMode(D9, INPUT);
  pinMode(D10, INPUT);
  attachInterrupt(digitalPinToInterrupt(D9), interruptD9, CHANGE);
  attachInterrupt(digitalPinToInterrupt(D10), interruptD10, CHANGE);
  // radar.begin(2,(scene_setting_t) 3);
  // radar.register_on_unoccupied(unocc);
  // radar.register_on_occupied(occ);
  // radar.register_on_stationary(stat);
  // radar.register_on_movement(moti);
  // radar.register_on_environmental_state(env);
  // radar.register_on_away_state(away);
  // radar.register_on_motor_signs(mot);
}

void interruptD9() {
  Serial.println('D9 INTERRUPT');
}
void interruptD10() {
  Serial.println('D10 INTERRUPT');
}

void loop() {
  // put your main code here, to run repeatedly:
  // parseserial();
  // radar.refresh();
  // long now = millis();
  // if(radar.getUpdatedMemberType() == 0x0C){
  //      Serial.println("Got a new Threshold!");
  //      delay(5000);
  // }
  
  delay(1000);
}