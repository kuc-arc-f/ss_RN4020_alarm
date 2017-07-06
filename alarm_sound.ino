#include "pitches.h"

#define BEATTIME 400   //sound Output  (msec)
// #define mSPEAKER 7

//void setup() {

//}
//
void temp_LowSound(){
  tone(mSPEAKER,  NOTE_C4,BEATTIME) ; // do
  delay(BEATTIME);
  tone(mSPEAKER,  NOTE_C4,BEATTIME) ; // do
  delay(BEATTIME);
  tone(mSPEAKER,  NOTE_B3 ,BEATTIME) ; // Si
  delay(BEATTIME);
  tone(mSPEAKER,  NOTE_C4,BEATTIME) ; // do
  delay(BEATTIME);
}
//
void temp_high_Sound(){
    for(int i=0; i<2; i++){
        tone(mSPEAKER,  NOTE_A6 ,BEATTIME) ; // ra
        delay(BEATTIME);
        tone(mSPEAKER,  NOTE_B6 ,BEATTIME) ; // si
        delay(BEATTIME);
        tone(mSPEAKER,  NOTE_C7 ,BEATTIME) ; // do
        delay(BEATTIME);
    }
}
  


