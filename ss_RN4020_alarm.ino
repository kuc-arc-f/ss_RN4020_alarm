
/* 
  SoftwareSerial, RN4020 BLE (Low Power version)
   , with alarm Sound
 *  v 0.9.5
*/
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(5, 6); /* RX:D5, TX:D6 */
String mBuff="";
const int mVoutPin = 0;
const int mPinBLE =8;
const int mPinLED =2;
const int mSPEAKER =7;

const int mOK_CODE=1;
const int mNG_CODE=0;
uint32_t mTimer_runMax= 0;
const int mNextSec   = 300; //Sec
const int mMax_runSec= 20; //Sec
const int mMax_BLE   = 5; //Sec

const int mMode_RUN  = 1;
const int mMode_WAIT = 2; 
int mMode =0;
//int mAlarmStat=0;
int mTempBefore=0;

// const char mDevice_name[3+1]="D11";
const char mDevice_name[3+1]="D12";

// LOW power
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
volatile int wdt_cycle;

//
long convert_Map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//
// reading LM60BIZ
int getTempNum(){
  int iRet=0;
//  float fSen  = 0;
  unsigned long reading  = 0;   
  for (int i=0; i<10; i++) {
    int  iTmp = analogRead(mVoutPin);
//Serial.print("get.iTmp=");
//Serial.println(iTmp);
    reading  += iTmp; 
    delay(100);
  }
  int SValue= reading / 10;
  int voltage=convert_Map(SValue, 0, 1000, 0,3300);  // V
Serial.println("SValue="+ String(SValue)   +" , voltage="  +String(voltage ) );
  int iTemp = (voltage - 424) / 6.25; //電圧値を温度に変換, offset=425
  iRet= iTemp;
  
  return iRet;  
}
//
int Is_resWait(String value, uint32_t maxMsec ){
  int ret= mNG_CODE;
  uint32_t tm_maxWait=millis();
  int iLen= value.length();
  String sBuff="";
  int iBool=1;
    while(iBool ){
        while( mySerial.available() ){
            char c= mySerial.read();
            Serial.print( c );
            if( (c != 0x0a ) && (c != 0x0d ) ){
                sBuff.concat(c );
            }            
        } //end_while
        if(  (int )sBuff.length() >= iLen ){ iBool=0;  }      
        delay(100);
        uint32_t tmMax =millis() -tm_maxWait;
        if(tmMax > (uint32_t)maxMsec ){ 
          Serial.println("#Error-Is_resWait:maxSec!");
          iBool=0; 
        } 
    }//end_while_1
    Serial.println("");
    if(sBuff.length() < 1){ return ret; }
    int iLenBuff= sBuff.length();
    int iSt=iLenBuff -(iLen); 
    Serial.println("iLenBuff="+ String(iLenBuff)+",iSt="+ String(iSt) ) ;
    String sRecv = sBuff.substring(iSt   , iLen );
    Serial.println( "sBuff="+ sBuff+ " ,sRecv="+ sRecv );
    if(sRecv == value ){
      ret= mOK_CODE;
    }
  return ret;  
}
//
void proc_getSSerial(){
  while( mySerial.available() ){
    char c= mySerial.read();
Serial.print( c );
  } //end_while
}

int mCounter=0;
//
//void proc_sendCmd(){
void proc_sendCmd(int iSenNum){
  int iWaitTm= 5000;
  char cBuff[24 +1];
  //int iSenNum= getTempNum();
  Serial.println("iSenNum=" + String(iSenNum) );
  //sprintf(cBuff , "SN,D11%06d000001\r", iSenNum );
  sprintf(cBuff , "SN,%s%06d000001\r", mDevice_name,iSenNum );
  // mDevice_name
  //SN
  Serial.println( "#Start SN, cBuff="+  String(cBuff)  );
   mySerial.print(String(cBuff) );
   if( Is_resWait("AOK", iWaitTm ) == mNG_CODE ){ return; }
   else{Serial.println("#OK SN" ); };
   proc_getSSerial();
//   wait_forSec(3);
   //R
   Serial.println( "#Start R" );
   mySerial.print("R,1\r");
   delay(1000);
   if( Is_resWait("RebootCMD", iWaitTm ) == mNG_CODE ){ return; }
   else{Serial.println("#OK R1" ); };
   proc_getSSerial();
   //wait_forSec(3);
   
   //A
   Serial.println( "#Start A" );
//   mySerial.print("A\r");
   mySerial.print("A,0064,07D0\r");    //100msec, total= 2000mSec
  if( Is_resWait("AOK", iWaitTm ) == mNG_CODE ){ return; }
   else{Serial.println("#OK A" ); };
   proc_getSSerial();
   mCounter= mCounter+1;
   wait_forSec(3 );
}

//
void wait_forSec(int wait){
   for(int i=0; i<wait; i++){
    delay(1000);
    Serial.println("#wait_forMsec: "+ String(i) );    
  }
}
//
void setup() {
  mMode = mMode_RUN;
  pinMode(mPinBLE ,OUTPUT);
  pinMode(mSPEAKER, OUTPUT);
  pinMode(mPinLED, OUTPUT);
  Serial.begin( 9600 );
  mySerial.begin( 9600 );
  Serial.println("#Start-setup-SS");
  setup_watchdog(6 );                    // WDT setting
  //wait
  wait_forSec(3);
  proc_getSSerial(); //cear-buff
  // mTimer_runMax =  ((uint32_t)mMax_runSec * 1000) + millis();
}
//
void play_sound(int iSenNum){
  // printf("dTemp=%f,iNowTemp=%d \n",dTemp , iNowTemp);
  Serial.println("dTemp="+ String(mTempBefore) + ",iSenNum =" + String(iSenNum )  );
  if(iSenNum > ((double)mTempBefore + 0.5  )){
      Serial.println("#temp=H"   );
      temp_high_Sound();
  }else if(iSenNum < ((double)mTempBefore - 0.5 )  ){
      Serial.println("#temp=L"   );
      temp_LowSound();
  }else{
      Serial.println("temp is same value."   );
  }
}
//
void loop() {
  //const int iWait =2000;
//  delay(100);
//  Serial.println( "mTimer="+ String(mTimer) + ",millis=" + String(millis()) );
  if(mMode ==mMode_RUN){
      digitalWrite(mPinLED, HIGH);
      if(mTimer_runMax <= 0 ){
          mTimer_runMax =  ((uint32_t)mMax_runSec * 1000) + millis();
          digitalWrite(mPinBLE, HIGH);
          delay(100 );
      }
      int iSenNum= getTempNum();
      if(millis() < mTimer_runMax){
          digitalWrite(mPinBLE, HIGH);
          proc_sendCmd(iSenNum );
      }else{
          mTimer_runMax=0;
          digitalWrite(mPinBLE,LOW  );
          play_sound(iSenNum );
          mTempBefore =iSenNum;
          digitalWrite(mPinLED, LOW);
          delay( 500);
          mMode = mMode_WAIT;
      }
  }else{
    mMode = mMode_RUN;
    for(int i=0; i< mNextSec ; i++){
        system_sleep();                       // power down ,wake up WDT 
        //Serial.print("i=");
        //delay(15);
        //Serial.println(i);
        //delay(15);
    }
  }
  
}

//
void system_sleep() {
  cbi(ADCSRA,ADEN);                     // ADC power off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // power down mode
  sleep_enable();
  sleep_mode();                         // sleep
  sleep_disable();                      // WDT time up
  sbi(ADCSRA,ADEN);                     // ADC ON
}

// WDT setting, param : time
// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {           // 
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}  //setup_watchdog

ISR(WDT_vect) {                         // WDT, time Up process
  wdt_cycle++;           
}







