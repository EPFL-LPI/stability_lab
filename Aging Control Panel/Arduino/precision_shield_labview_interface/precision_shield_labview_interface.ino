#include <stdint.h>
#include <SPI.h>


#define CSB_PIN 7
#define A0_PIN 6
#define A1_PIN 5
#define MODE_PINA 8
#define RANGE_PINA 9
#define MODE_PINB 10
#define RANGE_PINB 4
/* calibration settings for board SN000 (LPKF prototype) 
#define ADC2_OFFSET 1.50
#define ADC3_OFFSET 0.26
#define DAC1_OFFSET -0.25
#define DAC2_OFFSET 0.15
#define DAC3_OFFSET 0.29

#define ADC_MV_PER_BIT 7.629395e-2
#define RTD_OFFSET -0.225
#define RTD_MV_PER_BIT 3.814697e-2
#define DAC_BITS_PER_MV 13.1072

/**/
/* calibration settings for board SN001 */
/* based on estimated (internal) ref voltages of 2.498 and 4.902 */
/* measured (external) ref voltages are 2.500 and 4.904 */
/*
#define ADC2_OFFSET 2.29
#define ADC3_OFFSET 0.76
#define ADC_MV_PER_BIT 7.479858e-2
#define RTD_OFFSET -0.225
#define RTD_MV_PER_BIT 3.814544e-2
#define DAC1_OFFSET -3.015
#define DAC2_OFFSET -2.695
#define DAC3_OFFSET -2.857
#define DAC_BITS_PER_MV 13.1177
*/
/* calibration settings for board SN002 */
/* based on estimated (internal) ref voltages of 2.498 and 4.902 */
/* measured (external) ref voltages are 2.5006 and 4.909 */
/*
#define ADC2_OFFSET 1.19
#define ADC3_OFFSET 0.76
#define ADC_MV_PER_BIT 7.479858e-2
#define RTD_OFFSET -0.225
#define RTD_MV_PER_BIT 3.814544e-2
#define DAC1_OFFSET -1.251
#define DAC2_OFFSET -1.320
#define DAC3_OFFSET -1.283
#define DAC_BITS_PER_MV 13.1177
*/
/* calibration settings for board SN003 */
/* based on estimated (internal) ref voltages of 2.498 and 4.902 */
/* measured (external) ref voltages are 2.5006 and 4.909 */

#define ADC2_OFFSET 1.19
#define ADC3_OFFSET 0.76
#define ADC_MV_PER_BIT 7.432556e-2 //7.479858e-2 old value based on 5V.  new value based on 4.871V
#define RTD_OFFSET -1.0
#define RTD_MV_PER_BIT 3.814544e-2
#define DAC1_OFFSET -1.251
#define DAC2_OFFSET -1.320
#define DAC3_OFFSET -1.283
#define DAC_BITS_PER_MV 13.1177
/**/

byte wait_for_input(int time_to_wait);
void select_channel(byte channel);
float precision_read(float offset, int adc_type);
float precision_read_hires(float offset, int adc_type);
void precision_write(byte channel, unsigned int value);
float read_RTD();
float read_RTD_hires();
void PAS_power_down();
void PAS_wake_up();
void measure_noise_and_offset();

unsigned long rval = 0;
unsigned long rval_readback = 0;

unsigned int data_array[64];
unsigned long data_mean_raw = 0;
float mean = 0.;
float data_point_f = 0.;
float rms_accumulator = 0.;
float rms = 0.;
byte DAC_counter = 0;
byte incomingByte = 0;
String incomingString = "";
byte current_chan = 0;
float channel_offset = 0.;
int i, j, k;
byte DAC_channel = 0;
float input_multiplier = 0.;
float DAC_voltage = 0.;
float DAC_offset = 0.;
unsigned int DAC_word = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(CSB_PIN, OUTPUT);
  digitalWrite(CSB_PIN,HIGH);
  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  digitalWrite(A0_PIN,LOW);
  digitalWrite(A1_PIN,LOW);
  pinMode(MODE_PINA,OUTPUT);
  pinMode(RANGE_PINA,OUTPUT);
  pinMode(MODE_PINB,OUTPUT);
  pinMode(RANGE_PINB,OUTPUT);
  digitalWrite(MODE_PINA,HIGH);
  digitalWrite(RANGE_PINA,HIGH);
  digitalWrite(MODE_PINB,HIGH);
  digitalWrite(RANGE_PINB,HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE2));
  
  delay(100);

}

void loop() {

  /*
  Serial.print("Choose an operation:\n\r");
  Serial.print("1) Set current channel\n\r");
  Serial.print("2) Read voltage from channel\n\r");
  Serial.print("3) Read temperature from channel\n\r");
  Serial.print("4) Set voltage on channel\n\r");
  Serial.print("5) Read Hi-res voltage from channel\n\r");
  Serial.print("6) Read Hi-res temperature from channel\n\r");
  Serial.print("7) Power down current channel\n\r");
  Serial.print("8) Wake up current channel\n\r");
  Serial.print("9) Set Regulation Mode\n\r");
  Serial.print("10) Set Regulation Range\n\r");
  */
  incomingByte = wait_for_input(-1);

  // say what you got
  /*
  Serial.print((char)incomingByte);
  if(incomingByte == (byte)'\r') {
     Serial.print("\n");
  }
  Serial.print("\n\r");
  */
  switch (incomingByte) {
    case 0x31:
      //Serial.print("enter channel number: ");
      incomingByte = wait_for_input(-1);
      if(incomingByte > 0x2F && incomingByte < 0x34) {
        current_chan = incomingByte-0x30;
        if(current_chan == 2) channel_offset = ADC2_OFFSET;
        else if(current_chan == 3) channel_offset = ADC3_OFFSET;
        else channel_offset = RTD_OFFSET;
        select_channel(incomingByte-0x30);
        Serial.print("$\r\n");
      }
      else Serial.print("?\r\n"); //\n\rERROR: invalid input\n\r");
      break;
    case 0x32:
      //Serial.print("\n\r");
      if(current_chan == 1) Serial.print(precision_read(channel_offset,1));
      else Serial.print(precision_read(channel_offset,0));
      Serial.print("\r\n");
      //Serial.print("mV\n\r");
      break;
    case 0x33:
      //Serial.print("\n\r");
      Serial.print(read_RTD());
      Serial.print("\r\n");
      //Serial.print((char)167);
      //Serial.print("C\n\r");
      break;
    case 0x35:
      //Serial.print("\n\r");
      if(current_chan == 1) Serial.print(precision_read_hires(channel_offset,1));
      else Serial.print(precision_read_hires(channel_offset,0));
      Serial.print("\r\n");
      //Serial.print("mV\n\r");
      break;
    case 0x36:
      //Serial.print("\n\r");
      Serial.print(read_RTD_hires());
      Serial.print("\r\n");
      //Serial.print((char)167);
      //Serial.print("C\n\r");
      break;
    case 0x34:
      //Serial.print("Select DAC channel (1 or 2, 3 for both): ");
      incomingByte = wait_for_input(-1);
      if(incomingByte > 0x30 && incomingByte < 0x34) {
        DAC_channel = incomingByte-0x30;
        Serial.print("$\r\n");
      }
      else Serial.print("?\r\n"); //\n\rERROR: invalid input\n\r");
      //Serial.print("\r\nenter voltage (0-5000mV, format xxxx.xx): ");
      incomingByte = wait_for_input(-1);
      //echo the input to the terminal
      //Serial.print((char)incomingByte);
      i=0;
      incomingString = "";
      while(incomingByte != (byte)'\r') {
        //accumulate input
        incomingString += (char)incomingByte;
        incomingByte = wait_for_input(-1);
        //echo the input to the terminal
        //Serial.print((char)incomingByte);
        i++;
      }
      DAC_voltage = incomingString.toFloat();
      if(DAC_channel == 1) DAC_offset = DAC1_OFFSET;
      else if(DAC_channel == 2) DAC_offset = DAC2_OFFSET;
      else if(DAC_channel == 3) DAC_offset = DAC3_OFFSET;
      DAC_voltage -= DAC_offset;
      if(DAC_voltage < 0.) DAC_voltage = 0.;
      if(DAC_voltage > (6.5636e4/DAC_BITS_PER_MV)- DAC_offset - 1) DAC_voltage = (6.5636e4/DAC_BITS_PER_MV)- DAC_offset - 1;

      /*
      Serial.print("\n\rSetting DAC channel ");
      Serial.print(DAC_channel);
      Serial.print(" to ");
      Serial.print(DAC_voltage);
      Serial.print("mV\r\n");
      */
      
      DAC_word = (unsigned int)(DAC_voltage*DAC_BITS_PER_MV);

      precision_write(DAC_channel,DAC_word);
      Serial.print("$\r\n");
      break;
    case 0x37:
      PAS_power_down();
      break;
    case 0x38:
      PAS_wake_up();
      break;
    case 0x39:
      //Serial.print("\n\r");
      //Serial.print("enter regulation mode (0=light, 1=current): ");
      incomingByte = wait_for_input(-1);
      if(incomingByte > 0x2F && incomingByte < 0x34) {
        if(incomingByte == 0x30 || incomingByte == 0x32) digitalWrite(MODE_PINA,LOW);
        if(incomingByte == 0x31 || incomingByte == 0x33) digitalWrite(MODE_PINA,HIGH);
        if(incomingByte == 0x30 || incomingByte == 0x31) digitalWrite(MODE_PINB,LOW);
        if(incomingByte == 0x32 || incomingByte == 0x33) digitalWrite(MODE_PINB,HIGH);
        Serial.print("$\r\n");
      }
      else Serial.print("?\r\n"); //\n\rERROR: invalid input\n\r");
      break;
    case 0x61: //'a'
      //Serial.print("\n\r");
      //Serial.print("enter regulation range (0=high light, 1=low light): ");
      incomingByte = wait_for_input(-1);
      if(incomingByte > 0x2F && incomingByte < 0x34) {
        if(incomingByte == 0x30 || incomingByte == 0x32) digitalWrite(RANGE_PINA,LOW);
        if(incomingByte == 0x31 || incomingByte == 0x33) digitalWrite(RANGE_PINA,HIGH);
        if(incomingByte == 0x30 || incomingByte == 0x31) digitalWrite(RANGE_PINB,LOW);
        if(incomingByte == 0x32 || incomingByte == 0x33) digitalWrite(RANGE_PINB,HIGH);
        Serial.print("$\r\n");    
      }
      else Serial.print("?\r\n"); //\n\rERROR: invalid input\n\r");
      break;
    default:
      break;
  }

  //Serial.print("\n\r");
  //delay(10);
  
}

byte wait_for_input(int time_to_wait) {
  int i = 0;
  while(Serial.available() == 0) {
      if(time_to_wait == -1) ; //no exit case - wait forever
      else if(i == time_to_wait) break;
      else {
        delay(1); //wait a bit
        i++;
      }
  }
  // read the incoming byte:
  return Serial.read();
}

void select_channel(byte channel) {

  digitalWrite(A0_PIN,channel&0x01);
  digitalWrite(A1_PIN,(channel&0x02)>>1);
    
}

float precision_read(float offset, int adc_type) {

  byte data_byte = 0;
  byte data_byte1 = 0;
  byte data_byte2 = 0;
  byte data_byte3 = 0;
  unsigned int data_word = 0;

  digitalWrite(CSB_PIN,LOW);
  data_byte1 = SPI.transfer(0x00);
  data_byte2 = SPI.transfer(0x00);
  data_byte3 = SPI.transfer(0x00);
  digitalWrite(CSB_PIN,HIGH);

  if(adc_type == 0) {
      data_word = data_byte1;
      data_word = data_word << 8;
      data_word = data_word | data_byte2;
      data_word = data_word << 4;
      data_word = data_word | (data_byte3 >> 4);
    }else {
      data_word = data_byte1;
      data_word = data_word << 8;
      data_word = data_word | data_byte2;
      data_word = data_word << 7;
      data_word = data_word | (data_byte3 >> 1);
    }

  //Serial.print(data_word);
  //Serial.print("\n\r");

  if(adc_type == 0) return (float)data_word*ADC_MV_PER_BIT - offset;
  else return (float)data_word*RTD_MV_PER_BIT - offset;
}

float precision_read_hires(float offset, int adc_type) {

  byte data_byte = 0;
  byte data_byte1 = 0;
  byte data_byte2 = 0;
  byte data_byte3 = 0;
  unsigned int data_word = 0;
  unsigned long data_word_long = 0;

  for(i=0; i<64; i++) {
    digitalWrite(CSB_PIN,LOW);
    data_byte1 = SPI.transfer(0x00);
    data_byte2 = SPI.transfer(0x00);
    data_byte3 = SPI.transfer(0x00);
    digitalWrite(CSB_PIN,HIGH);

    if(adc_type == 0) {
      data_word = data_byte1;
      data_word = data_word << 8;
      data_word = data_word | data_byte2;
      data_word = data_word << 4;
      data_word = data_word | (data_byte3 >> 4);
    }else {
      data_word = data_byte1;
      data_word = data_word << 8;
      data_word = data_word | data_byte2;
      data_word = data_word << 7;
      data_word = data_word | (data_byte3 >> 1);
    }
    data_word_long += data_word;
    delayMicroseconds(251); //each cycle takes 61us. this way 64 samples takes 20ms - 1 period of 50Hz
  }

  //Serial.print(data_word);
  //Serial.print("\n\r");

  //scale by 5000/(64 * 2^16)
  if(adc_type == 0) return (float)data_word_long*ADC_MV_PER_BIT*0.015625 - offset;
  else return (float)data_word_long*RTD_MV_PER_BIT*0.015625 - offset;
}

void precision_write(byte channel, unsigned int value) {
  byte update_cmd = 0x30;
  update_cmd = update_cmd | channel&0x01; //channel 0 selected
  update_cmd = update_cmd | (channel<<2)&0x08; //channel 1 selected
  digitalWrite(CSB_PIN,LOW);
  SPI.transfer(update_cmd); //update selected DACs
  SPI.transfer((byte)(value>>8)); //MSB
  SPI.transfer((byte)(value&0x00FF)); //LSB
  digitalWrite(CSB_PIN,HIGH);
  
}

float read_RTD() {

  byte data_byte = 0;
  byte data_byte1 = 0;
  byte data_byte2 = 0;
  byte data_byte3 = 0;
  unsigned int data_word = 0;
  float voltage = 0.;
  float temperature = 0.;

  digitalWrite(CSB_PIN,LOW);
  data_byte1 = SPI.transfer(0x00);
  data_byte2 = SPI.transfer(0x00);
  data_byte3 = SPI.transfer(0x00);
  digitalWrite(CSB_PIN,HIGH);

  data_word = data_byte1;
  data_word = data_word << 8;
  data_word = data_word | data_byte2;
  data_word = data_word << 7;
  data_word = data_word | (data_byte3 >> 1);

  //Serial.println(data_word,HEX);

  voltage = (float)data_word*RTD_MV_PER_BIT - RTD_OFFSET;
  temperature = (voltage - 100.)/0.39;

  return temperature;
  
}

float read_RTD_hires() {

  byte data_byte = 0;
  byte data_byte1 = 0;
  byte data_byte2 = 0;
  byte data_byte3 = 0;
  unsigned int data_word = 0;
  unsigned long data_word_long = 0;
  float voltage = 0.;
  float temperature = 0.;

  for(i=0; i<64; i++) {
    digitalWrite(CSB_PIN,LOW);
    data_byte1 = SPI.transfer(0x00);
    data_byte2 = SPI.transfer(0x00);
    data_byte3 = SPI.transfer(0x00);
    digitalWrite(CSB_PIN,HIGH);

    data_word = data_byte1;
    data_word = data_word << 8;
    data_word = data_word | data_byte2;
    data_word = data_word << 7;
    data_word = data_word | (data_byte3 >> 1);

    data_word_long += data_word;
    delayMicroseconds(251); //312.5us minus 61us for readout cycle.  64 samples takes 20ms this way - 1 period of 50Hz
  }
  //Serial.println(data_word,HEX);

  //scale by 2496.5/(64 * 2^16)
  voltage = (float)data_word_long*RTD_MV_PER_BIT*1.5625e-2 - RTD_OFFSET;
  //voltage = (float)data_word_long*5.9521198e-4;
  temperature = (voltage - 100.)/0.39;

  return temperature;
  
}

void PAS_power_down() {

  digitalWrite(CSB_PIN,LOW);
  SPI.transfer(0x00);
  digitalWrite(CSB_PIN,HIGH);
  
}

void PAS_wake_up() {

  digitalWrite(CSB_PIN,LOW);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  digitalWrite(CSB_PIN,HIGH);
  
}

void measure_noise_and_offset() {
/* routine for measuring noise and offset of input channels.
 * 
  data_mean_raw = 0;
  mean = 0.;
  rms_accumulator = 0.;
  rms = 0.;

  for(int i=0; i<64; i++) {
  data_word = precision_read(3);

  data_array[i] = data_word;
  data_mean_raw = data_mean_raw + data_word;
  }

  mean = (float)data_mean_raw/64.;

  for(int i=0; i<64; i++) {
    data_point_f = (float)data_array[i] - mean;
    rms_accumulator = rms_accumulator + (data_point_f * data_point_f);
  }
  rms = rms_accumulator/64;
  rms = sqrt(rms)*7.629394e-2;
  mean = mean*7.629394e-2;

  Serial.print("Mean: ");
  Serial.print(mean);
  Serial.print("mV   ");
  Serial.print("rms: ");
  Serial.print(rms);
  Serial.print("mV\n\r");
  */
}
