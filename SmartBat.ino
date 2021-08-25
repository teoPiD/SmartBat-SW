#include <EEPROM.h>
#include <Wire.h>

#define Alarm   PIN_PC0
#define Buzzer  PIN_PB2
#define LED     PIN_PC1

#define Curr    PIN_PB0

#define DAC     PIN_PA6

#define VCell1  PIN_PA4
#define VCell2  PIN_PA5
#define VCell3  PIN_PA7
#define VCell4  PIN_PB5
#define VCell5  PIN_PB4
#define VCell6  PIN_PB1

#define VFront  PIN_PC2
#define VRear   PIN_PC3

#define SDA     PIN_PA1
#define SCL     PIN_PA2
#define SCK     PIN_PA3

#define Address 0x0B
#define totalVoltReg 0x09
#define currentReg 0x2A
#define cell1VolTReg 0x28
#define writeMinVoltReg 0x26
#define readMinVoltReg 0x27
#define MaxCellNum 6
#define ADCRes  12
#define lowestVoltage 100
#define updateFrequecy 100
#define loopCycle 1000/updateFrequecy

#define NOT_CUSTOM_INA false
#define CUSTOM_INA     true

#define BOARD CUSTOM_INA

#if BOARD == NOT_CUSTOM_INA
volatile uint16_t currMultFactor = 1.0;
#define CellNum 2
#else
volatile uint16_t currMultFactor = 1.0;
#define CellNum 4
#endif

enum alarmStates {
  OFF,
  ON_BAT,
  ON_ALARM_SIG,
  ON_BAT_AND_ALARM_SIG
};

enum I2CStates {
  READ,
  WAIT_LOWEST_BYTE,
  WAIT_HIGHER_BYTE
};

alarmStates alarmState = ON_BAT_AND_ALARM_SIG;
volatile I2CStates I2CState = READ;
volatile bool alarmFired = false;

volatile uint16_t totalVoltage;
volatile uint16_t cellVoltage[CellNum];
volatile uint16_t minVoltage;
volatile uint32_t curr;

// Voltage divider: Vout = Vin * Rn / (R1 + ... + Rn) <=> Vin = ((R1 + ... + Rn) / Rn) * Vout
// ADC in V: ADC Vin = Voltage Divider Vout = ADC value * reference / resolution = ADC value * 4.3V / 4096 - 12-bit resolution
// ADC in mV: ADC Vin * 1000 = ADC value * 4300 / 4096
// Vin = (ADC value * 4300 / 4096) * ((R1 + ... + Rn) / Rn)
const float cellVoltageMultFactor[MaxCellNum] = {
  
  92445.7/86016.0,  // 1 = (499 + 21000) / 21000 * (4300 / 4096)
  184900.0/86016.0, // 2 = (22000 + 21000) / 21000 * (4300 / 4096)
  277345.7/86016.0, // 3 = (499 + 43000 + 21000) / 21000 * (4300 / 4096)
  369800.0/86016.0, // 4 = (22000 + 43000 + 21000) / 21000 * (4300 / 4096)
  466550.0/86016.0, // 5 = (36500 + 51000 + 21000) / 21000 * (4300 / 4096)
  554700.0/86016.0  // 6 = (33000 + 75000 + 21000) / 21000 * (4300 / 4096)

};

unsigned long keepTime;
volatile uint8_t registerAddress;
volatile uint16_t tempMinVoltage;
volatile bool toUpdateEEPROM = false;

bool readCellVoltages();
void readCurrent(void);
bool setAlarmState(bool underMinVolt);
void updateAlarm(bool alarmUpdate);
void setAlarm(bool underMinVolt);
void receiveEvent(int howMany);
void requestEvent(void);
void updateEEPROM(void);

void setup() {

  pinMode(Alarm, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  // There is no memory gains when compared using direct pin assignment vs pinMode()
  //PORTC.DIRCLR = PIN0_bm; // Set PC0 - Alarm - as input
  //PORTB.DIRSET = PIN2_bm; // Set PB2 - Buzzer - as output
  //PORTC.DIRSET = PIN1_bm; // Set PC1 - LED - as output
  
  EEPROM.get(0, minVoltage);
  
  PORTC.PIN0CTRL = 0b00001001; //PULLUPEN = 1, ISC = 1 trigger both

  analogReference(INTERNAL4V3);
  
  Wire.swap(1);
  Wire.begin(Address);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  keepTime = millis();

}

void loop() {

  if(millis() - keepTime >= loopCycle){

    keepTime = millis();
    
    bool underMinVolt = readCellVoltages();

    readCurrent();

    setAlarm(underMinVolt);
    
  }
  
  if(toUpdateEEPROM){
    updateEEPROM();
  }
  
}

bool readCellVoltages(){
  
  uint8_t i, j;
  uint16_t measure = 0, voltageDifference = 0;
  bool underMinVolt = false;
  
  for (i = 0; i < CellNum; i++){
    switch(i){
      
      case 0:
        measure = analogReadEnh(VCell1, ADCRes, 0);
        break;
        
      case 1:
        measure = analogReadEnh(VCell2, ADCRes, 0);
        break;
        
      case 2:
        measure = analogReadEnh(VCell3, ADCRes, 0);
        break;
        
      case 3:
        measure = analogReadEnh(VCell4, ADCRes, 0);
        break;
        
      case 4:
        measure = analogReadEnh(VCell5, ADCRes, 0);
        break;
        
      case 5:
        measure = analogReadEnh(VCell6, ADCRes, 0);
        break;
        
      default:
        break;
    }

    // Cell voltage equals the measured voltage minus the voltage of the previous cells
    // For example, voltage of the cell number 5 is equal to the voltage difference between 
    // cell 5 and ground minus the voltage difference between cell 4 and ground
    cellVoltage[i] = ((uint16_t)(((float)measure * cellVoltageMultFactor[i])) - voltageDifference);

    // Voltage difference between current cell and ground
    voltageDifference += cellVoltage[i];

    // If the cell voltage is below minimum cell voltage and battery is connected - assumed 
    // that when a battery is connected the cell voltage is never lower than 100mV (to 
    // avoid floating port voltage level)
    if ((cellVoltage[i] < minVoltage) && (cellVoltage[i] >= lowestVoltage)){
      underMinVolt = true;
    }
    
  }

  totalVoltage = voltageDifference;

  return underMinVolt;
  
}

void readCurrent(void){

  curr = (uint32_t)((float) analogReadEnh(Curr, ADCRes, 0) * currMultFactor);
  return;
  
}

bool setAlarmState(bool underMinVolt){
  
  bool alarmUpdate = false;

  if(underMinVolt){
    if(alarmFired){
      // Don't need to waste clock cycles turning on buzzer and LED if they were already on
      alarmUpdate = !(alarmState == ON_BAT_AND_ALARM_SIG);
      alarmState = ON_BAT_AND_ALARM_SIG;
    }else{
      // Don't need to waste clock cycles turning on buzzer and LED if they were already on
      alarmUpdate = !(alarmState == ON_BAT);
      alarmState = ON_BAT;
    }
  }else{
     if(alarmFired){
      // Don't need to waste clock cycles turning on buzzer and LED if they were already on
      alarmUpdate = !(alarmState == ON_ALARM_SIG);
      alarmState = ON_ALARM_SIG;
    }else{
      // Don't need to waste clock cycles turning off buzzer and LED if they were already on
      alarmUpdate = !(alarmState == OFF);
      alarmState = OFF;
    }
  }

  return alarmUpdate;
  
}

void updateAlarm(bool alarmUpdate){
  
  if(alarmUpdate){
    if (alarmState == OFF){
      //digitalWrite(Buzzer, LOW);
      //digitalWrite(LED, LOW);
      PORTB.OUTCLR = PIN2_bm; // turn PB2 - Buzzer - output off
      PORTC.OUTCLR = PIN1_bm; // turn PC1 - LED - output off
    }else{
      //digitalWrite(Buzzer, HIGH);
      //digitalWrite(LED, HIGH);
      PORTB.OUTSET = PIN2_bm; // turn PB2 - Buzzer - output on
      PORTC.OUTSET = PIN1_bm; // turn PC1 - LED - output on
    }
  }
  
  return;
  
}

void setAlarm(bool underMinVolt){
  bool alarmUpdate = true;
  
  alarmUpdate = setAlarmState(underMinVolt);
  updateAlarm(alarmUpdate);

  return;
  
}

void receiveEvent(int howMany){
  
  while (Wire.available()){
    byte data = Wire.read();
    registerAddress = data;
    switch (I2CState) {
      
      case READ:
        if (data == writeMinVoltReg){
          I2CState = WAIT_LOWEST_BYTE;
        }
        break;
        
      case WAIT_LOWEST_BYTE:
        tempMinVoltage = data;
        I2CState = WAIT_HIGHER_BYTE;
        break;

      case WAIT_HIGHER_BYTE:
        tempMinVoltage |= data << 8;
        toUpdateEEPROM = true;
        I2CState = READ;
        break;

      default:
        break;
        
    }
  }

  return;
  
}

void requestEvent(void){
  
  switch (registerAddress) {

    case totalVoltReg: // Total battery voltage request
      Wire.write(totalVoltage & 0xFF);
      Wire.write((totalVoltage >> 8) & 0xFF);
      break;
    
    case currentReg: // Current request
      Wire.write(curr & 0xFF);
      Wire.write((curr >> 8) & 0xFF);
      Wire.write((curr >> 16) & 0xFF);
      Wire.write((curr >> 24) & 0xFF);
      break;
      
    case cell1VolTReg: // Individual cell voltage request
      for(uint8_t i = 0; i < CellNum; i++){
        Wire.write(cellVoltage[i] & 0xFF);
        Wire.write((cellVoltage[i] >> 8) & 0xFF);
      }
      break;

    case readMinVoltReg: // Alarm turn on voltage request
      Wire.write(minVoltage && 0xFF);
      Wire.write((minVoltage >> 8) && 0xFF);
      break;
      
    default:
      break;
      
  }
}

void updateEEPROM(void){
  
  EEPROM.put(0, tempMinVoltage);
  minVoltage = tempMinVoltage;
  toUpdateEEPROM = false;

  return;
}

ISR(PORTC_PORT_vect) {
  
  PORTC.INTFLAGS = 1; //we know only PC0 has an interrupt, so that's the only flag that could be set.
  alarmFired = !alarmFired;
  
  return;
}
