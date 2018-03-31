/////////////////////
//    Libraries    //
/////////////////////

#include <Time.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <SPI.h>
#include "mcp_can.h"

/////////////////////
//  Configuration  //
/////////////////////

const int CS_PIN_CAN0 = 10;
const int CS_PIN_CAN1 = 9;
const long SERIAL_SPEED = 115200;
const byte CAN_SPEED = CAN_125KBPS; // Entertainment CAN bus - Low speed

////////////////////
// Initialization //
////////////////////

MCP_CAN CAN0(CS_PIN_CAN0); // CAN-BUS Shield N°1
MCP_CAN CAN1(CS_PIN_CAN1); // CAN-BUS Shield N°2

////////////////////
//    Variables   //
////////////////////

// My variables
bool debugCAN0 = false; // Read data sent by ECUs from the car to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
bool debugCAN1 = false; // Read data sent by the NAC / SMEG to Entertainment CAN bus using https://github.com/alexandreblin/python-can-monitor
bool EconomyModeEnabled = true; // You can disable economy mode if you want to - Not recommended at all (Battery was dead in less than a day)
bool TemperatureInF = false; // Default temperature in Celcius
bool mpgMi = false;
int languageNum = 128; // (0x80) FR - If you need EN as default : 132 (0x84)
int languageID = 0; // FR - If you need EN as default : 1
int languageID_HeadupPanel = 0; // FR - If you need EN as default : 1
int TelematicTime_day = 1; // Default day if the RTC module is not configured
int TelematicTime_month = 1; // Default month if the RTC module is not configured
int TelematicTime_year = 2018; // Default year if the RTC module is not configured
int TelematicTime_hour = 0; // Default hour if the RTC module is not configured
int TelematicTime_minute = 0; // Default minute if the RTC module is not configured

// Default variables
int temperature = 0;
bool EconomyMode = false;
bool EngineRunning = false;
bool AirConditioningON = false;
int FanSpeed = 0;
bool FanOff = false;
bool AirRecycle = false;
bool DeMist = false;
bool DeFrost = false;
int LeftTemp = 0;
int RightTemp = 0;
bool Mono = false;
bool FootAerator = false;
bool WindShieldAerator = false;
bool CentralAerator = false;
bool AutoFan = false;
int FanPosition = 0;

// CAN-BUS Messages
unsigned char len = 0;
byte buffer[8];
unsigned char canMsg[8];

void setup() {
    // Initalize Serial for debug
    Serial.begin(SERIAL_SPEED);

    byte canSpeed = CAN_SPEED;

    // CAN-BUS from car
    Serial.println("Initialization CAN0 ...");

    while (CAN0.begin(canSpeed) != CAN_OK) {
        delay(100);
    }

    Serial.println("Initialization OK CAN0");

    // CAN-BUS to Telematic
    Serial.println("Initialization CAN1 ...");

    while (CAN1.begin(canSpeed) != CAN_OK) {
        delay(100);
    }

    Serial.println("Initialization OK CAN1");

    setSyncProvider(RTC.get); // Get time from the RTC module
    if (timeStatus() != timeSet) {
        Serial.println("Unable to sync with the RTC");

        // Set default time (01/01/2018 00:00)
        setTime(TelematicTime_hour, TelematicTime_minute, 0, TelematicTime_day, TelematicTime_month, TelematicTime_year);
    } else {
        Serial.println("RTC has set the system time");
    }

    // Set hour on RCZ CAN-BUS Clock
    canMsg[0] = hour();
    canMsg[1] = minute();
    CAN0.sendMsgBuf(0x228, 0, 2, canMsg);

    Serial.print("Current Time: ");
    Serial.print(day());
    Serial.print("/");
    Serial.print(month());
    Serial.print("/");
    Serial.print(year());

    Serial.print(" ");

    Serial.print(hour());
    Serial.print(":");
    Serial.print(minute());

    Serial.println();
}

void loop() {
    int tmpVal;

    // Receive CAN messages from the car
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
        CAN0.readMsgBuf( & len, buffer);
        int id = CAN0.getCanId();

        if (debugCAN0) {
            Serial.print("FRAME:ID=");
            Serial.print(id);
            Serial.print(":LEN=");
            Serial.print(len);

            char tmp[3];
            for (int i = 0; i < len; i++) {
                Serial.print(":");

                snprintf(tmp, 3, "%02X", buffer[i]);

                Serial.print(tmp);
            }

            Serial.println();

            CAN1.sendMsgBuf(id, 0, len, buffer);
        } else if (!debugCAN1) {
            if (id != 608 && id != 566 && id != 464 && id != 54) {
                CAN1.sendMsgBuf(id, 0, len, buffer);
            }

            if (id == 54 && len == 8) { // Economy Mode detection, there are probably other ways to detect it but this is a match for both RCZ & 308 (T9)
                tmpVal = (buffer[2] & 0xFF);
                if (tmpVal >= 128) {
                    if (!EconomyMode) {
                        Serial.println("Economy mode ON");
                    }

                    EconomyMode = true;
                } else {
                    if (EconomyMode) {
                        Serial.println("Economy mode OFF");
                    }

                    EconomyMode = false;
                }

                tmpVal = (buffer[3] & 0xFF);
                // Fix brightness when car lights are ON - Brightness headup panel "20" > "2F" (32 > 47)
                if (tmpVal >= 32) {
                    buffer[3] = 0x28; // Set fixed value so both values on the Telematic are independant
                }

                CAN1.sendMsgBuf(0x036, 0, 8, buffer);
            } else if (id == 182 && len == 8) {
                if (buffer[0] > 0x00 || buffer[1] > 0x00) { // Seems to be engine RPM, 0x00 0x00 when the engine is OFF
                    EngineRunning = true;
                } else {
                    EngineRunning = false;
                }
            } else if (id == 464 && len == 7 && EngineRunning) { // No fan activated if the engine is not ON on old models
                LeftTemp = (buffer[5] & 0xFF);
                RightTemp = (buffer[6] & 0xFF);
                if (LeftTemp == RightTemp) { // No other way to detect MONO mode
                    Mono = true;
                    LeftTemp = LeftTemp + 64;
                } else {
                    Mono = false;
                }

                FanOff = false;
                // Fan Speed BSI_2010 = "41" (Off) > "49" (Full speed)
                tmpVal = (buffer[2] & 0xFF);
                if (tmpVal == 15) {
                    FanOff = true;
                    FanSpeed = 0x41;
                } else {
                    FanSpeed = (tmpVal + 66);
                }

                // Position Fan
                tmpVal = (buffer[3] & 0xFF);

                if (tmpVal == 0x40) {
                    FootAerator = false;
                    WindShieldAerator = true;
                    CentralAerator = false;
                } else if (tmpVal == 0x30) {
                    FootAerator = false;
                    WindShieldAerator = false;
                    CentralAerator = true;
                } else if (tmpVal == 0x20) {
                    FootAerator = true;
                    WindShieldAerator = false;
                    CentralAerator = false;
                } else if (tmpVal == 0x70) {
                    FootAerator = false;
                    WindShieldAerator = true;
                    CentralAerator = true;
                } else if (tmpVal == 0x80) {
                    FootAerator = true;
                    WindShieldAerator = true;
                    CentralAerator = true;
                } else if (tmpVal == 0x50) {
                    FootAerator = true;
                    WindShieldAerator = false;
                    CentralAerator = true;
                } else if (tmpVal == 0x10) {
                    FootAerator = false;
                    WindShieldAerator = false;
                    CentralAerator = false;
                } else if (tmpVal == 0x60) {
                    FootAerator = true;
                    WindShieldAerator = true;
                    CentralAerator = false;
                } else {
                    FootAerator = false;
                    WindShieldAerator = false;
                    CentralAerator = false;
                }

                tmpVal = (buffer[4] & 0xFF);
                if (tmpVal == 0x10) {
                    DeMist = true;
                    AirRecycle = false;
                } else if (tmpVal == 0x30) {
                    AirRecycle = true;
                } else {
                    AirRecycle = false;
                }

                AutoFan = false;
                DeMist = false;

                tmpVal = (buffer[0] & 0xFF);
                if (tmpVal == 0x11) {
                    DeMist = true;
                    AirConditioningON = true;
                    FanOff = false;
                } else if (tmpVal == 0x12) {
                    DeMist = true;
                    AirConditioningON = false;
                    FanOff = false;
                } else if (tmpVal == 0x21) {
                    DeMist = true;
                    AirConditioningON = true;
                    FanOff = false;
                } else if (tmpVal == 0xA2) {
                    FanOff = true;
                    AirConditioningON = false;
                } else if (tmpVal == 0x22) {
                    AirConditioningON = false;
                } else if (tmpVal == 0x20) {
                    AirConditioningON = true;
                } else if (tmpVal == 0x02) {
                    AirConditioningON = false;
                    AutoFan = false;
                } else if (tmpVal == 0x00) {
                    AirConditioningON = true;
                    AutoFan = true;
                }

                if (!FootAerator && !WindShieldAerator && CentralAerator) {
                    FanPosition = 0x34;
                } else if (FootAerator && WindShieldAerator && CentralAerator) {
                    FanPosition = 0x84;
                } else if (!FootAerator && WindShieldAerator && CentralAerator) {
                    FanPosition = 0x74;
                } else if (FootAerator && !WindShieldAerator && CentralAerator) {
                    FanPosition = 0x54;
                } else if (FootAerator && !WindShieldAerator && !CentralAerator) {
                    FanPosition = 0x24;
                } else if (!FootAerator && WindShieldAerator && !CentralAerator) {
                    FanPosition = 0x44;
                } else if (FootAerator && WindShieldAerator && !CentralAerator) {
                    FanPosition = 0x64;
                } else {
                    FanPosition = 0x04; // Nothing
                }

                if (DeMist) {
                    FanSpeed = 0x10;
                    FanPosition = FanPosition + 16;
                } else if (AutoFan) {
                    FanSpeed = 0x10;
                }

                if (FanOff) {
                    AirConditioningON = false;
                    FanSpeed = 0x41;
                    LeftTemp = 0x00;
                    RightTemp = 0x00;
                    FanPosition = 0x04;
                }

                if (AirConditioningON) {
                    canMsg[0] = 0x01; // A/C ON - Auto Soft : "00" / Auto Normal "01" / Auto Fast "02"
                } else {
                    canMsg[0] = 0x09; // A/C OFF - Auto Soft : "08" / Auto Normal "09" / Auto Fast "0A"
                }
				
                canMsg[1] = 0x00;
                canMsg[2] = 0x00;
                canMsg[3] = LeftTemp;
                canMsg[4] = RightTemp;
                canMsg[5] = FanSpeed;
                canMsg[6] = FanPosition;
                canMsg[7] = 0x00;
                CAN1.sendMsgBuf(0x350, 0, 8, canMsg);
            } else if (id == 246 && len == 8) {
                // If time is synced
                if (timeStatus() != timeNotSet) {
                    canMsg[0] = (year() - 1872); // Year would not fit inside one byte (0 > 255), substract 1872 and you get this new range (1872 > 2127)
                    canMsg[1] = month();
                    canMsg[2] = day();
                    canMsg[3] = hour();
                    canMsg[4] = minute();
                    canMsg[5] = 0x3F;
                    canMsg[6] = 0xFE;
                    CAN1.sendMsgBuf(0x276, 0, 7, canMsg);
                } else {
                    canMsg[0] = (TelematicTime_year - 1872); // Year would not fit inside one byte (0 > 255), substract 1872 and you get this new range (1872 > 2127)
                    canMsg[1] = TelematicTime_month;
                    canMsg[2] = TelematicTime_day;
                    canMsg[3] = TelematicTime_hour;
                    canMsg[4] = TelematicTime_minute;
                    canMsg[5] = 0x3F;
                    canMsg[6] = 0xFE;
                    CAN1.sendMsgBuf(0x276, 0, 7, canMsg);
                }

                if (!EngineRunning) {
                    AirConditioningON = false;
                    FanSpeed = 0x41;
                    LeftTemp = 0x00;
                    RightTemp = 0x00;
                    FanPosition = 0x04;

                    canMsg[0] = 0x09;
                    canMsg[1] = 0x00;
                    canMsg[2] = 0x00;
                    canMsg[3] = LeftTemp;
                    canMsg[4] = RightTemp;
                    canMsg[5] = FanSpeed;
                    canMsg[6] = FanPosition;
                    canMsg[7] = 0x00;
                    CAN1.sendMsgBuf(0x350, 0, 8, canMsg);
                }

                // Lang / Temp / Aid options
                canMsg[0] = languageNum;

                if (TemperatureInF) {
                    canMsg[1] = 0x5C;
                } else {
                    canMsg[1] = 0x1C;
                }
                if (mpgMi) {
                    canMsg[1] = canMsg[1] + 128;
                }

				// I still have some investigation work to do here, it is the settings of interior ambience, and so on
				// **************************
                canMsg[2] = 0x97;
                canMsg[3] = 0x9B;
                canMsg[4] = 0xE0;
                canMsg[5] = 0xD0;
                canMsg[6] = 0x00;
				// **************************
				
                CAN1.sendMsgBuf(0x260, 0, 7, canMsg);

                // Economy mode simulation
                if (EconomyMode && EconomyModeEnabled) {
                    canMsg[0] = 0x14; // 0x14
                } else {
                    if (EngineRunning) {
                        canMsg[0] = 0x54;
                    } else {
                        canMsg[0] = 0x04;
                    }
                }
                CAN1.sendMsgBuf(0x236, 0, 1, canMsg);

                tmpVal = ceil((buffer[5] & 0xFF) / 2.0) - 40; // Temperatures can be negative but we only have 0 > 255, the new range is starting from -40°C
                if (temperature != tmpVal) {
                    temperature = tmpVal;

                    Serial.print("Ext. Temperature: ");
                    Serial.print(tmpVal);
                    Serial.println("°C");
                }
            } else if (id == 727 && len == 5) { // Headup Panel
                tmpVal = (buffer[0] & 0xFF);

                if (languageID_HeadupPanel != tmpVal) {
                    languageID_HeadupPanel = tmpVal;

                    // Change language on ID 608 for Telematic language change
                    languageNum = (languageID_HeadupPanel * 4) + 128;

                    Serial.print("Headup Panel - Change Language: ");
                    Serial.print(tmpVal);
                    Serial.println();
                }
            } else if (id == 608 && len == 7) {
                tmpVal = (buffer[0] & 0xFF);

                if (languageNum != tmpVal) {
                    languageNum = tmpVal;

                    Serial.print("BSI - Current Language: ");
                    Serial.print(tmpVal);
                    Serial.println();

                }

                tmpVal = (buffer[1] & 0xFF);
                if (tmpVal == 92 && !TemperatureInF) {
                    TemperatureInF = true;

                    Serial.print("BSI - Current Temperature Type: Fahrenheit");
                    Serial.println();
                } else if (tmpVal == 28 && TemperatureInF) {
                    TemperatureInF = false;

                    Serial.print("BSI - Current Temperature Type: Celcius");
                    Serial.println();
                }
            }
        } else {
            CAN1.sendMsgBuf(id, 0, len, buffer);
        }
    }

    if (CAN1.checkReceive() == CAN_MSGAVAIL) {
        CAN1.readMsgBuf( & len, buffer);
        int id = CAN1.getCanId();

        if (debugCAN1) {
            Serial.print("FRAME:ID=");
            Serial.print(id);
            Serial.print(":LEN=");
            Serial.print(len);

            char tmp[3];
            for (int i = 0; i < len; i++) {
                Serial.print(":");

                snprintf(tmp, 3, "%02X", buffer[i]);

                Serial.print(tmp);
            }

            Serial.println();

            CAN0.sendMsgBuf(id, 0, len, buffer);
        } else if (!debugCAN0) {
            if (id == 923 && len == 5) {
                TelematicTime_year = (buffer[0] & 0xFF) + 1872; // Year would not fit inside one byte (0 > 255), add 1872 and you get this new range (1872 > 2127)
                TelematicTime_month = (buffer[1] & 0xFF);
                TelematicTime_day = (buffer[2] & 0xFF);
                TelematicTime_hour = (buffer[3] & 0xFF);
                TelematicTime_minute = (buffer[4] & 0xFF);

                setTime(TelematicTime_hour, TelematicTime_minute, 0, TelematicTime_day, TelematicTime_month, TelematicTime_year);
                RTC.set(now()); // Set the time on the RTC module too

                // Set hour on RCZ CAN-BUS Clock
                canMsg[0] = hour();
                canMsg[1] = minute();
                CAN0.sendMsgBuf(0x228, 0, 2, canMsg);

                Serial.print("Change Hour/Date: ");
                Serial.print(day());
                Serial.print("/");
                Serial.print(month());
                Serial.print("/");
                Serial.print(year());

                Serial.print(" ");

                Serial.print(hour());
                Serial.print(":");
                Serial.print(minute());

                Serial.println();
            } else if (id == 347 && len == 8) {
                tmpVal = (buffer[0] & 0xFF);
                if (tmpVal >= 128) {
                    languageNum = tmpVal;

                    Serial.print("Telematic - Change Language (Number): ");
                    Serial.print(tmpVal);
                    Serial.println();

                    tmpVal = (buffer[1] & 0xFF);
                    if (tmpVal >= 128) {
                        mpgMi = true;
                        tmpVal = tmpVal - 128;
                    } else {
                        mpgMi = false;
                    }

                    if (tmpVal >= 64) {
                        TemperatureInF = true;

                        Serial.print("Telematic - Change Temperature Type: Fahrenheit");
                        Serial.println();
                    } else if (tmpVal >= 0) {
                        TemperatureInF = false;

                        Serial.print("Telematic - Change Temperature Type: Celcius");
                        Serial.println();
                    }
                } else {
                    tmpVal = ceil(tmpVal / 4.0);
                    languageID = tmpVal;

                    // Head-up panel is only one-way talking, we can't change the language on it from the Telematic :-(

                    Serial.print("Telematic - Change Language (ID): ");
                    Serial.print(tmpVal);
                    Serial.println();
                }
            } else if (id == 485 && len == 7) {
                // Ambience mapping
                tmpVal = (buffer[5] & 0xFF);
                if (tmpVal == 0x00) { // User
                    buffer[6] = 0x40;
                } else if (tmpVal == 0x08) { // Classical
                    buffer[6] = 0x44;
                } else if (tmpVal == 0x10) { // Jazz
                    buffer[6] = 0x48;
                } else if (tmpVal == 0x18) { // Pop-Rock
                    buffer[6] = 0x4C;
                } else if (tmpVal == 0x28) { // Techno
                    buffer[6] = 0x54;
                } else if (tmpVal == 0x20) { // Vocal
                    buffer[6] = 0x50;
                } else { // Default : User
                    buffer[6] = 0x40;
                }

                // Loudness / Volume linked to speed
                tmpVal = (buffer[4] & 0xFF);
                if (tmpVal == 0x10) { // Loudness / not linked to speed
                    buffer[5] = 0x40;
                } else if (tmpVal == 0x14) { // Loudness / Volume linked to speed
                    buffer[5] = 0x47;
                } else if (tmpVal == 0x04) { // No Loudness / Volume linked to speed
                    buffer[5] = 0x07;
                } else if (tmpVal == 0x00) { // No Loudness / not linked to speed
                    buffer[5] = 0x00;
                } else { // Default : No Loudness / not linked to speed
                    buffer[5] = 0x00;
                }

                // Bass
                // NG4 Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
                // Telematic Range: "32" > "88" ("60" = 0)
                tmpVal = (buffer[2] & 0xFF);
                buffer[2] = ((tmpVal - 32) / 4) + 57; // Converted value

                // Treble
                // NG4 Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
                // Telematic Range: "32" > "88" ("60" = 0)
                tmpVal = (buffer[3] & 0xFF);
                buffer[4] = ((tmpVal - 32) / 4) + 57; // Converted value on position 4 (while it's on 3 on a Telematic)

                // Balance - Left / Right
                // NG4 Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
                // Telematic Range: "32" > "88" ("60" = 0)
                tmpVal = (buffer[1] & 0xFF);
                buffer[1] = ((tmpVal - 32) / 4) + 57; // Converted value

                // Balance - Front / Back
                // NG4 Range: (-9) "54" > (-7) "57" > ... > "72" (+9) ("63" = 0)
                // Telematic Range: "32" > "88" ("60" = 0)
                tmpVal = (buffer[0] & 0xFF);
                buffer[0] = ((tmpVal - 32) / 4) + 57; // Converted value

                buffer[3] = 63; // ?? 0x3F = 63 (seems 0 default setting of something)

                CAN0.sendMsgBuf(id, 0, len, buffer);
            } else {
                CAN0.sendMsgBuf(id, 0, len, buffer);
            }
        } else {
            CAN0.sendMsgBuf(id, 0, len, buffer);
        }

    }
}