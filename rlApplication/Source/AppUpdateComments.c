//718
/*1) Display related issue resolved*/
//717
/*1) Ver No. has been changed to 468.717 on both display and memory
2) Communication Mode has been changed from IrDA to Optical.
3) LCD type changed to A type LCD.
4) In Production Mode, Pulse LED will glow when we send command (EEOMEMT).*/ 

//852
//1. NM LOCK COUNT INCREADED FROM 5 TO 180
//2. DEFAULT CALIBRATION CHANGED .
//3. NM LOCK WILL RESET WHEN PUSH BUTTON IS PRESSED.
//4. NM EXIT TIME DUE TO FALSE CURRENT CHANGED TO 3 MINUTES FROM 50 MINUTES.
//853
//5. REMOVED VER# FROM DISPLAY
//6. NOW IT CAN BE READ FROM COMMUNICATION. COMMAND "EEOVER0D0A";


//862
//1. earth is blinking. which is not correct.
//2. dc immunity range new set.

//866
//1. change in nm logic in normal mode.

//868
//1. Change in production display parameters.
//2. removed blinking in production mode.
//3. now command added to change offset values for current.
//4. show rev icon in production mode.
 
//875
//1. dynamic offset added when current flows through CT.
//2. change nm case if voltage is less than 120V
//0.03.14
//1. erase blocked in user mode.
//0.03.15
//1. first billing happening in NM if there is not LS data logged in normal mode.
//0.03.16
//1. power on duration is coorected in case meter RUN in NM.

//0.03.17
//1. billing occurs while erased in NM case.
//0.03.17
//inavlid

//0.03.019
//1. ND detection logic changed in case if it is applied REV.
//2. i2c reset clock increased for i2c hang problem.

//1.04.017
// 1. dc magnet immune logic changed.
//1.04.023
//1. meter will not run below 100V
//2. meter will comes only from power off mode if volatge is greater than 110V or if less than 110V and load is earthed.
//1.04.024
//1. meter takes time from battery mode to normal mode if supply is applied on portable source, if battery mode time is more than 5 mins.
//2. VRTC pin will get supply from MCU in AC supply condition.
//3. Now all seg will show when memtest command will be received in production mode.
//4. first stage powerfail data will be saved when voltage is less than 120V.
//4.65.000
//prekWh initial value changed.
//1. some times crc of data saved at power fail event mismatched. this is beacuse 12 bit timer counter modify pulse counter values after calculatiing crc.
//2. if there is no more than .1 kwh increament in one run, and kwh increased more than 3 kwh in such repeatition then kwh will become 0. now it is corrected.
//3. power fail data saved 1 level voltage chnaged to 120V. 

//900
//1. magnet logic changed.
//2. power assigned before tamper checking.

//904
//.1. NM accuracy flactuating.

//606
//1. R symbol will blink for cover open
//2. cover open will show continously
//3. cover open tamper will appear in push button with date and time.
//4. tod timing for seasons interchanged. (Reported by Pankaj )
//607,508
//1. no load scan time reduced to 60 secs from 3600 secs.
//609,510
//1. lcd on in battery mode after power fail.
//611,512
//1. avg pf of last month added in power fail mode
//617,516
//1. Avg V,Avg Current logic changed.
//2. Pf will be unity while there is no load
//3. Apperent power will be zero if it is below 1.6VA
//4. active power is greater than Apperent power check place after all calculation.
//5. tod zone time should be macth integration period , if not match then not set.
//621,520
//1. cover tamprer count is removed from total tamper count.
/*623,522
1. 2 seconds power added into NM case in battery mode while first time current detected to adjust NM case dial test.
2. Earth, Rev and ND will not restore below 50mA.
30/09/2022
3. production mode rtc batetry status modiled, earlier it show main battery status.
4. bill day obis code modified, now need to set through single action schedule.
4.1 bill date=="FF/FF/FF" then md reset
4.2 bill date=="xx/FF/FF" then bill day set.
4.3 otherwise single action scheduled.
5. Check Selft Diagonstic feature
13/10/2022
1. form factor threshold changed from 2 to 1.78, if voltage is less than 120V.
2. In battry mode voltage threshold chnaged from 50V to 35 V. it can effect NM case , need to verify it.
12/11/2022
1. commented E2PR acces in dlms.
2. also changed typedef for tag_attr_access_method_t in r_dlms_obis_ic.h file.
3. lcd mapping chnaged for Type A lcd.
15/11/2022
1. TOD kWh & kVAh register value not incrementing as per main kWh. Current TOD enrergy added for export energy.
15/12/2022
1. watt parameters will be show 5+1 instead of 4+2.
22/12/2022-711

1. pulse problem after current off during nm or nd restoration
14/06/2023-715
1. rtc adjustment added at every 5 mins
16/06/2023 716
1. uart1 is disabled
*/