//MASTER CONTROLLER!!
//Controls solar panel heat collection system
//Judas Gutenberg
//SOLAR CONTROLLER II:
//September 2006: core code
//August 9, 2008: added EEPROM storage of hotwatermax value, which can be modified on the fly serially
//August 10, 2008: stores minutecounter, a kind of poor man's RTC, across restarts. 
//Now logs things, and is highly interactive. Most operative values can be changed on the fly serially.
//SOLAR CONTROLLER III:
//February 20, 2011: added a real RTC, a slave arduino for more analog inputs, an LCD display, and a bunch of EEPROM
//March 7, 2011: added a second I2C bus connectable by quad bilateral switch (controlled by Arduino 0's Pin 7) 
//for use in a long cable out to the oil tank for fuel level readings
//June 23, 2011:  added a menu system developed since late April. 
//menu system happens entirely on the Slave, and then sends commands to Master in the same format as commands to 
//Master sent via the serial terminal (previously, the only way to reconfigure the controller).
//to achieve sufficient granularity without resorting to floating point math
//time values are normally manipulated in units of deciminutes (each being a tenth of a minute, or six seconds long)
//and temperatures are manipulated in units of decidegrees Fahrenheit 
//(2120 being the boiling point of water and 320 being the freezing point)
//June 27, 2013: added a serial watchdog allowing me to keep the capacitor connecting DTR to reset disconnected and 
//only connect it temporarily with a command of the form !X[digit] -- see serialwatchdog source for details -- it 
//resides in an AtTiny85.

#include <Wire.h>
#include <EEPROM.h>
#include <avr/wdt.h> /* Header for watchdog timers in AVR */


//didn't use to have to do this:
#define WDTO_8S 9

//#define EE_1 (0xA0>>1)
//#define EE_2 (0xA8>>1)
//i2c device addresses:
#define SLAVE_ADDRESS 19
#define SLAVE_RANGEFINDER 51
#define DS1307_ADDRESS 0x68
#define RANGER_ADDRESS 112
#define EEPROM_LOW_MEM_ADDRESS 84  //low eeprom is where i store the boiler log
#define EEPROM_HIGH_MEM_ADDRESS 85	//high eeprom is where i store the event log
#define BAUD_RATE 19200 

#define cmdByte 0x00                              // Command byte
#define rangeByte 0x02                            // Byte for start of ranging data
#define maxShortLog 80
#define FUELLEVELTESTACTIVITY 2
#define NOSPECIFICACTIVITY 0

unsigned int maxbigcursor=65527;
byte rcv_bytes; 
int state;
int state2;
int valer;
bool invalidFuelLevelRead=true;
//the following placements are in this order: seconds,minutes, hours, dayofweek, day, month, year
byte  clocksettingregime[4][7] = 
{  
	{0,1,2,3,4,5,6},
	{8,8,8,0,8,8,8},
	{2,1,0,8,8,8,8},
	{8,8,8,8,2,1,0},
}
;

//stringliterals

char str_not[] =" not";
char str_frozen[] =" frozen";
char str_summerdef[] ="summer";
char str_winterdef[] ="winter";
char str_milli[] ="milli";
char str_solar[] ="solar";
char str_freemem[] ="free memory: ";
char str_long[] ="long";
char str_fired[] ="fired";
char str_for[] =" for ";
char str_word[] ="word";
char str_loop[] ="loop";
char str_forced[] ="forced";
char str_of[] ="of";
char str_byte[] ="byte";
char str_counter[] ="counter";
char str_isfrozen[]="is frozen? (1 if so):";
char str_slab[] ="slab";
char str_togo75[] ="to go 75 ft: "; 
char str_switch[] ="switch";
char str_pump[] ="pump";
char str_change[] ="change";
char str_cleared[] ="cleared";
char str_waitstart[] ="waitstart";
char str_began[] ="began";
char str_pumping[] ="pumping";
char str_event[] ="event";	
char str_cursor[] ="cursor";	
char str_level[] ="level";	
char str_tank[] ="tank";
char str_extreme[] ="extreme";	
char str_forcirculation[] ="for circulation";
char str_data[] ="data";
char str_panel[]="panel";
char str_afterload[] ="afterload";
//char str_ambient[] ="ambient";
char str_hwater[] ="hotwater";
char str_outdoor[] ="outdoor";
char str_basement[] ="basement";
char str_fuel[] ="fuel";
char str_checked[] ="checked";
char str_shutoff[] ="shutoff";
char str_reboot[] ="reboot";
char str_ing[] ="ing";
char str_in[] ="in";
char str_info[] ="info";
char str_old[] ="old";
char str_type[] ="type";
char str_location[] ="location";
char str_log[] ="log";
char str_reset[] ="reset";
char str_start[] ="start";
char str_setto[] ="set to";
char str_degreesf[]= " degrees F";
char str_morninglogs[] =" morning logs ";
char str_season[] ="season";	
char str_delay[] ="delay ";		
char str_flowrate[] =" flowrate:";	
char str_time[] ="time";	
char str_temperature[] ="temperature";
char str_max[] ="max";	
char str_min[] ="min";
char str_boiler[] ="boiler";
char str_second[] ="second";
char str_minute[] ="minute";
char str_count[] ="count";
char str_suff[] ="sufficiency: ";
char str_spaceatspace[] = " at ";
char str_spaceparen[]=" (";
char str_colonspace[] =": ";
byte str_space = ' ';
byte str_slash = '/';
byte str_colon = ':';
byte str_at = '@';
byte str_s = 's';
byte str_tab=9;//that's the way you get a control character into a byte

//mess of globals:
boolean wearelogging=false;
boolean forceInsufficiency=false;
boolean forceSolarSufficiency=false;
boolean forceSlabSufficiency=false;
byte ispumprunning=0;
byte tentativeHotwaterSufficiency=0;
byte tentativeSlabSufficiency=0;
byte completeHotwaterSufficiency=0;
byte completeSlabSufficiency=0;
byte isfrozen=0;
byte oldhws=0;
byte oldss=0;
byte season=1;
byte boilerstate=0;
byte basementtempmax=0;
unsigned long timeofbasementtempmax=0;
byte serval=48;
int oldserval=48;
int paneltemp=0;
int basementtemp=0;
int hotwatertemp=0;
int afterloadtemp=0;
int garageTemp=0;
int outsidetemp=0;
int slabtemp=0;
int afterloadtempraw=0;
int garageTempraw=0;
int slabtempraw=0;

byte illumination=0;  //for the LED
byte solarlogcursor;
unsigned long timebeganpumping=0;
unsigned long timesinceboot=0;
unsigned long timeofsufficiency=0;
unsigned long waitstart=0;
unsigned long minutecounter=0;
//unsigned long lastmins=0;
//unsigned long thesemins=0;

//int loopcount=0;

//60 minutes:
int frozenwaittime=600;// (this is in units of six seconds, aka deciminutes, tenths of a minute);

//normally 15 minutes 
int minstillfrozen=150;// (this is in units of six seconds, aka deciminutes, tenths of a minute));
byte thisPhase=4; //stores the number of the Atmega pin lighting the LED that goes with the temperature on the meter
int basementtempatpumpstart=0; 
int paneltempatshutoff=0;
int basementtempatshutoff=0;
int afterloadtempatshutoff=0;
int slabtempatshutoff=0;
int basementdelta=0;
byte flowrate=0;
int timeforflowtogo75feet=0;
unsigned long timeatswitchchange=0;
byte maxtimetoswitchchange=10; //wait at least a minute for no state changes before actually changing a state, a default
byte hotwatermaxbyte=22; //location in the RTC RAM for the max temp allowed for hot water (in degrees)
byte loopdelaybyte=23; //location in the RTC RAM for the cycle delay (a deciseconds value)
byte mintemptocirculatesummerbyte=24; //location in the RTC RAM for the mininum temp to circulate in summer(in degrees)
byte seasonbyte=25; //location in the RTC RAM of the information about what season we're in
byte mintemptocirculatewinterbyte=28; //location in the RTC RAM for the mininum temp to circulate in winter(in degrees)
byte solarlogcursorbyte=26;  //location in the RTC RAM of the cursor containing the position of the latest morning log
byte maxtimetoswitchbyte=27; //location in the RTC RAM of the amount of unchanging deciminutes that need to pass before a solenoid change happens
//28, 29 is still available

byte boileroncounterbyte=30;

int boileroncount=0;
int hotwatermax=1320; //in decidegrees F, a default
int mintemptocirculate=1100;
int mintemptocirculatesummer=1150; //in decidegrees F, a default
int mintemptocirculatewinter=1050;//in decidegrees F, a default
int loopdelay=1000; //in milliseconds, a default
char serialdata[18]=" ";
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
bool wirerequestmode=false;
byte weektimedeltainsecondsbyte=32;
byte lasttimecompensationaddedlocation=34;
byte timeoflasttimethroughloopbyte=38;
byte thingweweredoingwhenrebootedbyte=42;
unsigned long lasttimecompensationadded=0;
int weektimedeltainseconds=0;//default to 90;
unsigned long boilerlogcursor=0;
byte boilerlogcursorlocation=0;  //inside RTC ram
unsigned long boilertimestart=0;
unsigned int fuellevel=0;
unsigned long totalboilerontimefortank=0; //in deciminutes
unsigned long timeoflasttanklevelsnapshot=0; //in deciminutes
byte totalboilerontimefortanklocation=4;  //inside RTC ram
unsigned long eventlogcursor=0; //place in high EEPROM where we log events
byte eventlogcursorlocation=8;  //inside RTC ram
byte boilertimecutonlocation=12;  //inside RTC ram, so it will survive a reset
byte timeatswitchchangelocation=16;  //inside RTC ram, so it will survive a reset
unsigned int oldfuellevel=0;
byte highByte = 0x00;                             // Stores high byte from ranging
byte lowByte = 0x00;                              // Stored low byte from ranging
bool cameupwhileboilerwason=false;
bool allowwinterhotwatercirculation=false;
bool checkfuellevelnow=false;
unsigned long timeoflasttimethroughloop=0;
byte countdowntodoggoad=0;
int irVal=0;
byte activityBeforeReboot=NOSPECIFICACTIVITY;  //activityBeforeReboot can be 0=none, 1=checking fuel level

void setCurrentActivity(byte currentActivityIn)
{
	storebyte(thingweweredoingwhenrebootedbyte, currentActivityIn,2);
	//maybe not:?
	//activityBeforeReboot=currentActivityIn;
	//activityBeforeReboot=NOSPECIFICACTIVITY;
}

byte newrandom(unsigned long howsmall, unsigned long howbig)
{
	return ((analogRead(0)+ analogRead(4))* analogRead(3) + millis()) % (howbig - howsmall);
} 

void setup() 
{
	//add a delay at the beginning to allow all crazy things to stabilize, but only do it once out of 20 times
	//wdt_reset();
	wdt_disable();
	wdt_enable(WDTO_8S);
	
	byte randomer=newrandom(0, 80);
	if(randomer  ==19  ||  randomer  ==16  ||  randomer  ==15  ||  randomer  ==13)
	{
		//delay(6000);
		//Serial.begin(BAUD_RATE);
		//Serial.println("Special delay!");
		//Serial.end();
	}
	pinMode(3, INPUT); //circulator p
	int potentialhotwatermax;
	int potentialloopdelay;
	int potentialmintemptocirculatesummer; 
	int potentialmintemptocirculatewinter; 
	int potentialmaxtimetoswitch;
	int potentialboileroncount;
	

	Wire.begin();
 	
	getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
	minutecounter=deciminutessince2000();
	Serial.begin(BAUD_RATE); // use the serial port to send data back to the computer
	//old way:  in the past i would store the minute counter in EEPROM and get it back on reboot
	//so as to keep track of time across reboots. it wasn't very accurate, but it was better than the alternative
	//minutecounter=getlong(0);
	//storelong(4, minutecounter); //store occasion of last reboot at EEPROM location starting with 4
	//now i just set the minutecounter from the realtime clock with the number of deciminutes since january 1, 2000
	eventlogcursor=getlong(eventlogcursorlocation,2);
	boilerlogcursor=getlong(boilerlogcursorlocation,2);
	totalboilerontimefortank=getlong(totalboilerontimefortanklocation,2);
	//Serial.print((int)EE_1);
	//Serial.print(" ");
	//Serial.println((int)EE_2);
	pinMode(3, INPUT); //circulator pump reader
	pinMode(12, INPUT); //boiler reader
	//pinMode(11, INPUT); //season push button reader
	pinMode(13, INPUT);//this is normlly the hot water thermostat bypass, but we want it to start up as an input so it won't throw the bypass relay initially (which can make a lot of noise if it's stuck in a reboot loop).
	pinMode(8, OUTPUT); //hot water sufficiency output
	pinMode(9, OUTPUT); //slab sufficiency output
	pinMode(4, OUTPUT); //aux; used to be used for lighting a green LED indicating Solar Panel temperature being displayed
	pinMode(5, OUTPUT); //aux; used to be used for lighting a green LED indicating Basement pipe (from panel) temperature being displayed
	pinMode(6, OUTPUT); //aux; used to be used for lighting a green LED indicating after heat sink temperature being displayed
	pinMode(7, OUTPUT); //used for connecting the external i2c bus, used for reading fuel level in oil tank. normally disconnected because it is noisy and slow.
	//pin 2 still unused!
	//PWM on pin 10 is used to display temperature on the meter depending on the green LED lit
	digitalWrite(8,LOW); //turn off hot water sufficiency initially
	delay(55);    //delay to make sure two relays not thrown simultaneously, which could cause large transients
	digitalWrite(9,LOW);  //turn off slab sufficiency initially
	delay(55);    //delay to make sure two relays not thrown simultaneously, which could cause large transients
	
	potentialhotwatermax=eeprombyte2temp(getbyte(hotwatermaxbyte, 2));
	if(potentialhotwatermax!=0 && potentialhotwatermax!=2550)
	{
		hotwatermax=potentialhotwatermax;
	}
	potentialloopdelay=getbyte(loopdelaybyte,2);
	if(potentialloopdelay!=0 && potentialloopdelay!=255)
	{
		loopdelay=potentialloopdelay*100;
	}
	if(loopdelay>7000)
	{
		loopdelay=7000;  //keep watchdog from freaking out
	}
	
	potentialboileroncount=getint(boileroncounterbyte,2);
	if(potentialboileroncount!=0 && potentialboileroncount!=255)
	{
		boileroncount=potentialboileroncount;
	}
	
	potentialmintemptocirculatesummer=eeprombyte2temp(getbyte(mintemptocirculatesummerbyte,2));
	if(potentialmintemptocirculatesummer!=0 && potentialmintemptocirculatesummer!=2550)
	{
		mintemptocirculatesummer=potentialmintemptocirculatesummer;
	}
	
	potentialmintemptocirculatewinter=eeprombyte2temp(getbyte(mintemptocirculatewinterbyte,2));
	if(potentialmintemptocirculatewinter!=0 && potentialmintemptocirculatewinter!=2550)
	{
		mintemptocirculatewinter=potentialmintemptocirculatewinter;
	}
	
	potentialmaxtimetoswitch=getbyte(maxtimetoswitchbyte,2);
	if(potentialmaxtimetoswitch!=255)
	{
		maxtimetoswitchchange=potentialmaxtimetoswitch;
	}

	season=getbyte(seasonbyte,2);
	if(season!=0)
	{
		season=1;
	}
	SetSeason(season);

	solarlogcursor=getbyte(solarlogcursorbyte,2);
	if(solarlogcursor==255)
	{
		solarlogcursor=0;
	}
	if(digitalRead(12)==1)
	{
		cameupwhileboilerwason=true;
	}
	logevent(4, averageRead(3,0));
	//WriteDs1307Ram(27, 41);
	//storelong(27, 66223422, 2);
	//storelong(47, 17, 1);
	//storebyte(15, 5, 1);
	//storelong(15, 11111111, 1);
	//storelong(15, 111111117, 1);
	activityBeforeReboot=getbyte(thingweweredoingwhenrebootedbyte,2);
	//Serial.print("just after get:");
	//Serial.println((int)activityBeforeReboot);
	
	Serial.println("Starting up.");
	sendAllDefaultsToMenuSystem();
}
	
	
void loop() 
{
	byte i;
	wdt_reset();//petting the watchdog timer every time through the loop
	if(millis()>6000)//if system is in a weird beginning state, don't screw with season mode bypass relay
	{
		pinMode(13, OUTPUT); //season mode light/bypass hot water tank thermostat.  allows solar array to make hot water hotter than boiler does
		
	}
	//irVal=readIR();//turn this off for now

	
	//pure candy:  use to display something meaningful with analogWrite on atmega 0's pin 6
	if(illumination<14)
	{
		illumination=illumination+1;
	}
	else
	{
		illumination=0;
	}
	analogWrite(6, illumination* illumination); //exponential is more dramatic
	/*
	digitalWrite(13,HIGH);
	delay(2000);
	digitalWrite(13,LOW);
	delay(2000);
	*/
	/////end pure candy
	
	//Serial.println((unsigned long)getlong(15, 1));
	//writeonslave(6, 7);
	
	if(minute==0  && second==0  && hour==1  && month>2 && season==0  && outsidetemp>320  && !boilerstate)
	{
		//if it's one in the morning, the boiler is off, it's not january or february, and outside temperature is above 32F, try setting to summer mode.  if the boiler kicks in, then it's still winter, and the situation will be fixed, but otherwise perhaps i forgot to tell the controller it's time for summer
		SetSeason(1);

		//eventtype 5 is a season to summer change, quantity is outdoor temperature
		//eventtype 6 is a season to winter change, quantity is outdoor temperature
	}
	if(timeofsufficiency>0  && timeofsufficiency+10< minutecounter && ispumprunning==0 ) //we have sufficiency but the pump has been off for more than a minute
	{
	
		//this could mean the controller is set to summer but the old school rotary switches do not support summer mode, so back to winter
		SetSeason(0);
		//also make a record in the log:
		logevent(3, outsidetemp);
		
	}


	getDateDs1307(&second, &minute, &hour, &dayOfWeek, &dayOfMonth, &month, &year);
	performMenuCommand();
	//activityBeforeReboot will be FUELLEVELTESTACTIVITY if checkfuellevel hangs enough to trigger watchdog. if so, don't bother with fuel level check
	//Serial.println((int)activityBeforeReboot);
	//checkfuellevelnow=true;
	//Serial.println(checkfuellevelnow);
	//Serial.println(fuellevel);
	//Serial.print("activitybeforereboot:");
	//Serial.print((int)activityBeforeReboot);
	//Serial.print(" was okay then?:");
	//Serial.print((int)( activityBeforeReboot!=FUELLEVELTESTACTIVITY ));
	//Serial.println("");
	//checkcheckfuel
	if(  activityBeforeReboot!=FUELLEVELTESTACTIVITY  && (minute==0 && second==0 && hour!=0 || fuellevel==0  || checkfuellevelnow  || invalidFuelLevelRead ) || (second==0 && invalidFuelLevelRead))//changed to 999 while i work on shit
	{
		//only check the fuel level spontaneously once every hour or reboot

		setCurrentActivity(FUELLEVELTESTACTIVITY); //set current activity to indicate we're about to test fuel level. if it hangs, it will know after reboot not to try it for a whole up cycle, thereby allowing the device to do something useful
		
		//Serial.print("just after setting to 2:");
		//Serial.println((int)getbyte(thingweweredoingwhenrebootedbyte,2));
		
		//fuellevel = getDistanceToFuel();
		setCurrentActivity(NOSPECIFICACTIVITY); //done with risky activity
		checkfuellevelnow=false;
	}
	else if((int)activityBeforeReboot==FUELLEVELTESTACTIVITY)
	{
		setCurrentActivity(NOSPECIFICACTIVITY);
	}
	if(oldfuellevel-10>fuellevel  && oldfuellevel>0)//somebody just refilled the tank with more than 10 cm of fuel, so reset totalboilerontimefortank
	{
		totalboilerontimefortank=0;
		storelong(totalboilerontimefortanklocation,totalboilerontimefortank, 2);
		//log the tank refueling
		logevent(0, fuellevel);
	}
	if(fuellevel>0  && timeoflasttanklevelsnapshot<minutecounter-100  || timeoflasttanklevelsnapshot==0)//store the old fuel level every ten minutes
	{
		oldfuellevel=fuellevel;
		timeoflasttanklevelsnapshot=minutecounter;
	}

	//deal with the known delta that the rtc is too slow, about 61 seconds per week (can be changed with snDELTA)
	minutecounter=deciminutessince2000();
	//keep record of last time through loop
	timeoflasttimethroughloop=getlong(timeoflasttimethroughloopbyte, 2);
	if(minutecounter-100>timeoflasttimethroughloop)
	{
		//there was a freakish reset, so let's log
		//can't use logevent(7, timeweweredown) because we're not storing minute counter but instead a saved time;
		storelong(eventlogcursor,timeoflasttimethroughloop, 4);
		storebyte(eventlogcursor+4, 7,4);
		storeint(eventlogcursor+5,6*(minutecounter-timeoflasttimethroughloop), 4);//time we were down in seconds (not deciminutes)
		if(eventlogcursor<maxbigcursor)
		{
			eventlogcursor=eventlogcursor+7;
			storelong(eventlogcursorlocation, eventlogcursor, 2);
		}
		else
		{
			eventlogcursor=0;
			storelong(eventlogcursorlocation, 0, 2);
			//logevent(?, ?); it would be pointless to log a wrap around event of the log itself
		}
	
	}
	storelong(timeoflasttimethroughloopbyte,minutecounter, 2);
	weektimedeltainseconds =getint(weektimedeltainsecondsbyte, 2);
 	if(weektimedeltainseconds>32768)
	{
		weektimedeltainseconds=32768-weektimedeltainseconds;
	}
	lasttimecompensationadded =getlong(lasttimecompensationaddedlocation, 2);
	//Serial.println(weektimedeltainseconds);
	if(lasttimecompensationadded==0  || minutecounter-lasttimecompensationadded>100800)
	{
		if(lasttimecompensationadded>0)
		{
			//minutecounter=minutecounter+(weektimedeltainseconds/6);  
			//deciminutestousefultimeparts(minutecounter,1);
		}
		lasttimecompensationadded=minutecounter;
		storelong(lasttimecompensationaddedlocation,lasttimecompensationadded, 2);
	}
 
	
	
	 

	slave_communication(thisPhase, SLAVE_ADDRESS, 0,""); 
	if((boolean)digitalRead(12))
	{
		//if the boiler is on, then there's no way it's summer
		SetSeason(0);
	}
	if((boolean)digitalRead(12)!=(boolean)boilerstate)
	{
		//we're counting boiler stuff!
		if(!(boolean)boilerstate)
		{
			boileroncount++;
			boilertimestart=minutecounter; //note the time the boiler starts up so we later know how long it ran in deciminutes
			if(cameupwhileboilerwason)
			{
				boilertimestart =getlong(boilertimecutonlocation, 2); //store it in the rtc so we can retrieve it through a controller reboot
				
			}
			else
			{
				storelong(boilertimecutonlocation, boilertimestart, 2);
			}
			
	 
		}
		else
		{
			//store the time of boiler done
			storelong(boilerlogcursor, minutecounter, 3);  
			//store fuel level  -- when we can measure it, it won't be zero
			//fuellevel=getDistanceToFuel();//get the latest fuel level
			storeint(boilerlogcursor+4, fuellevel, 3);  
			// WireEepromWrite(int theDeviceAddress, unsigned int theMemoryAddress, byte theByte) 
			//store time it ran 
			if(boilertimestart==0)
			{
				boilertimestart =getlong(boilertimecutonlocation, 2);
			}
			storebyte(boilerlogcursor+6,(unsigned byte )minutecounter-boilertimestart,3);
			storebyte(boilerlogcursor+7, (unsigned byte )temp2eeprombyte(outsidetemp),3);

			if(boilerlogcursor<maxbigcursor)
			{
				boilerlogcursor=boilerlogcursor+8; //advance boiler log cursor seven bytes for next record to store
			}
			else
			{
				boilerlogcursor=0;//overflow!
				logevent(2, fuellevel);
			}
			storelong(boilerlogcursorlocation,boilerlogcursor,2);
			totalboilerontimefortank=totalboilerontimefortank+(minutecounter-boilertimestart);
			storelong(totalboilerontimefortanklocation,totalboilerontimefortank, 2);
			
		}
		cameupwhileboilerwason=false;
		storeint(boileroncounterbyte, boileroncount,2);
		boilerstate=(boolean)digitalRead(12);
	}
	
	if (season==0  && !allowwinterhotwatercirculation)
	{
		mintemptocirculate=mintemptocirculatewinter;
	}
	else
	{
		mintemptocirculate=mintemptocirculatesummer;
	}
	if(timeatswitchchange==0  || timeatswitchchange>minutecounter)
	{
		timeatswitchchange =getlong(timeatswitchchangelocation,2);
		//Serial.print("*");
		//Serial.print(timeatswitchchange);
		//Serial.println("");
		if(timeatswitchchange==0  || timeatswitchchange>minutecounter)
		{
			//Serial.print("*");
			//Serial.print(timeatswitchchange);
			//Serial.print("=");
			//Serial.print(minutecounter);
			//Serial.println("");
			timeatswitchchange=minutecounter;
			storelong(timeatswitchchangelocation,timeatswitchchange,2);
		}
	}
	
	//clear serial data
	for(i=0; i<18; i++)
	{
		serialdata[i]=0;
	}
	
	//don't need to do this any more: this dates to the ghetto rtc
	//loopcount++;
	//if(loopcount==1000) //don't want to wear out the EEPROM
	//{
		//don't need to do this anymore now that we use an RTC
		//storelong(0, minutecounter); //stores minutecounter at the bottom of EEPROM.
		//loopcount=0;
	//}
	oldhws=tentativeHotwaterSufficiency;
	oldss=tentativeSlabSufficiency;
	
	//process serial commands here

	//Serial.print("serial before process: ");
	//Serial.println((int)serval);
	//Serial.println((int)serval);
	if(serval>'9')
	{
		processcommand(serval, 0);
		serval=oldserval;
	
	}
	else if (serval==0  || serval==10) //ends up being zero or line feed under some conditions
	{
		
		serval=oldserval;
	}
	
  	//Serial.print("tick tick number: ");
	//Serial.println((int)serval);
	
	if (digitalRead(3)==HIGH)
	{
		if(ispumprunning==0)
		{
			flowrate=0;
			timebeganpumping=minutecounter;
			basementtempatpumpstart=averageRead(1,1);
		}
		ispumprunning=1;
	}
	else
	{
		ispumprunning=0;
		timebeganpumping=0;
	}
	if(countdowntodoggoad==1)
	{
		delay(20000);
		countdowntodoggoad=0;
		
	}
	if(countdowntodoggoad>1)
	{
		countdowntodoggoad--;
		Serial.println((int)countdowntodoggoad);
	}
	paneltemp=averageRead(0,0);
	basementtemp=averageRead(1,1);

	//these two will be coming off the slave arduino
	//afterloadtemp=averageRead(5,0);
	//slabtemp=averageRead(4,0);
	hotwatertemp=averageRead(2,3); 
	outsidetemp=averageRead(3,0);
	if(hotwatertemp<750)
	{
		//if hot water in tank is getting cold, it could be because we're in winter mode. switch it back to summer!
		SetSeason(1);
		
	}
	//storeifmax in EEPROM; lowstart is as follows:  
	//20 for panel
	//25 for boiler room
	//30 for after load
	//35 for slab
	//40 for hotwater tank
	//45 for outside
	storeifmax(paneltemp, 20);
	storeifmax(basementtemp, 25);
	storeifmax(afterloadtemp, 30);
	storeifmax(slabtemp, 35);
	storeifmax(hotwatertemp, 40);
	storeifmax(outsidetemp, 45);
	
	//storeifmin in EEPROM; lowstart is as follows:  
	//50 for panel
	//55 for boiler room
	//60 for after load
	//65 for slab
	//70 for hotwater tank
	//75 for outside
	storeifmin(paneltemp, 50);
	storeifmin(basementtemp, 55);
	storeifmin(afterloadtemp, 60);
	storeifmin(slabtemp, 65);
	storeifmin(hotwatertemp, 70);
	storeifmin(outsidetemp, 75);
 
	//had been using hotwatertempcuton instead of hotwatertemp
	//PAY ATTENTION, SINCE THIS IS CONFUSING: the math here keeps circulation from happening if the panel is not significantly hotter than the hot water tank.
	 
	if(!(paneltemp-100<afterloadtemp)  && paneltemp>50+hotwatertemp  && ((minutecounter-waitstart>frozenwaittime || waitstart==0)))
	{
 
		//the allowwinterhotwatercirculation logic goes down here
		if(season!=0  || allowwinterhotwatercirculation)
		{
		//only do hot water in the summer
			//Serial.println("Collected water hot enough."); 
			tentativeHotwaterSufficiency=1;
		 
		}
	}
	//Serial.println(int(season));
	//had been using outsidetempcuton instead of basementtemp
	
	//Serial.print((int) tentativeHotwaterSufficiency);
	//Serial.print(' ');
	//Serial.print((int)tentativeSlabSufficiency);
	//Serial.println("");
	//Serial.println((int)waitstart);
	
	if(((minutecounter-waitstart>frozenwaittime || waitstart==0)))
	//if(!(paneltemp-100<afterloadtemp)  && paneltemp>300+slabtemp && ((minutecounter-waitstart>frozenwaittime || waitstart==0)))
	//hotwater is already too high but to also make sure tentativeHotwaterSufficiency is off
	//if(paneltemp<outsidetempcuton)
	{
		tentativeSlabSufficiency=1;//slabon1
	}
	//Serial.print(hotwatertemp);
	//Serial.print(' ');
	//Serial.print(hotwatermax);
	//Serial.print(' ');
	//Serial.print(hotwatertemp>hotwatermax);
	//Serial.println("");
	//Serial.print((int) tentativeHotwaterSufficiency);
	//Serial.print(' ');
	//Serial.print((int)tentativeSlabSufficiency);
	//Serial.println("");
	//4/15/2013:
	if(hotwatermax<1100)//unusual condition, so flag. only happens if there is a problem with the rtc
	{	
		Serial.print(str_hwater);
		Serial.print(str_space);
		Serial.print(str_max);
		Serial.print(" is set low: ");
		Serial.println((int)hotwatermax); 
		hotwatermax=1100;
	}
	//4/18/2013:
	if(hotwatermax>2000)//unusual condition, so flag. only happens if there is a problem with the rtc
	{
		Serial.print(str_hwater);
		Serial.print(str_space);
		Serial.print(str_max);
		Serial.print(" is set high: ");
		Serial.println((int)hotwatermax); 
		hotwatermax=2000; //higher than this threatens boiling
	}
	if( hotwatertemp>hotwatermax)
	//that hotwatertemp>hotwatermax clause is to switch on slab sufficiency when
	{
		//Serial.println("Collected water too hot."); 
		//Serial.println("Hot water max:"); 
		//Serial.println((int)hotwatermax); 
		tentativeHotwaterSufficiency=0;
		tentativeSlabSufficiency=1;//slabon2
	}
	
	if( tentativeHotwaterSufficiency!=0 )
	{
		tentativeSlabSufficiency=0;
	}
	//if panel is cooler than 10 degrees plus water temp from after the load then turn off
	if(paneltemp-100<afterloadtemp  || paneltemp<mintemptocirculate) //the || paneltemp<mintemptocirculate is to make sure the
	//if( paneltemp<mintemptocirculate) //the || paneltemp<mintemptocirculate is to make sure the
	//slab doesn't get heated by a warm night in summer -- the panel has to be at least 115 degrees!
	{
		//Serial.println("Panel water not hot enough."); 
		tentativeSlabSufficiency=0;
		tentativeHotwaterSufficiency=0;
	}
 
	basementdelta=basementtemp-basementtempatpumpstart;
	//if basementdelta is greater than 8 degrees and we haven't calculated a flow rate, do so now
	if (ispumprunning==1  && flowrate==0  && basementdelta>80  && basementtemp!=0  && basementtempatpumpstart!=0 && minutecounter>0   )
	{
		//we have approx. 75 feet of pipe connecting panels to the basment
		//this works out to 1.72 gallons of fluid.
		timeforflowtogo75feet=minutecounter-timebeganpumping;
		if(timeforflowtogo75feet>0)
		{
			flowrate=1720/(timeforflowtogo75feet*10); //flowrate is also times ten to avoid decimal places
		}  
	}
	//if more than 15 minutes of pumping and no hot water, there's a problem
	//so wait an hour
	
	if((tentativeSlabSufficiency==1 || tentativeHotwaterSufficiency==1)  && basementdelta<50  && minutecounter-timebeganpumping>minstillfrozen && ispumprunning==1 )
	{
		//this really shouldn't happen unless outside temps are less than 16 degrees
		if (outsidetemp<160)
		{
			//Serial.println("Determined Frozen"); 
			tentativeSlabSufficiency=0;
			tentativeHotwaterSufficiency=0;
			waitstart=minutecounter;
			timebeganpumping=0;
			basementtempatpumpstart=0;
			isfrozen=1;
			logevent(1, outsidetemp);
		}
	}
	
	if(wearelogging)
	{
		if(basementtempmax<temp2eeprombyte(basementtemp))
		{
			basementtempmax=temp2eeprombyte(basementtemp);
			timeofbasementtempmax=minutecounter;
		}
		
		if(temp2eeprombyte(basementtemp)+6<basementtempmax) //if the current temp has dropped more than 6 degrees from the max then we consider the peak to have passed.  In this case we're dealing with degrees, not decidegrees.
		{
			//add the stored peak information to the log
			storebyte((solarlogcursor*10)+85, basementtempmax,0);
			storelong((solarlogcursor*10)+86, timeofbasementtempmax,0);
			wearelogging=false;
			basementtempmax=0;
			//advance the log cursor
			if(solarlogcursor<maxShortLog) //don't log more than 80 of the last mornings -- we don't have room for more in a 328
			{
				solarlogcursor=solarlogcursor+1; //unlike other cursors, this one advances by one;  details of the format are calculated from it
			}
			else
			{
				solarlogcursor=0;
			}
			//write solarlogcursor to EEPROM
			storebyte(solarlogcursorbyte, solarlogcursor,2);
		}
	}
	
	if (oldhws!=tentativeHotwaterSufficiency  || oldss!=tentativeSlabSufficiency )
	{
		//Serial.print("pleh-maxtimeatswitchchange:");
		//Serial.print((int)maxtimetoswitchchange);
		//Serial.print(" timeatswitchchange:");
		//Serial.print(timeatswitchchange);
		//Serial.print(" minutecounter:");
		//Serial.print(minutecounter);
		//Serial.println(' ');
		if(minutecounter-timeatswitchchange>7200 && timeatswitchchange!=0) //7200 is twelve hours in deciminutes
		{
			//if more than twelve hours has passed since a swithchange then it's morning in America
			wearelogging=true;
			storebyte((solarlogcursor*10)+80, temp2eeprombyte(paneltemp),0);
			
			storelong((solarlogcursor*10)+81, minutecounter,0);
		}
	
		timeatswitchchange=minutecounter;
		storelong(timeatswitchchangelocation,timeatswitchchange,2);
		
	}
 
	if(forceSolarSufficiency)
	{
		tentativeHotwaterSufficiency=1;
	}
	if(forceSlabSufficiency)
	{
		tentativeSlabSufficiency=1;
	}
        //Serial.print(timeatswitchchange);
        //Serial.print(" ");
        //Serial.print(maxtimetoswitchchange);
        //Serial.print(" ");
	//Serial.print(minutecounter);
	if(timeatswitchchange+maxtimetoswitchchange<minutecounter || minutecounter<0  || maxtimetoswitchchange>60) //only make a relay change after the maxtimetoswitchchange cooling off
	{
		//Serial.print("meh:-maxtimeatswitchchange:");
		
		//Serial.print((int)maxtimetoswitchchange);
		//Serial.print(" timeatswitchchange:");
		//Serial.print(timeatswitchchange);
		//Serial.print(" minutecounter:");
		//Serial.print(minutecounter);
		//Serial.println(' ');


		if (tentativeHotwaterSufficiency==1  && !forceInsufficiency || forceSolarSufficiency)
		{
			timeofsufficiency=minutecounter;
			completeHotwaterSufficiency=1;
			digitalWrite(8,HIGH);
			delay(55);    //delay to make sure two relays not thrown simultaneously
		}
		else 
		{
			timeofsufficiency=0;
			completeHotwaterSufficiency=0;
			digitalWrite(8,LOW);
			delay(55); 
			if(paneltempatshutoff==0)
			{
				
				paneltempatshutoff=paneltemp;
				basementtempatshutoff=basementtemp;
				afterloadtempatshutoff=afterloadtemp;
				slabtempatshutoff=slabtemp;
			}
		}
		//Serial.print("tentativeSlabSufficiency:");
              //  Serial.print(tentativeSlabSufficiency);
               //	Serial.print("forceInsufficiency:");
               // Serial.print(forceInsufficiency);
              //  Serial.println(forceInsufficiency);
		if (tentativeSlabSufficiency==1 && !forceInsufficiency || forceSlabSufficiency)
		{
			timeofsufficiency=minutecounter;
			completeSlabSufficiency=1;
			digitalWrite(9,HIGH);
			delay(55);  
		}
		else  
		{
		
			timeofsufficiency=0;
			completeSlabSufficiency=0;
			digitalWrite(9,LOW);
			delay(55);  
		}
	}
	
	//don't bother with this any more, but do use pin 6 to indicate things.
	//this code block illuminates the correct LED and sends the corresponding analog of the temperature to the meter
	//digitalWrite(4, LOW);
	//digitalWrite(5, LOW);
	//digitalWrite(6, LOW);
	//digitalWrite(thisPhase, HIGH);
	if(thisPhase<7) //we have four phases (4,5,6,7) as we loop through to help with some things -- it's a legacy of the LED pattern used in Solar Controller II
	{
		thisPhase++;
	}
	else
	{
		thisPhase=4;
	}
	//Serial.println(serval);
	serialout(serval);
	//in case loopdelay is something stupid
	if(loopdelay<10000)
	{
		delay(loopdelay); 
	}
	else
	{
		delay(200); 
	}
	
	//old minutecounter increment code:
	//probably no need for this code:
	//thesemins=  millis() / 6000;
	//if (lastmins<thesemins  || thesemins+4800 < lastmins)
	//{
		//lastmins=thesemins;
		//we get it straight from the rtc with every loop, so no need for the following line:
		//minutecounter++;
	//}
	
	if (Serial.available()) 
	{
		// read the most recent byte (which will be from 0 to 255)
		oldserval=serval;
		serval = Serial.read() ;
		//Serial.print("this serial: ");
		//Serial.println(serval);
	}
	
	
	//read the season push button, which uses the line to display the 'now showing ambient temperature' LED
	//pinMode(7, INPUT);
	if(digitalRead(7)==HIGH)//season button is pushed, time to toggle season!
	{
		//skip this part for now;  season will be automatically determined
		if(season==1)
		{
			//season=0;
			//digitalWrite(13,HIGH);
		}
		else
		{	
			//season=1;
			//digitalWrite(13,LOW);
		}
		//EEPROM.write(seasonbyte, (byte)season);
	}
	SetSeason(season); //this is to make sure season relay state eventually gets set if it was missed in the first 7000 seconds
	//pinMode(7, OUTPUT);
}//end main loop

void processcommand(byte initialcommand, byte type) //type==0 means serial, 1 means via I2C from the slave
{
 
	char readbyte=str_space;
	byte cursor=0;
	char secondcharacter=str_space;
	unsigned long possiblefirstnumber=-1;
	
	byte foreepromwrite;
	long rawserial;
	int thisbyte=-1;
	byte i;
	//Serial.print("initialcommand: ");
	//Serial.print(initialcommand);
	//Serial.print(" type: ");
	//Serial.println((int)type);
	bool doSave=false;
	if(initialcommand>64  && initialcommand<173) //letters, mostly
	{

		while(readbyte>0)
		{
			
			if(type==0)
			{
				readbyte=Serial.read();
			}
			else if(type==1)
			{
				if(Wire.available())
				{
					readbyte = Wire.read();	
					//Serial.println(readbyte);
				}
				else
				{
					return;	
				}
			}
			serialdata[cursor]=  readbyte;
			cursor++;
			if(secondcharacter==str_space)
			{
				secondcharacter=readbyte;
			}
			if(readbyte==str_space  && possiblefirstnumber==-1)
			{
				possiblefirstnumber=myStringToNumber(serialdata);
				cursor=0;
			}
		}
		serialdata[--cursor]='\0';
		rawserial=myStringToNumber(serialdata);
		
		if (initialcommand=='?')  //display help
		{
			displayhelp();
		}
		else if (initialcommand=='m')  //show memory
		{
			Serial.print(str_freemem);
			Serial.print(get_free_memory());
			Serial.println("");
		}
		else if (initialcommand=='r')
		{
			if(secondcharacter=='b') //'rb' means "reboot"
			{
				Serial.print(str_reboot);
				Serial.print(str_ing);
				Serial.print("...");
				Serial.println("");
				void(* resetFunc) (void) = 0;
				resetFunc();
				setup();
			}
		}
		else if (initialcommand=='y')
		{
			if(secondcharacter=='t')		//'yt' means "you there?"
			{
				Serial.println("Yes I am here.");
			}
		
		}
		else if (initialcommand=='d')
		{
			if(secondcharacter=='x')//'dx': display hot water max
			{
			
				Serial.print(str_max);
				Serial.print(str_space);
				Serial.print(str_hwater);
				Serial.print(str_colonspace);
				Serial.println((int)eeprombyte2temp(getbyte(hotwatermaxbyte, 2)));
			}
			else if (secondcharacter=='C') //'dC': display millis
			{
	 			Serial.print("Millis: ");
				Serial.println(millis());
			}
			else if(secondcharacter=='s')		//'ds' means "display solar log"
			{
				displaymorningsolarlog();
			}
			else if(secondcharacter=='l') 	//'dl' means "display long"
			{
				Serial.print(str_long);//EEPROM long word starting at 
				Serial.print(str_word);
				Serial.print(str_spaceatspace);
				Serial.print(rawserial);
				Serial.print(": ");
				Serial.print((unsigned long)getlong(rawserial, 0));
				Serial.println("");
			}
			else if(secondcharacter=='n') 	//'dn' means "display clock compensation"
			{
				Serial.print("Clock compensation: ");  
				//Serial.print(str_space);
				Serial.print((int)weektimedeltainseconds);
				Serial.println("");
			}
			else if(secondcharacter=='e') 	//'de' means "display extremes, in this case those extremes and times
			{
				printextremes();
			}
			else if(secondcharacter=='f') 	//'df' means "display fuel info
			{
				printfuelinfo();
			}
			else if(secondcharacter=='b') //'db' means "display boiler log 
			{
				DisplayBoilerLog();
			}
			else if(secondcharacter=='v') //'dv' means "display event log 
			{
				DisplayEventLog();
			}
 
			else if(secondcharacter=='t') //'dt' means "display boiler count"
			{
				Serial.print(str_boiler);
				Serial.print(str_space);
				Serial.print(str_count);
				Serial.print(str_colon);
				Serial.println(getint(boileroncounterbyte,2));
			}
			else if(secondcharacter=='c') //'dc' means "display cursors"
			{
				Serial.print(str_boiler);
				Serial.print(str_space);
				Serial.print(str_cursor);
				Serial.print(str_colon);
				Serial.print(boilerlogcursor);
				Serial.print(str_space);
				Serial.print("(");
				Serial.print(boilerlogcursor*100/maxbigcursor);
				Serial.print("%");
				Serial.print(")");
				Serial.print(str_space);
				Serial.print(str_event);
				Serial.print(str_space);
				Serial.print(str_cursor);
				Serial.print(str_colon);
				Serial.print(eventlogcursor);
				Serial.print(str_space);
				Serial.print("(");
				Serial.print(eventlogcursor*100/maxbigcursor);
				Serial.print("%");
				Serial.print(")");
				Serial.println("");
			}
			else if(secondcharacter=='r')//'dr' means "display real time variables"
			{
				RTCVars();
			}
			else //display byte data in EEPROM
			{
				Serial.print(str_byte);
				Serial.print(str_spaceatspace);
				Serial.print(rawserial);
				Serial.print(": ");
				Serial.print((unsigned long)getbyte(rawserial,0));
				Serial.println("");
			}
	
			
			
		}
		else if (initialcommand=='c')
		{
			if (secondcharacter=='e') //'ce': clear extreme data
			{
				for(i=20; i<80; i++)
				{
					storebyte(i, 255,0);
				}
				Serial.print(str_temperature);
				Serial.print(str_space);
				Serial.print(str_extreme);
				Serial.print(str_s);
				Serial.print(str_space);
				Serial.print(str_cleared);
				Serial.println("");
			}
			else if (secondcharacter=='f') //'cf': clear fuel info
			{
				totalboilerontimefortank=0;
				storelong(totalboilerontimefortanklocation,totalboilerontimefortank, 2);
				Serial.print(str_fuel);
				Serial.print(str_space);
				Serial.print(str_info);
				Serial.print(str_space);
				Serial.print(str_cleared);
				Serial.println("");
			}
			else if (secondcharacter=='F') //'cF': clear fuel level reading
			{
				fuellevel=0;
				checkfuellevelnow=true;
				invalidFuelLevelRead=true;
				setCurrentActivity(NOSPECIFICACTIVITY);
				activityBeforeReboot=NOSPECIFICACTIVITY;
				Serial.print(str_fuel);
				Serial.print(str_space);
				Serial.print(str_level);
				Serial.print(str_space);
				Serial.print(str_cleared);
				Serial.println("");
			}
			else if (secondcharacter=='b') //'cb': clear boiler log location
			{
 				storelong(boilerlogcursorlocation, 0, 2);
				boilerlogcursor=0;
				Serial.print(str_boiler);
				Serial.print(str_space);
				Serial.print(str_log);
				Serial.print(str_space);
				Serial.print(str_location);
				Serial.print(str_space);
				Serial.print(str_reset);
				Serial.println("");
			}
			else if (secondcharacter=='v') //'cv': clear event log location
			{
	 
 				storelong(eventlogcursorlocation, 0, 2);
				eventlogcursor=0;
				Serial.print(str_event);
				Serial.print(str_space);
				Serial.print(str_log);
				Serial.print(str_space);
				Serial.print(str_location);
				Serial.print(str_space);
				Serial.print(str_reset);
				Serial.println("");
			}
			else if(secondcharacter=='i'  || secondcharacter=='s') //'ci' or 'cs' means "clear in/sufficiency force"
			{	
				forceSlabSufficiency=false;
				forceSolarSufficiency=false;
				forceInsufficiency=false;
				Serial.print(str_in);
				Serial.print(str_slash);
				Serial.print(str_suff);
				Serial.print(str_cleared);
				Serial.println("");
			}
		}
		else if (initialcommand=='s')
		{
			if (secondcharacter=='x') //'sx': set switchover temp from hot water to slab (makes sense mostly in the summer)
			{
				doSave=true;
				hotwatermax=rawserial*10;
				foreepromwrite=rawserial;//temp2eeprombyte(rawserial);//accept temperature as is, not in decidegrees
				thisbyte=hotwatermaxbyte;
				Serial.print(str_max);
				Serial.print(str_space);
				Serial.print(str_hwater);
				Serial.print(str_space);
				Serial.print(str_temperature);
				Serial.print(str_space);
				Serial.print(str_setto);
				Serial.print(str_space);
				Serial.print((int)foreepromwrite);
				Serial.print(str_degreesf);
				Serial.println("");
			}
			if (secondcharacter=='n') //'sn': set clock compensation)
			{
				doSave=true;
				storeint(weektimedeltainsecondsbyte, rawserial ,2);//used to store it in atmega 0, now store in the rtc
				weektimedeltainseconds=rawserial;
				Serial.print("Clock delta ");
				Serial.print(str_setto);
				Serial.print(str_space);
				Serial.print((int)rawserial);
				Serial.print(str_space);
				Serial.print(str_second);
				Serial.print("s per week.");
				Serial.println("");
			}
			else if(secondcharacter=='c'  || secondcharacter=='W' || secondcharacter=='T' || secondcharacter=='D'  )//'sc', 'sT', 'sD', 'sW' means "set clock -- that is, real time clock"
			{
				Serial.print("clock set to: ");  
				//time has to be passed in as scSECONDS,MINUTES,HOURS,DAYOFWEEK,DAY,MONTH,YEAR (no spaces).  items left out will default to existing
				//Serial.print(rawserial);
				//sc10,34,17,2,14,3,11
				//old way:
				//storelong(0, rawserial);
				//minutecounter=rawserial;
				
				//new way
				byte dpcursor;
				dpcursor=0;
				byte j;
				byte whichsettingregime;
				int thisint;
				byte b;
				bool bwlTimeScanDone;
				bwlTimeScanDone=false;
				thisint=0;
					
				
				if(secondcharacter=='c')
				{
					whichsettingregime=0;
				}
				else if(secondcharacter=='W')
				{
					whichsettingregime=1;
				}
				else if(secondcharacter=='T')
				{
					whichsettingregime=2;
				}
				else if(secondcharacter=='D')
				{
					whichsettingregime=3;
				}
				for(j=0; j<30; j++)
				{
					b=serialdata[j];
					if(b>='0'  && b<='9')
					{
						thisint=thisint*10+(b-48);
					}
				
					if(b==' ' || b==':' || b=='-' || b==',' || b==0  && !bwlTimeScanDone)
					{
						
						if(dpcursor==clocksettingregime[whichsettingregime][0]  )
						{
							second=thisint;
							
							Serial.print(str_space);
							Serial.print(str_second);
							Serial.print(str_s);
							Serial.print(str_colon);
							Serial.print(thisint);
						}
						else if(dpcursor==clocksettingregime[whichsettingregime][1]  )
						{
							minute=thisint;
							
							Serial.print(" minutes:");
							Serial.print(thisint);
						}
						else if(dpcursor==clocksettingregime[whichsettingregime][2]  && thisint!=0)
						{
							hour=thisint;
							
							Serial.print(" hours:");
							Serial.print(thisint);
						}
						else if(dpcursor==clocksettingregime[whichsettingregime][3] && thisint!=0)
						{
							dayOfWeek=thisint;
							
							Serial.print(" dayofweek:");
							Serial.print(thisint);
						}
						else if(dpcursor==clocksettingregime[whichsettingregime][4] && thisint!=0)
						{
							dayOfMonth=thisint;
							
							Serial.print(" day:");
							Serial.print(thisint);
						}
						else if(dpcursor==clocksettingregime[whichsettingregime][5] && thisint!=0)
						{
							month=thisint;
							
							
							Serial.print(" month:");
							Serial.print(thisint);
						}
						else if(dpcursor==clocksettingregime[whichsettingregime][6] && thisint!=0)
						{
							year=thisint;
							Serial.print(" year:");
							Serial.print(thisint);
						}
				
						setDateDs1307(second,   minute, hour, dayOfWeek, dayOfMonth, month, year);
						thisint=0;
						dpcursor++;
					}
					if(b==0)
					{
						bwlTimeScanDone=true;
					}
					
				}
				Serial.print("Year: ");
				Serial.print((int)year);
				Serial.println("");
				minutecounter=deciminutessince2000();
				Serial.println("");
				
			}
			else if(secondcharacter=='b')//'sb' means "set boiler count"
			{
				doSave=true;
				Serial.print(str_boiler);
				Serial.print(rawserial);
				Serial.println("");
				storeint(boileroncounterbyte, rawserial,2);
			}
			
			else if (secondcharacter=='t') //'st': set time of delay of main loop in deciseconds
			{
				doSave=true;
				loopdelay=rawserial*100;
				foreepromwrite=rawserial;
				thisbyte=loopdelaybyte;
				Serial.print(str_loop);
				Serial.print(str_space);
				Serial.print(str_delay);
				Serial.print(str_setto);
				Serial.print(str_space);
				Serial.print((int)loopdelay);
				Serial.print(str_space);
				Serial.print(str_milli);
				Serial.print(str_second);
				Serial.print(str_s);
				
				Serial.println("");
			}
			else if (secondcharacter=='d') //'sd': set time of delay of relay action until this number of unchanging deciseconds
			{
				doSave=true;
				maxtimetoswitchchange=rawserial;
				foreepromwrite=rawserial;
				thisbyte=maxtimetoswitchbyte;
				Serial.print(str_pump);
				Serial.print(str_space);
				Serial.print(str_switch);
				Serial.print(str_space);
				Serial.print(str_delay);
				Serial.print(str_setto);
				Serial.print(str_space);
				Serial.print((int)maxtimetoswitchchange * 6);
				Serial.print(str_space);
				Serial.print(str_second);
				Serial.print(str_s);
				
				Serial.println("");
			}
			else if(secondcharacter=='m'  || secondcharacter=='l'  )  //'sm' or 'sl'; set minimum summer temperature to run panel
			{
				doSave=true;
				mintemptocirculatesummer=rawserial*10;
				foreepromwrite=rawserial;//temp2eeprombyte(rawserial);//accept temperature as is
				thisbyte=mintemptocirculatesummerbyte;
				mintemptocirculate=mintemptocirculatesummer;
				Serial.print(str_min);
				Serial.print(str_space);
				Serial.print(str_temperature);
				Serial.print(str_for);
				Serial.print(str_summerdef);
				Serial.print(str_space);
				Serial.print(str_forcirculation);//minimum summer panel temperature for circulation
				Serial.print(str_space);
				Serial.print(str_setto);
				Serial.print(str_space);
				Serial.print((int)foreepromwrite);
				Serial.print(str_degreesf);
				Serial.println("");
			}
			else if(secondcharacter=='w' )  //'sw'; set minimum winter temperature to run panel
			{
				doSave=true;
				mintemptocirculatewinter=rawserial*10;
				mintemptocirculate=mintemptocirculatewinter;
				foreepromwrite=rawserial;//temp2eeprombyte(rawserial);//accept temperature as is
				thisbyte=mintemptocirculatewinterbyte;
				Serial.print(str_min);
				Serial.print(str_space);
				Serial.print(str_temperature);
				Serial.print(str_for);
				Serial.print(str_winterdef);
				Serial.print(str_space);
				Serial.print(str_forcirculation);//minimum panel temperature for circulation
				Serial.print(str_space);
				Serial.print(str_setto);
				Serial.print(str_space);
				Serial.print((int)foreepromwrite);
				Serial.print(str_degreesf);
				Serial.println("");
			}
			else if(secondcharacter=='s') //'ss' means "season set"
			{
				doSave=true;
				season=rawserial;
				foreepromwrite=rawserial;
				thisbyte=seasonbyte;
				Serial.print(str_season);
				Serial.print(str_space);
				Serial.print(str_setto);
				Serial.print(str_space);
				if(season)
				{
					Serial.print(str_summerdef);
				}
				else
				{
					Serial.print(str_winterdef);
				}
				SetSeason(season);
				Serial.println("");
			}
			else if(secondcharacter=='f') //'sf' means "set/show frozen"
			{
			
				Serial.print(str_space);
				if(isfrozen==1)
				{
					isfrozen=0;
					waitstart=0;
					Serial.print(str_setto);
					Serial.print(str_not);
					Serial.print(str_frozen);
				}
				else
				{	
					isfrozen=1;
					waitstart=minutecounter;
					Serial.print(str_setto);
					Serial.print(str_frozen);
				}
				Serial.println("");
			}

		
		}
		else if(initialcommand=='f') 
		{
			if(secondcharacter=='i') //'fi': force insufficiency
			{
				forceInsufficiency=true;
				Serial.print(str_in);
				Serial.print(str_suff);
				Serial.print(str_forced);
				Serial.println("");
			}
			else if(secondcharacter=='s') //'fs': force solar sufficiency
			{
				forceSolarSufficiency=true;
				//Serial.print(str_in);
				Serial.print(str_solar);
				Serial.print(str_suff);
				Serial.print(str_forced);
				Serial.println("");
			}
			else if(secondcharacter=='S') //'fS': force slab sufficiency
			{
				forceSlabSufficiency=true;
				//Serial.print(str_in);
				Serial.print(str_slab);
				Serial.print(str_suff);
				Serial.print(str_forced);
				Serial.println("");
			}
		}
		else if(initialcommand=='g')
		{
			if(secondcharacter=='d') //'gd' means "goad dog" -- basically test to see if watchdog trips during a 20 second delay after so many times through the loop.
			//enter gd 20 to see the dog goaded 20 loops in the future. system should reset nicely at that point and start up again.
			//while the countdown happens, every other line is the count until goading
			{
		 
				countdowntodoggoad=rawserial;
		 
			}
 
		
		}
		else if(initialcommand=='x') //'xf' means "check fuel" in oil tank for boiler
		{
			if(secondcharacter=='f')
			{
				checkfuellevelnow=true;
			}
		}
		else if(initialcommand=='u') //'us' means "update slave" for menu system;
		{
			if(secondcharacter=='s')
			{
				sendAllDefaultsToMenuSystem();
				return;
			}
		}
		else if(initialcommand=='w')
		{
			if(possiblefirstnumber!=-1  && (secondcharacter=='e'  || secondcharacter=='b')) //'we' or 'wb' means "write eeprom" or "write byte"
			{
				//storelong(int lowstart, long datain)
				Serial.print(str_data);
				Serial.print(str_spaceatspace);
				Serial.print((int)possiblefirstnumber);
				Serial.print(str_space);
				Serial.print(str_setto);
				Serial.print(str_space);
				Serial.print((int)rawserial);
				Serial.println("");
				storebyte(possiblefirstnumber, rawserial,0);
			}
			else if (possiblefirstnumber>-1  && secondcharacter=='l')//'wl' means "write long"
			{
				//doSave=true;
				//storelong(int lowstart, long datain)
				Serial.print(str_long);//long data beginning at
				//Serial.print(str_space)
				Serial.print(str_word);
				Serial.print(str_spaceatspace);
				Serial.print((int)possiblefirstnumber);
				Serial.print(str_space);
				Serial.print(str_setto);
				Serial.print(str_space);
				Serial.print((unsigned long)rawserial);
				Serial.println("");
				storelong(possiblefirstnumber, rawserial,0);
				possiblefirstnumber=-1;
				rawserial=0;
				secondcharacter=str_space;
			}
			else if (  secondcharacter=='p')//'wp' means "write panel"
			{
				//storelong(int lowstart, long datain)
				//doSave=true;
				slave_communication(4, SLAVE_ADDRESS, 0, serialdata);
			}
			//slave_communication(byte thisPhase, int slaveAddress, byte infoIWant, char* textOverride) 
		
		}
		if(thisbyte!=-1   && doSave)
		{
			//Serial.println((int)thisbyte);
			//Serial.println((int)foreepromwrite);
			Serial.println("data saved");
			storebyte(thisbyte, foreepromwrite,2);//used to store it in atmega 0, now store in the rtc
			sendAllDefaultsToMenuSystem();  //update the menu system with new defaults
		}

	}		
	if(type==0)
	{
		Serial.flush();
	}
	
}

void displaymorningsolarlog()
{
  byte i;
  printstars(18);
  Serial.print(str_morninglogs);
  printstars(36);
  Serial.println(""); 
  Serial.print(str_panel);
  printtabs(2);

  Serial.print(str_time);
  printtabs(4);
  Serial.print(str_basement);
  printtabs(1);
  Serial.print(str_time);
  Serial.println("");

  Serial.print(str_start);
  printtabs(2);
  Serial.print(str_max);
  printtabs(4);
  Serial.print(str_max);
  printtabs(2);
  Serial.print(str_max);
  Serial.println("");


  for(i=0; i<maxShortLog+1; i++)
  {
    if(i==solarlogcursor)
    {
      Serial.print("->");
    }
    else
    {
      printchars(2, str_space, 0);
    }
    Serial.print((int)getbyte(80+(i*10),0));
    printtabs(2);
	
	deciminutestousefultimeparts(getlong(81+(i*10),0),0);
   
    printtabs(2);
    Serial.print((int)getbyte(85+(i*10),0));
    printtabs(2);
 
	deciminutestousefultimeparts(getlong(86+(i*10),0),0);
    Serial.println("");
  }
  printstars(68);
  Serial.println("");
  Serial.println("");
}

void printextremes()
{		
	printstars(19);
	Serial.print(str_space);
	Serial.print(str_extreme);
	Serial.print(str_space);
	Serial.print(str_data);
	Serial.print(str_space);
	printstars(39);
	Serial.println("");
	printtabs(3);
	Serial.print(str_max);
	Serial.print(str_tab);
	Serial.print(str_time);
	Serial.print(str_tab);
	Serial.print(str_tab);
	Serial.print(str_tab);
	Serial.print(str_min);
	Serial.print(str_tab);
	Serial.print(str_time);
	Serial.println("");
	Serial.print(str_panel);
	printchars(4, str_space, 0);
	printtabs(2);
	Serial.print((int)getbyte(20,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(21,0),0);
	Serial.print(str_tab);
	Serial.print((int)getbyte(50,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(51,0),0);
	Serial.println("");
	
	Serial.print(str_basement);
	printchars(1, str_space, 0);
	printtabs(2);
	Serial.print((int)getbyte(25,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(26,0),0);
	Serial.print(str_tab);
	Serial.print((int)getbyte(55,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(56,0),0);
	Serial.println("");
	
	Serial.print(str_afterload);
	printtabs(2);
	Serial.print((int)getbyte(30,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(31,0),0);
	Serial.print(str_tab);
	
	Serial.print((int)getbyte(60,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(61,0),0);
	Serial.println("");
	
	Serial.print(str_slab);
	printchars(5, str_space, 0);
	printtabs(2);
	Serial.print((int)getbyte(35,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(36,0),0);
	Serial.print(str_tab);
	Serial.print((int)getbyte(65,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(66,0),0);
	Serial.println("");
	
	Serial.print(str_hwater);
	printchars(1, str_space, 0);
	printtabs(2);
	Serial.print((int)getbyte(40,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(41,0),0);
	Serial.print(str_tab);
	Serial.print((int)getbyte(70,0));
	Serial.print(str_tab);
	
	deciminutestousefultimeparts(getlong(71,0),0);
	Serial.println("");
	
	Serial.print(str_outdoor);
	printchars(2, str_space, 0);
	printtabs(2);
	Serial.print((int)getbyte(45,0));
	Serial.print(str_tab);
	deciminutestousefultimeparts(getlong(46,0),0);
	Serial.print(str_tab);
	Serial.print((int)getbyte(75,0));
	Serial.print(str_tab);
	
	deciminutestousefultimeparts(getlong(76,0),0);
	Serial.println("");
	
	printstars(72);
	Serial.println("");
	Serial.println("");
}

void storeifmax(int tempin, int lowstart)
{
  //lowstart is as follows:  
  //20 for panel
  //25 for boiler room
  //30 for after load
  //35 for slab
  //40 for hotwater tank
  //45 for outside
  //format is: record temp (a byte - in degrees, not decidegrees) followed by a long representing time.
  byte oldtemp;
  byte bytetempin;
  oldtemp=getbyte(lowstart,0);
  bytetempin=temp2eeprombyte(tempin);
  if(oldtemp<bytetempin  || oldtemp==255)
  {
    storebyte(lowstart, bytetempin,0);
    storelong(lowstart+1, minutecounter,0);
  }

}
void storeifmin(int tempin, int lowstart)
{
  //lowstart is as follows:  
  //50 for panel
  //55 for boiler room
  //60 for after load
  //65 for slab
  //70 for hotwater tank
  //75 for outside
  //format is: record temp (a byte - in degrees, not decidegrees) followed by a long representing time.
  byte oldtemp;
  byte bytetempin;
  oldtemp=getbyte(lowstart,0);
  bytetempin=temp2eeprombyte(tempin);
  if(oldtemp>bytetempin  || oldtemp==0)
  {
    storebyte(lowstart, bytetempin,0);
    storelong(lowstart+1,minutecounter,0);
  }
}

extern int __bss_end;
extern int *__brkval;

int get_free_memory()
{
  int free_memory;

  if((int)__brkval == 0)
    free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}

unsigned long powerof(long intin, byte powerin)
//raises intin to the power of powerin
//not using "pow" saves me a lot of Flash memory
{
  unsigned long outdata=1;
  byte k;
  for (k = 1; k <powerin+1; k++)
  {
    outdata=outdata*intin;
  }
  return outdata;
}

long myStringToNumber(char datain[18])
//parses a string into a long, August 9 2008
{
	int i;
	int thischar;
	int scale=1;
	int negscale=1;
	unsigned long out=0;
	for (i = 18; i > -1; i--)
	{
		thischar=datain[i];
		if( thischar=='\0')//clear the number if we're reading from beyond the zero terminator -- total primitive C thing
		{
			out=0;
			scale=1;
		}
		else if(thischar>47  && thischar<58)
		{
			out=out+(thischar-48)*powerof(10, scale-1);
			scale++;
		}
		else if(thischar=='-' )
		{
			negscale=-1  ;
		}
	}
	if(scale==1)
	{
		out=-1;
	}
	return negscale * out;
}

byte temp2eeprombyte(unsigned long temp)
//chucks the tenths of a degree because who cares for this app
{
  byte out=0;
  out = temp/10;
  return out;
}

unsigned long eeprombyte2temp(byte eeprombyte)
{
  unsigned long out=0;
  out = eeprombyte * 10;
  return out;
}




int calculateTemperature(unsigned long raw, int type)
//turns raw probe data into an actual temperature in decidegrees
{
	//types here:
	//paneltemp=calculateTemperature(0,0);
	//basementtemp=calculateTemperature(1,1);
	//afterloadtemp=  Arduino2, analog0 type 5
	//slabtemp=Arduino2, analog1, type 6
	//hotwatertemp=calculateTemperature(3,0); 
	//outsidetemp=calculateTemperature(2,0);
	unsigned long out=0;
	if (type==1 ) //also used for the garage temperature
	{
		//was 2207 as of 1/1/2013
		out=  (raw *  2257 )/1000 ;
		//was 1940
		//then1760
		//then 1770
		out=1820-out;
	}
	else if (type==6  )
	{
		out=  (raw *  1207 )/1000 ;
		//was 1940
		//then1760
		//then 1770
		out=1580-out;
	}
	else if (type==5 )
	{
		//out=(raw * 834)/1000;
		//out=   1178-out;
		//out=1896-(raw*2017/1000); //was in jan 2013
		//out=1896-(700/347)*raw; //was 1996-
		
		out=1755-(raw*1608)/1000;
		if(1==2)
		{
			Serial.print((int)raw);
			Serial.print(" ");
			Serial.print(out);
			Serial.println("");
		}
		//y=-700/347x+692480/347
	}
	else if (type==0  || type==3 )
	{
		//  out=(raw * 1934)/1020;
		// out=   1688-out;
		//this should be pretty accurate given a 15K balance resistor to the 10k thermistor
		//out=  1338-(315*raw)/223;
		out=1732-raw*2;
		//out=1564-(350/197)*raw;
	}
	else
	{
		out=(raw * 1934)/1000;
		out=   1788-out;
	}
	if  (type==3) //possible hack for temporary hotwater temp probe
	{
		out= out +620;
	}
  return out;
}

unsigned long averageRead(int pin, int type)
//originally I read the input several times and took an average, but that later proved unnecessary
{
	int j=0;
	int sum=0;
	int times=1;
	int out=0;
	for (j=0; j<times; j++)
	{
		sum+=  calculateTemperature(analogRead(pin), type);
	}
  //Serial.println (sum);
  out= sum/times;
  sum=0;
  j=0;
  if(pin==0)
  {
    //for testing!
    //out=1520;
  }
  return out;
}

void slopeTesterChunk(char* descr,  char numeric, char type)
{
	long rawvalue;
	long value;
	if(numeric>58)
	{
		if(numeric=='X')
		{
			if(type=='5')
			{
				rawvalue=slabtempraw;
			}
			else
			{
				rawvalue=afterloadtempraw;
			}
		}
		else
		{
			rawvalue=garageTempraw;
		}
		value=calculateTemperature(rawvalue, type-48);
	}
	else
	{
		rawvalue=analogRead(numeric-48);
		value=averageRead(numeric-48,type-48);
	}
	if(numeric==type)
	{
		Serial.print(numeric);
	}
	else
	{
		Serial.print(numeric);
		Serial.print(',');
		Serial.print(type);
	}
	Serial.print(':');
	Serial.print(descr);
	Serial.print(':');
	Serial.print(rawvalue);
	Serial.print(':');
	Serial.print(value);
	Serial.print(' ');
}

void serialout(char serval)
{
  if(serval=='0')
  {
  	//the main data stream that i like to monitor via the serial port: 
	//Serial.print(minutecounter);     //in deciminutes, that is, six second granularity  
	//Serial.print(str_space); 
	deciminutestousefultimeparts(minutecounter, 0); 
    Serial.print(str_space); 
	frd(paneltemp, 0);
    Serial.print(str_space); 
	frd(basementtemp, 0); 
    Serial.print(str_space); 
    frd(afterloadtemp, 0);    
    Serial.print(str_space); 
    frd(slabtemp, 0);
    Serial.print(str_space); 
    frd(hotwatertemp, 0);  
    Serial.print(str_space); 
    frd(outsidetemp, 0);         
    Serial.print(str_space); 
	frd(garageTemp, 0);   
	Serial.print(str_space);   
    Serial.print((int)tentativeHotwaterSufficiency); 
    Serial.print((int)completeHotwaterSufficiency);
    Serial.print(str_space); 
    Serial.print((int)tentativeSlabSufficiency); 
    Serial.print((int)completeSlabSufficiency);
    Serial.print(str_space); 
    Serial.print((int)ispumprunning); 
    Serial.print(str_space); 
    Serial.print(digitalRead(12)); 
    Serial.print(str_flowrate); 
    Serial.print((int)flowrate); 
    Serial.print(str_space); 
    //Serial.print(str_season); 
    //Serial.print(str_colon); 
	if(season)
	{
		 Serial.print(str_summerdef);
	}
	else
	{
		Serial.print(str_winterdef);
	}
    //Serial.print((int)season);
	Serial.print(str_space); 
	Serial.print(str_fuel);
	Serial.print(str_colon); 
	Serial.print((int)fuellevel);
	/*
	Serial.print(str_space);
	Serial.print("IR:");
	Serial.print((int) irVal);
	*/
  }
  else if (serval=='1')
  {
  
  
	//handy for tweaking raw-to-temperature slope formulae
	//paneltemp=averageRead(0,0); basementtemp=averageRead(1,1); hotwatertemp=averageRead(2,3); outsidetemp=averageRead(3,0); slabtempraw, afterloadtempraw, garageTempraw
	slopeTesterChunk(str_panel,'0','0');
	slopeTesterChunk(str_basement,'1','1');
	slopeTesterChunk(str_hwater, '2','3');
	slopeTesterChunk(str_outdoor, '3','0');
	slopeTesterChunk(str_slab, 'X','5');
	slopeTesterChunk(str_afterload, 'X','6');
	slopeTesterChunk("garage", 'Y','1');
  
     		
  }
  else if  (serval=='2')
  {
    
	Serial.print(str_panel); 
	Serial.print(str_colon);
	Serial.print(paneltemp);
	Serial.print(str_slash); 
	Serial.print(analogRead(0));
	Serial.print(str_space);
	Serial.print(str_basement); 
	Serial.print(str_colon);
	Serial.print(basementtemp);
	Serial.print(str_slash); 
	Serial.print(analogRead(1));
	Serial.print(str_space);
	Serial.print(str_afterload); 
	Serial.print(str_colon);
	Serial.print(afterloadtemp);
	Serial.print(str_slash); 
	Serial.print(afterloadtempraw);
	Serial.print(str_space);
	Serial.print(str_hwater);
	Serial.print(str_colon);
	Serial.print(hotwatertemp);
	Serial.print(str_slash); 
	Serial.print(analogRead(2));
     		
  }
  else if  (serval=='3')
  {
    
	Serial.print(str_time);
	Serial.print(str_space);
	Serial.print(str_togo75);		 
	Serial.print(str_space);
	Serial.print((int)timeforflowtogo75feet);

  }
  else if(serval=='4')
  {
 
	Serial.print(str_time);
	Serial.print(str_space);
	Serial.print(str_at);
	Serial.print(str_space);
	Serial.print(str_switch);
	Serial.print(str_space);
	Serial.print(str_change);
	Serial.print(str_colon);
	deciminutestousefultimeparts(timeatswitchchange, 0); 
	//Serial.print(timeatswitchchange); 
	Serial.print(str_space);
	Serial.print(str_time);
	Serial.print(str_space);
	Serial.print("since switchchange");
	Serial.print(str_colon);
	Serial.print((minutecounter-timeatswitchchange)/10);
	Serial.print(str_space);
	Serial.print(str_minute);  
	Serial.print(str_s);
	Serial.print(str_space); 
	Serial.print(str_hwater);
	Serial.print(str_space);
	Serial.print(str_suff);
	Serial.print((int)tentativeHotwaterSufficiency); 
	Serial.print((int)completeHotwaterSufficiency); 
	Serial.print(str_space); 
	Serial.print(str_slab);
	Serial.print(str_space);
	Serial.print(str_suff);
	Serial.print((int)tentativeSlabSufficiency);  
	Serial.print((int)completeSlabSufficiency);  
	 
     		
  }
  else if(serval=='5')
  {
    Serial.print(str_isfrozen);
    Serial.print((int)isfrozen); 
    Serial.print(str_space); 
  }
  if(serval=='6')
  {
	Serial.print(hour, DEC);
	Serial.print(":");
	Serial.print(minute, DEC);
	Serial.print(":");
	Serial.print(second, DEC);
	Serial.print("  ");
	Serial.print(month, DEC);
	Serial.print("/");
	Serial.print(dayOfMonth, DEC);
	Serial.print("/");
	Serial.print(year, DEC);
	Serial.print("  day of week:");
	Serial.print(dayOfWeek, DEC);
	//end rtc part of loop
  
  }
   	if(serval=='7')
	{
		Serial.print(str_hwater);
		Serial.print(str_max);
		Serial.print(":");
		Serial.print(hotwatermax);
		for(byte i=0; i<2; i++)
		{
			Serial.print(" ");
			Serial.print(str_min);
			Serial.print(str_temperature);
			Serial.print("tocirculate");
			if(i==0)
			{
				Serial.print(str_summerdef);
			}
			else
			{
				Serial.print(str_winterdef);
			}
			
			Serial.print(":");
			if(i==0)
			{
				Serial.print(mintemptocirculatesummer);
			}
			else
			{
				Serial.print(mintemptocirculatewinter);
			}
		 
		}
 
  	}
	if(serval=='8')
	{

	
	   /*
		Serial.print(str_space);
		Serial.print(str_panel); 
		Serial.print(str_at); 
		Serial.print(str_shutoff); 
		Serial.print(str_colon); 
		Serial.print(paneltempatshutoff);      
		Serial.print(str_space);
		Serial.print(str_basement); 
		Serial.print(str_at); 
		
		Serial.print(str_shutoff); 
		Serial.print(str_colon); 
		
		Serial.print(basementtempatshutoff); 
		
		Serial.print(str_space);
		
		//28666
		Serial.print(str_afterload); 
		Serial.print(str_at); 
		Serial.print(str_shutoff); 
		Serial.print(str_colon); 
		Serial.print(afterloadtempatshutoff);   
		
		Serial.print(str_space);
		Serial.print(str_panel); 
		Serial.print(str_at); 
		Serial.print(str_slab); 
		Serial.print(str_colon); 
		Serial.print(slabtempatshutoff); 
		Serial.print(str_space); 
		Serial.print(basementtempatpumpstart); 
		Serial.print(str_space);
		Serial.print(str_time);
		Serial.print(str_space);
		Serial.print(str_began); 
		Serial.print(str_space);
		Serial.print(str_pumping); 
		Serial.print(timebeganpumping); 
		Serial.print(str_space);
		Serial.print(str_waitstart);
		Serial.print(str_colon);
		 */ 
		
	}
  if (serval<'9')//don't gimme blank lines if there is nothing
  {
    Serial.println(""); 
  }
}

void printstars(byte numberofchars)
{
	printchars(numberofchars, '*', 0);
}

void printtabs(byte numberofchars)
{
	printchars(numberofchars, str_tab, 0);
}

void sendDefaultToMenuSystem(byte datain, byte address)
{
	byte thesevals[5];
	thesevals[0]='!';
	thesevals[1]=(byte)16 ; //write eeprom code
	thesevals[2]= 0;
	
	thesevals[3]= address;
	thesevals[4]= datain;
	Wire.beginTransmission(SLAVE_ADDRESS);              
	Wire.write(thesevals,5);                     
	Wire.endTransmission();
}

void sendAllDefaultsToMenuSystem()
{
	//SECONDS,MINUTES,HOURS,DAYOFWEEK,DAY,MONTH,YEAR,SEASON,MAXHOTWATER,MINSUMMER,MINWINTER
	//second, minute, hour, dayOfWeek, dayOfMonth, month, year;
	sendDefaultToMenuSystem(second, 0);
	sendDefaultToMenuSystem(minute, 1);
	sendDefaultToMenuSystem(hour, 2);
	sendDefaultToMenuSystem(dayOfWeek, 3);
	sendDefaultToMenuSystem(dayOfMonth, 4);
	sendDefaultToMenuSystem(month, 5);
	sendDefaultToMenuSystem(year, 6);
	sendDefaultToMenuSystem(season, 7);
	//send these as degrees, not decidegrees
	sendDefaultToMenuSystem(hotwatermax/10, 8);
	sendDefaultToMenuSystem(mintemptocirculatesummer/10, 9);
	sendDefaultToMenuSystem(mintemptocirculatewinter/10, 10);
	sendDefaultToMenuSystem(loopdelay/100, 11);
	
	/*
	if(1==2)
	{
	Serial.print(second);
	Serial.print(" ");
	Serial.print(minute);
	Serial.print(" ");
	Serial.print(hour);
	Serial.print(" ");
	Serial.print(dayOfWeek);
	Serial.print(" ");
	Serial.print(dayOfMonth);
	Serial.print(" ");
	Serial.print(month);
	Serial.print(" year:");
	Serial.print(year);
	Serial.print(" seas:");
	Serial.print(season);
	Serial.print(" hwmx:");
	Serial.print(hotwatermax);
	Serial.print(" minsum:");
	Serial.print(mintemptocirculatesummer);
	Serial.print(" minwin:");
	Serial.print(mintemptocirculatewinter);
	Serial.println(" ");
	}
	*/
	
}

void printchars(byte numberofchars, char charin, byte mode)
{
	byte i;
	for(i=0; i<numberofchars; i++)
	{
		if(mode==0)
		{
			Serial.print(charin);
		}
		else if(mode==1)
		{
			Wire.write(charin);
		}
	}
}
void WireEepromRead(int theDeviceAddress, unsigned int theMemoryAddress, int theByteCount, byte* theByteArray) 
{
	int theByteIndex;
	for (theByteIndex = 0; theByteIndex < theByteCount; theByteIndex++) 
	{
		Wire.beginTransmission(theDeviceAddress);
		Wire.write((byte)((theMemoryAddress + theByteIndex) >> 8));
		Wire.write((byte)((theMemoryAddress + theByteIndex) >> 0));
		Wire.endTransmission();
		//delay(5);
		Wire.requestFrom(theDeviceAddress, sizeof(byte));
		theByteArray[theByteIndex] = Wire.read();
	}
}


byte WireEepromReadByte(int theDeviceAddress, unsigned int theMemoryAddress) 
{
	byte theByteArray[sizeof(byte)];
	WireEepromRead(theDeviceAddress, theMemoryAddress, sizeof(byte), theByteArray);
	return (byte)(((theByteArray[0] << 0)));
}

void WireEepromWrite(int theDeviceAddress, unsigned int theMemoryAddress, int theByteCount, byte* theByteArray) 
{
  for (int theByteIndex = 0; theByteIndex < theByteCount; theByteIndex++) 
  {
    Wire.beginTransmission(theDeviceAddress);
    Wire.write((byte)((theMemoryAddress + theByteIndex) >> 8));
    Wire.write((byte)((theMemoryAddress + theByteIndex) >> 0));
    Wire.write(theByteArray[theByteIndex]);
    Wire.endTransmission();
    delay(5);
  }
}

void WireEepromWriteOneByte(int theDeviceAddress, unsigned int theMemoryAddress, byte theByte) 
{
 
	Wire.beginTransmission(theDeviceAddress);
	Wire.write((byte)((theMemoryAddress ) >> 8));
	Wire.write((byte)((theMemoryAddress) >> 0));
	Wire.write(theByte);
	Wire.endTransmission();
	delay(5);
 
}
void storebyte(unsigned int low,   byte datain, byte type)
{
	if(type==0)
	{
		EEPROM.write(low, datain);
	}
	else if(type==1)
	{
		
		byte thesevals[5];
		thesevals[0]='!';
		thesevals[1]=(byte)2 ; //write eeprom code
		thesevals[2]= (byte)low/256;
		
		thesevals[3]= (byte)(low-(thesevals[2]*256));
		thesevals[4]=(byte)datain;
		Wire.beginTransmission(SLAVE_ADDRESS);              
		Wire.write(thesevals,5);                     
		Wire.endTransmission();
		//delay(10);
	}
	else if(type==2)
	{
		WriteDs1307Ram(low,datain);
	}
	else if(type==3)
	{
		WireEepromWriteOneByte(EEPROM_LOW_MEM_ADDRESS, low,  datain);
	}
	else if(type==4)
	{
		WireEepromWriteOneByte(EEPROM_HIGH_MEM_ADDRESS, low,  datain);
	}
}


//not used in masterman
byte readIR()
{
	byte thisbyte, thisotherbye, thisotherotherbyte;
	byte thesevals[4];
	byte count;
	bool bwlFail=true;
	thesevals[0]='!';
	thesevals[1]=(byte) 14; //read IR code
	thesevals[2]= 0;
 	
	thesevals[3]= 0;
	//Serial.print((int)thesevals[2]);
	//Serial.print(" ");
	//Serial.print((byte)thesevals[3]);
	//Serial.println("");
	Wire.beginTransmission(SLAVE_ADDRESS);              
	Wire.write(thesevals,4);                
	Wire.endTransmission();
	 	
		
 	//delay(22);
 	while(bwlFail)
	{
 		Wire.requestFrom(SLAVE_ADDRESS,3); 
		count=0;
		while(Wire.available()  && count<100)
		{
			//thisbyte = Wire.read(); 
			//}
			thisotherotherbyte = Wire.read(); 
			thisotherbye = Wire.read(); 
			thisbyte = Wire.read(); 
			if(thisotherbye!=thisbyte  || thisbyte!=thisotherotherbyte)
			{
				bwlFail=true;
			}
			else
			{
				bwlFail=false;
			}
			count++;
		}
		
	}
	return thisbyte;
}

byte getbyte(unsigned int low,  byte type)
{
	byte thisbyte, thisotherbye, thisotherotherbyte;
 	byte thesevals[4];
	bool bwlFail=true;
	byte count;
	if(type==0)
	{
		thisbyte=EEPROM.read(low);
	}
	else if(type==1)
	{
	 
		thesevals[0]='!';
		thesevals[1]=(byte) 1; //read eeprom code
		thesevals[2]= (byte)low/256;
 
		thesevals[3]= (byte)(low-(thesevals[2]*256));
		//Serial.print((int)thesevals[2]);
		//Serial.print(" ");
		//Serial.print((byte)thesevals[3]);
		//Serial.println("");
		Wire.beginTransmission(SLAVE_ADDRESS);              
		Wire.write(thesevals,4);                
		Wire.endTransmission();
	 	
		
 		//delay(22);
	 	while(bwlFail)
		{
	 		Wire.requestFrom(SLAVE_ADDRESS,3); 
			count=0;
			while(Wire.available()  && count<100)
			{
				//thisbyte = Wire.read(); 
				//}
				thisotherotherbyte = Wire.read(); 
				thisotherbye = Wire.read(); 
				thisbyte = Wire.read(); 
				if(thisotherbye!=thisbyte  || thisbyte!=thisotherotherbyte)
				{
					bwlFail=true;
				}
				else
				{
					bwlFail=false;
				}
				count++;
			}
			
		}
		//Serial.println("");
	}
	else if(type==2) //RTC memory
	{
		thisbyte =(int)ReadDs1307Ram(low);
	}
	else if(type==3)
	{
		thisbyte =WireEepromReadByte(EEPROM_LOW_MEM_ADDRESS, low);
	}
	else if(type==4)
	{
		thisbyte =WireEepromReadByte(EEPROM_HIGH_MEM_ADDRESS, low);
	}
	return thisbyte;
}


void performMenuCommand()
{
	byte thisbyte, thisotherbye, thisotherotherbyte;
 	byte thesevals[4];
	bool bwlFail=true;
 
	thesevals[0]='!';
	thesevals[1]=(byte) 21; //command pickup code
	thesevals[2]= 0;

	thesevals[3]= 0;
	//Serial.print((int)thesevals[2]);
	//Serial.print(" ");
	//Serial.print((byte)thesevals[3]);
	//Serial.println("");
	Wire.beginTransmission(SLAVE_ADDRESS);              
	Wire.write(thesevals,4);                
	Wire.endTransmission();
 	
	
	//delay(22);
 	Wire.requestFrom(SLAVE_ADDRESS,30); 
	thisbyte=Wire.read();
	//Serial.println((int)thisbyte);
	processcommand(thisbyte, 1);
 	
}


void storelong(unsigned int lowstart, unsigned long datain, byte type)
//stores a long in EEPROM beginning at lowstart (Least Significant Byte first)
//type is as follows: 0: master atmega eeprom, 1: slave atmega eeprom, 2: Ds1307 ram, 3:low EEPROM, 4: high EEPROM
{
	byte j;
	byte thisbyte;
	byte * theByteArray;
	unsigned long thispower;
	int theDeviceAddress;
 
	for(j=0; j<4; j++)
	{
		thispower=powerof(256, j);
		thisbyte=datain/thispower;
		if(type==13  || type==14)//had to make these be 13 and 14 instead of 3 and 4 because it wasn't working
		{
			theByteArray[j]=thisbyte;
		}
		else
		{
			storebyte(lowstart+j,thisbyte, type);
		}
		datain=datain-(thisbyte * thispower);
	}
	if (type==13  || type==14)  //had to make these be 13 and 14 instead of 3 and 4 because it wasn't working
	{
		if(type==3)
		{
			theDeviceAddress=(int)EEPROM_LOW_MEM_ADDRESS;
		}
		else
		{
			theDeviceAddress=(int)EEPROM_HIGH_MEM_ADDRESS;
		}
		WireEepromWrite(theDeviceAddress, lowstart, 4, theByteArray) ;
	
	}
	return;
}

unsigned long getlong(unsigned int lowstart, byte type)
//returns the long value stored in EEPROM
//type is as follows: 0: master atmega eeprom, 1: slave atmega eeprom, 2: Ds1307 ram, 3 low eeprom, 4 low eeprom
{
	byte i;
	byte thisbyte;
	unsigned long out=0;
	for(i=0; i<4; i++)
	{
	
		thisbyte=getbyte(lowstart+i, type);
		out=out + (thisbyte * powerof(256, i));
	}
	if(type==1)
	{	
		Wire.beginTransmission(SLAVE_ADDRESS); 
		Wire.write(itoa(1000+lowstart,10));
		Wire.write("|257|");
		Wire.endTransmission(); 
	}
	return out;
}

void storeint(unsigned int lowstart, unsigned int datain, byte type)
//stores a long in EEPROM beginning at lowstart (Least Significant Byte first)
//type is as follows: 0: master atmega eeprom, 1: slave atmega eeprom, 2: Ds1307 ram
{
	byte j;
	byte thisbyte;
	unsigned long thispower;
	for(j=0; j<2; j++)
	{
		thispower=powerof(256, j);
		thisbyte=datain/thispower;
		storebyte(lowstart+j,thisbyte, type);
		datain=datain-(thisbyte * thispower);
	}
	return;
}

int getint(unsigned int lowstart, byte type)
//returns the long value stored in EEPROM
//type is as follows: 0: master atmega eeprom, 1: slave atmega eeprom, 2: Ds1307 ram
{
	byte i;
	byte thisbyte;
	unsigned long out=0;
	for(i=0; i<2; i++)
	{
		thisbyte=getbyte(lowstart+i, type);
		out=out + (thisbyte * powerof(256, i));
	}
	return out;
}

int slave_communication(byte thisPhase, int slaveAddress, byte infoIWant, char* textOverride) 
{ 
	//int slaveAddress=SLAVE_ADDRESS;
	int outSlave;
	int count;
	int temp1;
	int temp2;
	int temp3;
	int temp4;
	int thisAnalog;
	byte thisbyte[32];
	unsigned long thislong=0;
	bool gotvaliddata=false;
	count=0;
	
	
	char thesevals[4];
	thesevals[0]='!';
	thesevals[1]=char(9); //want slave packet
	Wire.beginTransmission(slaveAddress);              
	Wire.write(thesevals);                        
	Wire.endTransmission();
	//delay(10);
	
	//Wire.beginTransmission(slaveAddress);
	//Wire.write('!!!!!!!!!!!');
 	//Wire.endTransmission(); 

	//Wire.beginTransmission(slaveAddress); 
  	//where i would send a command byte but am not doing so
	Wire.requestFrom(slaveAddress,32); 
	
	
	//Serial.println(' ');
	while(Wire.available()  && count<40)
	{
		
		gotvaliddata=true;
		thisbyte[count]=Wire.read();
		//Serial.print('*');
		//Serial.print((int)thisbyte[count]);
		if(count % 2 ==1)
		{
			//Serial.print("  " );
           
			thisAnalog=thisbyte[count]*256+thisbyte[count-1];
			//Serial.println(thisAnalog );
		}
		else if (count==12)
		{
			//Serial.println(" found byte:");
			//Serial.print(0+thisbyte[count]);
		}
		else if (count>12)
		{
			//what i'm working on
			//datain=datain-(thisbyte * thispower);
			//Serial.print('*');
			//Serial.print(count);
			//Serial.print('+');
			//Serial.print((int)thisbyte[count]);
			//Serial.println("");
			thislong=thislong + (int)thisbyte[count] * powerof(256, count-13);
		}
		else
		{
			//Serial.print(" " );
		}
		
		if(count==1)
		{
			temp1=thisAnalog;
		}
		if(count==3)
		{
			temp2=thisAnalog;
		}
		if(count==5)
		{
			temp3=thisAnalog;
		}
		if(count==7)
		{
			temp4=thisAnalog;
		}
		//if count is above 17 then we're getting data from the Arduino that connects to the slave via serial port
		if(count==23)
		{
			garageTempraw=thisAnalog;
			int prospectivegarageTemp=calculateTemperature(garageTempraw, 1);
			if(prospectivegarageTemp>0  && prospectivegarageTemp<1500)
			{
				garageTemp=prospectivegarageTemp;
			}
		}
		if(count==31)//was 25 before i used the capacitor probe
		{
			//if(thisAnalog>0  && thisAnalog<1023)
			
			if(thisAnalog>700  )
			{
				fuellevel=(((long)thisAnalog-900) * 1000)/450 ;
				if(fuellevel>1000)
				{
					fuellevel=1000;
				}
			}
		 	
		}
		if(count==infoIWant)
		{
			outSlave=thisAnalog;
		}
 		//if(SLAVE_RANGEFINDER==slaveAddress  )
		//if(count==21  || count==22  ||   count==23 ||   count==24)
		if(1==2)
		{
			Serial.print(count);
			Serial.print(" ");
			
			Serial.print((int)thisbyte[count]);
			Serial.print(" ");
			
			//Serial.print(count);
			//Serial.print(" ");
			
			Serial.print(thisAnalog);
			Serial.println(" ");
		}
		count++;
	}
	Wire.endTransmission(); 
	//Serial.println(thislong);
	//global temps from the slave get set here
	if(slaveAddress ==SLAVE_ADDRESS  && gotvaliddata)
	{
		afterloadtempraw=temp1;
		slabtempraw=temp2;
		afterloadtemp=calculateTemperature(temp1, 5);
		slabtemp=calculateTemperature(temp2, 6);

	}
	else
	{
		//garageTempraw=temp3;
		//Serial.print("graw: ");
		//Serial.println(garageTempraw);
		//if(garageTempraw>0  && garageTempraw<1033)
		{
			//garageTemp=calculateTemperature(garageTempraw, 1);
		}
	}
	
	if(1==2)
	{
		thisbyte[0]=0;
		thisbyte[1]=0;
		thisbyte[2]=0;
		thisbyte[3]=0;
		thisbyte[4]=0;
		thisbyte[5]=0;
		thisbyte[6]=0;
		thisbyte[7]=0;
	}
	if(infoIWant>0  )
	{
	
		return outSlave;
	}
	/*
	Serial.print("phase:");
	Serial.print((int)thisPhase);
	Serial.println(" ");
	*/
	//Serial.println("");
	//the part where we tell the menu what to display
	if(!wirerequestmode  || 1==1)
	{
		Wire.beginTransmission(slaveAddress); 
		//sample config strings for sending info to a slave arduino:
		//remember not to send more than 32 characters at a time, or you will swamp the I2C buffer
		//the config strings can put a message at a cursor coordinate on the LCD, for example
		//4|1|hello world will put the message 'hello world' 4 positions to the left and one position down from the top left corner.
		//the config strings can also put bytes in the eeprom
		//just add 1000 to the eeprom address and send your value in as ONETHOUSAND_PLUS_EEPROMADDRESS|VALUE
		//if that value is greater than 255, then a read is done and returned as value#12 in the 14 values that come back with every I2C interaction
		//the config strings can also put values on Atmega pins
		//just add 100 to the pin number and send your value in as ONEHUDNRED_PLUS_PINNUMBER|VALUE
		//if that value is  is 256, a digital read is performed and the result is returned as value#13 in the 14 values that come back with every I2C interaction
		//if that value is greater than 256, an analog read is performed on the analog in pins and is returned as value #13 in the 14 values
		//actually, you wouldn't use this feature (which returns one too few bytes) since all the analog pins are read and returned in the lower 12 bytes returned with every I2C interaction
		
		char bytetosend;
		char * cpaneltemp;
		char * cbasementtemp;
		char seasonindication;
		//int i;
		//for(i=0; i<15; i++)
		{
			//bytetosend=stufftosend[i];
			
		}
		if(season==0)
		{
			seasonindication='W';
		}
		else
		{
			seasonindication='S';
		}
		
		if(strlen(textOverride)>1)
		{
			Serial.print("wrote '");
			
			Serial.print(textOverride);
			Serial.print("' on the LCD.");
			Serial.println("");
			Wire.write("0|0|");
			Wire.write(textOverride);
			Wire.write(' ');
 			Wire.endTransmission(); 
			 
		
		}
			
		else if(thisPhase==4  || thisPhase==5)
		{
			//Serial.println((int)slaveAddress);
		  	Wire.write("0|0|");
			Wire.write(seasonindication);
			Wire.write(' ');
			Wire.write(str_panel);
			Wire.write(str_colon);
			printchars(2, ' ', 1);
		 
			frd(paneltemp,1);
			printchars(4, ' ', 1);
			Wire.endTransmission(); 
			Wire.beginTransmission(slaveAddress);
			Wire.write("0|1|");
			Wire.write(str_basement);
			Wire.write(str_colon);
			Wire.write(str_space);
			frd(basementtemp,1);
			printchars(4, ' ', 1);
			Wire.endTransmission();
			Wire.beginTransmission(slaveAddress);
			Wire.write("0|2|");
			Wire.write(str_afterload);
			Wire.write(str_colon);
			frd(afterloadtemp,1);
			printchars(4, ' ', 1);
			Wire.endTransmission(); 
			Wire.beginTransmission(slaveAddress);
			Wire.write("0|3|");
			Wire.write(str_hwater);
			Wire.write(str_colon);
			Wire.write(str_space);
			frd(hotwatertemp,1);
			printchars(4, ' ', 1);
			Wire.endTransmission(); 
		}
		else
		{
			Wire.write("0|0|");
			Wire.write(seasonindication);
			Wire.write(' ');
			Wire.write(str_slab);
			Wire.write(str_colon);
			printchars(3, ' ', 1);
		
		 
			frd(slabtemp,1);
			printchars(4, ' ', 1);
			Wire.endTransmission(); 
			Wire.beginTransmission(slaveAddress);
			Wire.write("0|1|");
			Wire.write(str_outdoor);
			Wire.write(str_colon);
			Wire.write(str_space);
			Wire.write(str_space);
			frd(outsidetemp,1);
			printchars(4, ' ', 1);
			Wire.endTransmission();
			Wire.beginTransmission(slaveAddress);
			Wire.write("0|2|");
			Wire.write("analogx1:");
			Wire.write(temp3);
			printchars(4, ' ', 1);
			Wire.endTransmission(); 
			Wire.beginTransmission(slaveAddress);
			Wire.write("0|3|");
			//Wire.write("analogx2:");
			Wire.write("IR      :");
			//Wire.write(temp4);
			Wire.write((int)irVal);
			printchars(4, ' ', 1);
			Wire.endTransmission(); 
		
		}
		
		
		Wire.beginTransmission(slaveAddress);
		wirerequestmode=true;
	}
 
	return outSlave;
}   














 
char* itoa(int val, int base)
{
	static char buf[32] = {0};
	int i = 30;
	for(; val && i ; --i, val /= base)
	{
		buf[i] = "0123456789abcdef"[val % base];
	}
	return &buf[i+1];
}

//////////////////rtc section///////////////////////
// some rtc code from:
// Maurice Ribble
// 4-17-2008
// http://www.glacialwanderer.com/hobbyrobotics

byte decToBcd(byte val)
{
	return ( (val/10*16) + (val%10) );
}

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
	return ( (val/16*10) + (val%16) );
}

// Stops the DS1307, but it has the side effect of setting seconds to 0
// Probably only want to use this for testing
/*void stopDs1307()
{
  Wire.beginTransmission(DS1307);
  Wire.write(0);
  Wire.write(0x80);
  Wire.endTransmission();
}*/

// 1) Sets the date and time on the ds1307
// 2) Starts the clock
// 3) Sets hour mode to 24 hour clock
// Assumes you're passing in valid numbers
void setDateDs1307(byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year)          // 0-99
{
   Wire.beginTransmission(DS1307_ADDRESS);
   Wire.write(0);
   Wire.write(decToBcd(second));    // 0 to bit 7 starts the clock
   Wire.write(decToBcd(minute));
   Wire.write(decToBcd(hour));      // If you want 12 hour am/pm you need to set
                                   // bit 6 (also need to change readDateDs1307)
   Wire.write(decToBcd(dayOfWeek));
   Wire.write(decToBcd(dayOfMonth));
   Wire.write(decToBcd(month));
   Wire.write(decToBcd(year));
   //delay(5);
   Wire.endTransmission();
   //Serial.println("time set");
}

// Gets the date and time from the ds1307
void getDateDs1307(byte *second,
          byte *minute,
          byte *hour,
          byte *dayOfWeek,
          byte *dayOfMonth,
          byte *month,
          byte *year)
{
  // Reset the register pointer
  Wire.beginTransmission(DS1307_ADDRESS);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_ADDRESS, 7);

  // A few of these need masks because certain bits are control bits
  *second     = bcdToDec(Wire.read() & 0x7f);
  *minute     = bcdToDec(Wire.read());
  *hour       = bcdToDec(Wire.read() & 0x3f);  // Need to change this if 12 hour am/pm
  *dayOfWeek  = bcdToDec(Wire.read());
  *dayOfMonth = bcdToDec(Wire.read());
  *month      = bcdToDec(Wire.read());
  *year       = bcdToDec(Wire.read());
  if(*year>40)//sanity check
  {
  	*second=0;
	*minute=0;
	*hour=12;
	*dayOfWeek=1;
	*dayOfMonth=16;
	*month=2;
	*year=14;
  }
}

void WriteDs1307Ram(int address, byte data)
{
	Wire.beginTransmission(DS1307_ADDRESS);		// Select DS1307
	Wire.write(address+8);							// address location starts at 8, 0-6 are date, 7 is control
													//where address is 0-55 
	Wire.write(data);								// send data
	Wire.endTransmission();
}

int ReadDs1307Ram(byte address)
{
	Wire.beginTransmission(DS1307_ADDRESS);		// Select DS1307
	Wire.write(address+8);
  	Wire.endTransmission();
	Wire.requestFrom(DS1307_ADDRESS, 1);
	//Wire.write(address+8);							// address location starts at 8, 0-6 are date, 7 is control
													//where address is 0-55 
	return (int)Wire.read();
}
////end rtc section////////////

//extra date/time functions:
long dayoftoday(int yin, int mmin, int din)
{
 	int out;
	mmin = (mmin + 9) % 12;
	yin   = yin - mmin/10;
	out=365*yin + (int)yin/4 - (int)yin/100 + (int)yin/400 + (int)(mmin*306 + 5)/10 + (din - 1);
	return (out);
}

void pwlvin(byte inval)
{
	if((int)inval<10)
	{
		Serial.print("0");
	}
	Serial.print((int)inval);
}

void deciminutestousefultimeparts(unsigned long deciminutes, byte type)
{
	unsigned long y, ddd, mi, mm, dd, g, rdeciminutes, hours, minutes, seconds;
	g=deciminutes/14400;
	rdeciminutes=deciminutes-g*14400;
	hours=rdeciminutes/600;
	rdeciminutes=rdeciminutes-hours*600;
	minutes=rdeciminutes/10;
	seconds=(rdeciminutes-minutes*10)*6;
	y = (10000*g + 14780)/3652425;
	ddd = g - (365*y + y/4 - y/100 + y/400);
	if (ddd < 0)
	{
		y = y - 1;
		ddd = g - (365*y + y/4 - y/100 + y/400);
	}
	mi = (100*ddd + 52)/3060;
	mm = (mi + 2)%12 + 1;
	y = y + (mi + 2)/12;
	dd = ddd - (mi*306 + 5)/10 + 1;
	if(type==0)//print to serial
	{
		pwlvin(y);
		Serial.print("-");
		pwlvin(mm);
		Serial.print("-");
		pwlvin(dd);
		Serial.print(" ");
		pwlvin(hours);
		Serial.print(":");
		pwlvin(minutes);
		Serial.print(":");
		pwlvin(seconds);
	}
	else if(type==1)
	{
		setDateDs1307(seconds, minutes, hours, dayOfWeek, dd, mm, y);//set time -- don't know what to do about dayofweek yet
	}
	else if(type==2)
	{
	
	}
}

unsigned long deciminutessince2000()
{
	//globals we reference: byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
	long days, out;
	days=dayoftoday(year,month,dayOfMonth);
 
	out=days*24*600 + hour*600 + minute *10 + (int)second/6;
	return (out);
}

void frd(int temp, byte mode)
{
	char buf[32] = {0};
	int i = 30;
	int count=0;
	if(temp<100)
	{
		if(mode==0)
		{
			Serial.print(" ");
		}
		else if(mode==1)
		{
			Wire.write(" ");
		}
	}
 	if(temp<1000)
	{
		if(mode==0)
		{
			Serial.print(" ");
		}
		else if(mode==1)
		{
			Wire.write(" ");
		}
	}
	for(; temp && i ; --i, temp /= 10)
	{
		if(count==1)
		{
			buf[i] ='.';
			i--;
		
		}
		buf[i] = "0123456789"[temp % 10];
		count++;
	}

	for(i=0; i<32; i++)
	{

				
		if(buf[i]>='0' && buf[i]<='9'  || buf[i]=='.')
		{
			if(mode==0)
			{
			
				Serial.print(buf[i]);
			}
			else if(mode==1)
			{
	 
				Wire.write(buf[i]);
			}
		}
	}
}
 
void DisplayBoilerLog()
{
	int i;
	for(i=0; i<boilerlogcursor; i+=8)
	{
		deciminutestousefultimeparts((unsigned long)getlong(i,3), 0);
		Serial.print(": ");
		Serial.print(getint(i+4,3));
		Serial.print(" ");
		Serial.print((unsigned int) getbyte(i+6,3)*6);
		Serial.print(" ");
		Serial.print((unsigned int) getbyte(i+7,3) );
		Serial.println("");
		wdt_reset();//gotta pet the dog during these data dumps
	}
}

void DisplayEventLog()
{
	int i;
	for(i=0; i<eventlogcursor; i+=7)
	{
		//Serial.print(getlong(i,4));
		//Serial.print(str_space);
		deciminutestousefultimeparts((unsigned long)getlong(i,4), 0);
		Serial.print(str_space);
		Serial.print(str_event);
		Serial.print(str_space);
		Serial.print(str_type);
		Serial.print(str_colon);
		Serial.print(str_space);
		Serial.print((int) getbyte(i+4,4));
		Serial.print(" measured quantity: ");
		Serial.print(getint(i+5,4));
		Serial.println("");
		wdt_reset();//gotta pet the dog during these data dumps
	}
}

int getDistanceToFuel()
{ 
	//Serial.println("wa" );
	//return 94;
	byte iterations=9;       		//check the level this number of times and use whatever value comes up three times in a row first
	int sumrange=0;
	int ranges[iterations];
 	int actualDivisor;
	int runningaverage=0;
	bool disqualifyThisRead=false;
	unsigned long ourtime;
	int i;
	
	
	Serial.print(str_fuel);
	Serial.print(str_space);
	Serial.print(str_level);
	Serial.print(str_space);
	Serial.print(str_checked);
	Serial.println("");
		
	//connect the long i2c bus, normally disconnected because of noise
	//(there is a quad bilateral switch attached to the master's pin #13 (Arduino #7)
	digitalWrite(7, HIGH);									
	for(i=1; i<=iterations; i++)
	{
		//ranges[i]=78;
		ranges[i]=slave_communication(0, SLAVE_RANGEFINDER,8,""); 
		Serial.println(ranges[i]);
       	delay(44);        
		//return ranges[i];
	}
	//disconnect the long i2c bus, normally disconnected because of noise
	//(there is a quad bilateral switch attached to the master's pin #13 (Arduino #7)
	digitalWrite(7, LOW); 
	actualDivisor=0;
	for(i=1; i<=iterations; i++)
	{
		
		if(ranges[i]<1  || ranges[i]>1000 )
		{
			//actualDivisor--;
		}
		else
		{
			actualDivisor++;
			sumrange+=ranges[i];
			runningaverage=sumrange/actualDivisor;
			//if this reading was above zero but wildly divergent from average, then the whole reading is kaput
			if(ranges[i]>0  && (10*runningaverage/ranges[i]>11  ||  (10*runningaverage)/ranges[i]<9) )
			{
				
				disqualifyThisRead=true;
			 
				Serial.print("Fuel reading disqualified: ");
				Serial.print(ranges[i]);
				Serial.print(", running average: ");
				Serial.print(runningaverage);
				Serial.println("");
			}
		}
		/*
		Serial.print(actualDivisor);
		Serial.print(" ");
		Serial.print(ranges[i]);
		Serial.print(" ");
	 	Serial.print(sumrange);
		Serial.println(" ");
		*/
	}
 	if(actualDivisor>iterations/2 && !disqualifyThisRead)
	{
	
		invalidFuelLevelRead=false;
	}
	/*
	Serial.println("++++++" );
	Serial.println(sumrange );
	Serial.println(actualDivisor );
	*/
	return runningaverage; //  return an average
}


// This function gets a ranging from the SRF08, in this case for oil level
//NO LONGER USED
int xgetDistanceToFuel()
{   
	//return 94;
	byte iterations=12;       		//check the level this number of times and use whatever value comes up three times in a row first
	int ranges[iterations];
	int lastrange, secondtolastrange;
	unsigned long ourtime;
	int i;
	//connect the long i2c bus, normally disconnected because of noise
	//(there is a quad bilateral switch attached to the master's pin #13 (Arduino #7)
	digitalWrite(7, HIGH);									
	for(i=0; i<=iterations; i++)
	{
		ourtime=millis();
		Wire.beginTransmission(RANGER_ADDRESS);             // Start communticating with SRF08
		Wire.write(cmdByte);                             	// Send Command Byte
		Wire.write(0x51);                                	// Send 0x51 to start a ranging in centimeters
		Wire.endTransmission();
		delay(80);                                     		// Wait for ranging to be complete
		Wire.beginTransmission(RANGER_ADDRESS);             // start communicating with SRFmodule
		Wire.write(rangeByte);                           	// Call the register for start of ranging data
		Wire.endTransmission();
		Wire.requestFrom(RANGER_ADDRESS, 2);                // Request 2 bytes from SRF module
		while(Wire.available() < 2 && millis()-ourtime<50); // Wait for data to arrive, but not more than 50 milliseconds
		highByte = Wire.read();                      	// Get high byte
		lowByte = Wire.read();                       	// Get low byte
		ranges[i] = (highByte << 8) + lowByte;              // Put them together
	}
	//disconnect the long i2c bus, normally disconnected because of noise
	//(there is a quad bilateral switch attached to the master's pin #13 (Arduino #7)
	digitalWrite(7, LOW); 
	for(i=0; i<=iterations; i++)
	{
		if(ranges[i]==lastrange  && secondtolastrange==lastrange  &&  lastrange!=0)
		{
			return ranges[i];
		}
		secondtolastrange=lastrange;
		lastrange=ranges[i];
	}
	return (secondtolastrange + lastrange)/2; //else return an average
}

void logevent(byte eventtype, int eventquantity)
{
	//eventtype 0 is a boiler tank refill, quantity is cms from ranging
	//eventtype 1 is a frozen determination, quantity is outdoor temperature
	//eventtype 2 is a boilerlog overflow, quantity is fuel level
	//eventtype 3 is a sufficiency without pumping situation, quantity is outdoor temperature
	//eventtype 4 is a controller reboot, quantity is outdoor temperature
	//eventtype 5 is a season to summer change, quantity is outdoor temperature
	//eventtype 6 is a season to winter change, quantity is outdoor temperature
	//eventtype 7 reflects the logging of the last time things were working if it comes up from a crash, quantity is time we were down in seconds (not deciminutes)
	//byte totalboilerontimefortanklocation=4;  //inside RTC ram
	//unsigned long eventlogcursor=0; //place in high EEPROM where we log events
	//Serial.println(eventlogcursor);
	storelong(eventlogcursor,minutecounter, 4);
	storebyte(eventlogcursor+4, eventtype,4);
	storeint(eventlogcursor+5,eventquantity, 4);
	if(eventlogcursor<maxbigcursor)
	{
		eventlogcursor=eventlogcursor+7;
		storelong(eventlogcursorlocation, eventlogcursor, 2);
	}
	else
	{
		eventlogcursor=0;
		storelong(eventlogcursorlocation, 0, 2);
		//logevent(?, ?); it would be pointless to log a wrap around event of the log itself
	}
}

void printfuelinfo()
{
	Serial.print(str_fuel);
	Serial.print(str_space);
	Serial.print(str_level);
	Serial.print(str_colon);
	Serial.print(str_space);
	Serial.print(fuellevel);
	Serial.print(str_space);
	Serial.print(str_old);
	Serial.print(str_space);
	Serial.print(str_fuel);
	Serial.print(str_space);
	Serial.print(str_level);
	Serial.print(str_colon);
	Serial.print(oldfuellevel);
	Serial.print(str_space);
	Serial.print(str_time);
	Serial.print(str_space);
	Serial.print(str_of);
	Serial.print(str_space);
	Serial.print(str_old);
	Serial.print(str_space);
	Serial.print(str_fuel);
	Serial.print(str_space);
	Serial.print(str_level);
	
	
	Serial.print(str_colon);
	deciminutestousefultimeparts((unsigned long)timeoflasttanklevelsnapshot, 0);
	Serial.print(str_space);
	Serial.print(str_second);
	Serial.print(str_s);
	Serial.print(str_space);
	Serial.print(str_fired);
 	Serial.print(str_for);
	Serial.print(str_tank);
	Serial.print(str_colon);
	Serial.print(str_space);
	Serial.print(totalboilerontimefortank * 6);
	Serial.print(str_space);
	Serial.print(str_time);
	Serial.print(str_space);
	Serial.print(str_of);
	Serial.print(" last ");
	Serial.print(str_fired);
	Serial.print(str_colon);
	Serial.print(str_space);
	deciminutestousefultimeparts((unsigned long)getlong(boilertimecutonlocation, 2), 0);
	Serial.println("");
}

void RTCVars()
{
	Serial.print("boilerlogcursor (");
	Serial.print((int)boilerlogcursorlocation);
	Serial.print(") : ");
	Serial.print(getlong(boilerlogcursorlocation,2));
	Serial.println("");
	Serial.print("totalboilerontimefortank (");
	Serial.print((int)totalboilerontimefortanklocation);
	Serial.print(") : ");
	Serial.print(getlong(totalboilerontimefortanklocation,2));
	Serial.println("");
	Serial.print("eventlogcursor (");
	Serial.print((int)eventlogcursorlocation);
	Serial.print(") : ");
	Serial.print(getlong(eventlogcursorlocation,2));
	Serial.println("");
	Serial.print("boilertimecuton (");
	Serial.print((int)boilertimecutonlocation);
	Serial.print(") : ");
	deciminutestousefultimeparts(getlong(boilertimecutonlocation,2),0) ;
	Serial.println("");
	Serial.print("timeatswitchchange (");
	Serial.print((int)timeatswitchchangelocation);
	
	Serial.print(") : ");
	//Serial.print(getlong(timeatswitchchangelocation,2));
	deciminutestousefultimeparts(getlong(timeatswitchchangelocation,2),0) ;
	Serial.println("");
	Serial.print(str_hwater);
	Serial.print(str_max);
	Serial.print(str_spaceparen);
	Serial.print((int)hotwatermaxbyte);
	Serial.print(") : ");
	Serial.print((int)getbyte(hotwatermaxbyte,2));
	Serial.println("");
	Serial.print("loopdelay (");
	Serial.print((int)loopdelaybyte);
	Serial.print(") : ");
	Serial.print((int)getbyte(loopdelaybyte,2));
	Serial.println("");
	Serial.print("mintemptocirculatesummer (");
	Serial.print((int)mintemptocirculatesummerbyte);
	Serial.print(") : ");
	Serial.print((int)getbyte(mintemptocirculatesummerbyte,2));
	Serial.println("");
	Serial.print(str_season);
	Serial.print(str_spaceparen);
	Serial.print((int)seasonbyte);
	Serial.print(") : ");
	Serial.print((int)getbyte(seasonbyte,2));
	Serial.println("");
	Serial.print(str_solar);
	Serial.print(str_log);
	Serial.print(str_cursor);
	Serial.print(str_spaceparen);
	Serial.print((int)solarlogcursorbyte);
	Serial.print(") : ");
	Serial.print((int)getbyte(solarlogcursorbyte,2));
	Serial.println("");
	Serial.print(str_max);
	Serial.print("timetoswitch (");
	Serial.print((int)maxtimetoswitchbyte);
	Serial.print(") : ");
	Serial.print((int)getbyte(maxtimetoswitchbyte,2));
	Serial.println("");
	Serial.print("mintemptocirculatewinter (");
	Serial.print((int)mintemptocirculatewinterbyte);
	Serial.print(") : ");
	Serial.print((int)getbyte(mintemptocirculatewinterbyte,2));
	Serial.println("");
	Serial.print(str_boiler);
	Serial.print(str_counter);
	Serial.print(str_spaceparen);
	Serial.print((int)boileroncounterbyte);
	Serial.print(") : ");
	Serial.print((int)getbyte(boileroncounterbyte,2));
	Serial.println("");
	Serial.print("weektimedeltainseconds (");
	Serial.print((int)weektimedeltainsecondsbyte);
	Serial.print(") : ");
	Serial.print((int)getint(weektimedeltainsecondsbyte,2));
	Serial.println("");
	Serial.print("lasttimecompensationadded (");
	Serial.print((int)lasttimecompensationaddedlocation);
	Serial.print(") : ");
	//Serial.print((int)getint(lasttimecompensationaddedlocation,2));
	deciminutestousefultimeparts(getlong(lasttimecompensationaddedlocation,2),0) ;
	Serial.println("");
	
}

 
void SetSeason(byte seasonin)
{
	//assumes season, seasonbyte is a global
	if(millis()>7000)
	{
		digitalWrite(13,seasonin); ////was HIGH but changed to keep hot water from being overheated during a controller failure
	}
	if(seasonin!=season)
	{
		if(seasonin==1)
		{
			logevent(5, outsidetemp);//eventtype 5 is a season to summer change, quantity is outdoor temperature
		}
		else if(seasonin==0)
		{
			logevent(6, outsidetemp);//eventtype 6 is a season to winter change, quantity is outdoor temperature
		}
	}
	season=seasonin;
	storebyte(seasonbyte, seasonin,2);
}

void displayhelp()
{
 
}
