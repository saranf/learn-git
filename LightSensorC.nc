configuration LightSensorC
{}

implementation
{
	components MainC, LightSensorM, LedsC,
		        new LightC() as Sensor,
		        new TimerMilliC() as Timer1,
		        new TimerMilliC() as Timer2;

	LightSensorM.Boot -> MainC;
	LightSensorM.Leds -> LedsC;
	LightSensorM.Read -> Sensor;
	LightSensorM.T1 -> Timer1;
	LightSensorM.T2 -> Timer2;
}
