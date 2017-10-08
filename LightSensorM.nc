module LightSensorM
{
	uses {
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as T1;
		interface Timer<TMilli> as T2;
		interface Read<uint16_t>;
	}
}

implementation
{
	event void Boot.booted(){
		atomic{
			P2DIR |= (1<<1);
			P2OUT |= (1<<1);
		}	
		call T1.startPeriodic(10);
	}

	event void T1.fired(){
		call Read.read();
	}

	event void T2.fired(){
		call Leds.led0Toggle();
	}

	event void Read.readDone(error_t result, uint16_t data){
		if(result == SUCCESS){
			if(data<1500){
				if(call T2.isRunning() == FALSE)
					call T2.startOneShot(1000);
			}
			else
				call Leds.set(0);
		}
	}
}
