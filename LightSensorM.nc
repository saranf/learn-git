module LightSensorM
{
	uses {
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as T1;
		interface Timer<TMilli> as T2;
		interface Timer<TMilli> as T3;
		interface Read<uint16_t>;
	}
}

implementation
{
	event void Boot.booted(){
		call Leds.set(0);
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

	event void T3.fired(){
		call Leds.set(0);
	}


	event void Read.readDone(error_t result, uint16_t data){
		if(result == SUCCESS){
			if(data<2000)
			{
				if(call T2.isRunning() == FALSE)
					call T2.startOneShot(300);
				else
					call Leds.set(0);
			}
			else
					call T3.startOneShot(10);


						
			if (data<3000)
			{
				if (call T2.isRunning()== FALSE)
					call T2.startOneShot(1000);
				else 
					call Leds.set(0);
			}
			else
					call T3.startOneShot(10);

							
			if (data<4000)
			{
				if (call T2.isRunning()== FALSE)
					call T2.startOneShot(2000);
				else 
					call Leds.set(0);
			}
			else
					call T3.startOneShot(10);

				
		}	
	}
}
