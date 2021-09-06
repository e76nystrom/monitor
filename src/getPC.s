.global getPC

#ifdef ARDUINO_AVR_MEGA2560
	
getPC:
	clr 25
	pop r24
	pop r23
	pop r22
	push r22
	push r23
	push r25
	ret
	
#endif

#ifdef ARDUINO_AVR_PRO
	
getPC:
	pop r25
	pop r24
	push r24
	push r25
	ret
	
#endif
	
