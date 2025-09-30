/** 화제 감지시 LED켜지고, 저항 가변으로 100옴으로 부하 증가, OP_AMP 전원 ON, TX 신호 보내는 트랜지스터 ON)**/
#define TX_ON GPIOA-> ODR |= GPIO_PIN_10 //PA10---->1 
#define TX_OFF GPIOA-> ODR &= ~GPIO_PIN_10 //PA10---->0