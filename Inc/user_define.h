/** ȭ�� ������ LED������, ���� �������� 100������ ���� ����, OP_AMP ���� ON, TX ��ȣ ������ Ʈ�������� ON)**/
#define TX_ON GPIOA-> ODR |= GPIO_PIN_10 //PA10---->1 
#define TX_OFF GPIOA-> ODR &= ~GPIO_PIN_10 //PA10---->0