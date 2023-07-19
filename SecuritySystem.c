#include <msp430.h>

#define TRUE 1
#define FALSE 0

// Definição do endereço do PCF_8574
#define PCF_ADR1 0x3F
#define PCF_ADR2 0x27
#define PCF_ADR PCF_ADR2

#define BR_100K 11 // SMCLK/100K = 11
#define BR_50K 21  // SMCLK/50K  = 21
#define BR_10K 105 // SMCLK/10K  = 105

void lcd_inic(void);
void lcd_char(char x);
void lcd_str(char *pt);
void lcd_aux(char dado);
int pcf_read(void);
void pcf_write(char dado);
int pcf_teste(char adr);
void led_vd(void);
void led_VD(void);
void led_vm(void);
void led_VM(void);
void i2c_config(void);
void gpio_config(void);
void sensor_detector();
void lcd_clear(void);
void delay(long limite);
void lcd_show();
void sensor_config();
void lcd_disparado_msg();
void bt_str(char *vet);
void bt_char(char c);
void USCI_A0_config(void);

int main(void)
{

    WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer

    USCI_A0_config();
    gpio_config();
    i2c_config();
    sensor_config();

    if (pcf_teste(PCF_ADR) == FALSE)
    {
        led_VM(); // Indicar que não houve ACK
        while (TRUE)
            ; // Travar
    }
    else
        led_VD(); // Houve ACK, tudo certo

    lcd_inic();   // Inicializar LCD
    pcf_write(8); // Acender Back Ligh

    while (1)
    {
        lcd_str("PRESSIONE P2.1");
        bt_str("\nSISTEMA DE ALARME COM BLUETOOTH\n");
        bt_str("PRESSIONE P2.1 PARA COMEÇAR\n");
        delay(5000);

        while (1)
        {

            if ((P2IN & BIT1) == 0)
            {
                lcd_show();

                sensor_detector();
            }
        }
    }

    return 0;
}

// Incializar LCD modo 4 bits
void lcd_inic(void)
{

    // Preparar I2C para operar
    UCB0I2CSA = PCF_ADR; // Endereço Escravo
    UCB0CTL1 |= UCTR |   // Mestre TX
                UCTXSTT; // Gerar START
    while ((UCB0IFG & UCTXIFG) == 0)
        ;          // Esperar TXIFG=1
    UCB0TXBUF = 0; // Saída PCF = 0;
    while ((UCB0CTL1 & UCTXSTT) == UCTXSTT)
        ;                                   // Esperar STT=0
    if ((UCB0IFG & UCNACKIFG) == UCNACKIFG) // NACK?
        while (1)
            ;

    // Come�ar inicializa��o
    lcd_aux(0); // RS=RW=0, BL=1
    delay(20000);
    lcd_aux(3); // 3
    delay(10000);
    lcd_aux(3); // 3
    delay(10000);
    lcd_aux(3); // 3
    delay(10000);
    lcd_aux(2); // 2

    // Entrou em modo 4 bits
    lcd_aux(2);
    lcd_aux(8); // 0x28
    lcd_aux(0);
    lcd_aux(8); // 0x08
    lcd_aux(0);
    lcd_aux(1); // 0x01
    lcd_aux(0);
    lcd_aux(6); // 0x06
    lcd_aux(0);
    lcd_aux(0xF); // 0x0F

    while ((UCB0IFG & UCTXIFG) == 0)
        ;                // Esperar TXIFG=1
    UCB0CTL1 |= UCTXSTP; // Gerar STOP
    while ((UCB0CTL1 & UCTXSTP) == UCTXSTP)
        ; // Esperar STOP
    delay(50);
}

// Auxiliar inicialização do LCD (RS=RW=0)

void lcd_aux(char dado)
{
    while ((UCB0IFG & UCTXIFG) == 0)
        ;                                    // Esperar TXIFG=1
    UCB0TXBUF = ((dado << 4) & 0XF0) | BIT3; // PCF7:4 = dado;
    delay(50);
    while ((UCB0IFG & UCTXIFG) == 0)
        ;                                           // Esperar TXIFG=1
    UCB0TXBUF = ((dado << 4) & 0XF0) | BIT3 | BIT2; // E=1
    delay(50);
    while ((UCB0IFG & UCTXIFG) == 0)
        ;                                    // Esperar TXIFG=1
    UCB0TXBUF = ((dado << 4) & 0XF0) | BIT3; // E=0;
}

void lcd_clear(void)
{
    // Preparar I2C para operar
    UCB0I2CSA = PCF_ADR; // Endereço Escravo
    UCB0CTL1 |= UCTR |   // Mestre TX
                UCTXSTT; // Gerar START
    while ((UCB0IFG & UCTXIFG) == 0)
        ;          // Esperar TXIFG=1
    UCB0TXBUF = 0; // Sa�da PCF = 0;
    while ((UCB0CTL1 & UCTXSTT) == UCTXSTT)
        ;                                   // Esperar STT=0
    if ((UCB0IFG & UCNACKIFG) == UCNACKIFG) // NACK?
        while (1)
            ;

    // Limpar display
    lcd_aux(0);    // RS=RW=0, BL=1
    lcd_aux(0x01); // Clear display

    while ((UCB0IFG & UCTXIFG) == 0)
        ;                // Esperar TXIFG=1
    UCB0CTL1 |= UCTXSTP; // Gerar STOP
    while ((UCB0CTL1 & UCTXSTP) == UCTXSTP)
        ; // Esperar STOP
    delay(50);
}

// Ler a porta do PCF
int pcf_read(void)
{
    int dado;
    UCB0I2CSA = PCF_ADR; // Endereço Escravo
    UCB0CTL1 &= ~UCTR;   // Mestre RX
    UCB0CTL1 |= UCTXSTT; // Gerar START
    while ((UCB0CTL1 & UCTXSTT) == UCTXSTT)
        ;
    UCB0CTL1 |= UCTXSTP; // Gerar STOP + NACK
    while ((UCB0CTL1 & UCTXSTP) == UCTXSTP)
        ; // Esperar STOP
    while ((UCB0IFG & UCRXIFG) == 0)
        ; // Esperar RX
    dado = UCB0RXBUF;
    return dado;
}

// Escrever dado na porta
void pcf_write(char dado)
{
    UCB0I2CSA = PCF_ADR; // Endereço Escravo
    UCB0CTL1 |= UCTR |   // Mestre TX
                UCTXSTT; // Gerar START
    while ((UCB0IFG & UCTXIFG) == 0)
        ;             // Esperar TXIFG=1
    UCB0TXBUF = dado; // Escrever dado
    while ((UCB0CTL1 & UCTXSTT) == UCTXSTT)
        ;                                   // Esperar STT=0
    if ((UCB0IFG & UCNACKIFG) == UCNACKIFG) // NACK?
        while (1)
            ;            // Escravo gerou NACK
    UCB0CTL1 |= UCTXSTP; // Gerar STOP
    while ((UCB0CTL1 & UCTXSTP) == UCTXSTP)
        ; // Esperar STOP
}

// Testar endereço I2C
// TRUE se recebeu ACK
int pcf_teste(char adr)
{
    UCB0I2CSA = adr;            // Endereço do PCF
    UCB0CTL1 |= UCTR | UCTXSTT; // Gerar START, Mestre transmissor
    while ((UCB0IFG & UCTXIFG) == 0)
        ;                // Esperar pelo START
    UCB0CTL1 |= UCTXSTP; // Gerar STOP
    while ((UCB0CTL1 & UCTXSTP) == UCTXSTP)
        ; // Esperar pelo STOP
    if ((UCB0IFG & UCNACKIFG) == 0)
        return TRUE;
    else
        return FALSE;
}

// Configurar UCSB0 e Pinos I2C
// P3.0 = SDA e P3.1=SCL
void i2c_config(void)
{
    UCB0CTL1 |= UCSWRST;  // UCSI B0 em ressete
    UCB0CTL0 = UCSYNC |   // Síncrono
               UCMODE_3 | // Modo I2C
               UCMST;     // Mestre
    UCB0BRW = BR_100K;    // 100 kbps
    P3SEL |= BIT1 | BIT0; // Use dedicated module
    UCB0CTL1 = UCSSEL_2;  // SMCLK e remove ressete
}

void USCI_A0_config(void)
{
    UCA0CTL1 = UCSWRST;
      UCA0CTL0 =0;
      UCA0BRW =6;
      UCA0MCTL = UCBRF_13|UCOS16;
      P3SEL|=BIT4|BIT3;
      UCA0CTL1 = UCSSEL_2;
}



void lcd_char(char x)
{
    char esq, dir;
    esq = x & 0xF0;
    dir = (x & 0xF) << 4;
    pcf_write(esq | 9);
    pcf_write(esq | 0xd);
    pcf_write(esq | 9);
    pcf_write(dir | 9);
    pcf_write(dir | 0xd);
    pcf_write(dir | 9);
}

void lcd_str(char *pt)
{

    char i = 0;
    while (pt[i] != 0)
    {

        lcd_char(pt[i++]);
    }
}

void lcd_show()
{

    delay(5000);
    lcd_clear();
    delay(10000);
    lcd_str("PREPARANDO...");
    delay(100000);
    lcd_clear();
    lcd_str("1...");
    delay(100000);
    lcd_clear();
    lcd_str("2...");
    delay(100000);
    lcd_clear();
    lcd_str("3...");
    delay(100000);
    lcd_clear();
    delay(20000);
    lcd_str("ARMADO!");
    delay(5000);
    bt_str("ALARME PRONTO\n");
}

void lcd_disparado_msg()
{

    P1DIR &= ~BIT1; // Config botão alarme
    P1REN |= BIT1;
    P1OUT |= BIT1;
    USCI_A0_config();

    while ((P1IN & BIT1) != 0)
    {

        lcd_clear();
        lcd_str("ALARME ACIONADO");
        delay(100000);
        lcd_clear();
        lcd_str("PRESSIONE P1.1");
        delay(100000);
        int i = 0;
        while (i < 20)
        {
            bt_str("ALARME ASSIONADO PRESSIONE P1.1\n");
            i++;
        }
    }
    delay(5000);
    lcd_clear();
    lcd_str("DESARMANDO...");
    bt_str("DESARMANDO...");
    delay(100000);
    main();
}

void sensor_config()
{
    P2DIR &= ~BIT2; // Entrada
}
void sensor_detector(){

    sensor_config();

       P1DIR |= BIT0;  // Led vermelho
       P1OUT &= ~BIT0; // Vermelho Apagado
       P4DIR |= BIT7;  // Led verde
       P4OUT &= ~BIT7; // Verde Apagado

       while(1){
           if((P2IN & BIT2) == BIT2){
               P4OUT |= BIT7;
               delay(5000);
               lcd_disparado_msg();
           }
           else{
               P4OUT &= ~BIT7;
           }
       }
}

void bt_str(char *vet)
{
    unsigned int i = 0;
    while (vet[i] != '\0')
        bt_char(vet[i++]);
}

void bt_char(char c)
{
    while ((UCA0IFG & UCTXIFG) == 0)
        ;
    UCA0TXBUF = c;
}

void lcd_dec8(char z)
{ // funcao de escrever numeros por nibble
    char w;
    w = z / 100;
    lcd_char(w | 0x30);
    z = z - 100 * w;
    w = z / 10;
    lcd_char(w | 0x30);
    w = z - 10 * w;
    lcd_char(w | 0x30);
}

void led_vd(void) { P4OUT &= ~BIT7; } // Apagar verde
void led_VD(void) { P4OUT |= BIT7; }  // Acender verde
void led_vm(void) { P1OUT &= ~BIT0; } // Apagar vermelho
void led_VM(void) { P1OUT |= BIT0; }  // Acender vermelho

// Configurar leds
void gpio_config(void)
{
    P1DIR |= BIT0;  // Led vermelho
    P1OUT &= ~BIT0; // Vermelho Apagado
    P4DIR |= BIT7;  // Led verde
    P4OUT &= ~BIT7; // Verde Apagado

    P2DIR &= ~BIT1; // Config botão alarme
    P2REN |= BIT1;
    P2OUT |= BIT1;
}

void delay(long limite)
{
    volatile long cont = 0;
    while (cont++ < limite)
        ;
}
