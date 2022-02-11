#define F_CPU 16000000UL 
//define a frequência do microcontrolador - 16MHz
#include <avr/io.h> 
//definições do componente especificado
#include <util/delay.h> 
//biblioteca para o uso das rotinas de _delay_ms e _delay_us()

//Definições de macros para o trabalho com bits
#define set_bit(y,bit) (y|=(1<<bit)) 
//coloca em 1 o bit x da variável Y
#define clr_bit(y,bit) (y&=~(1<<bit))
//coloca em 0 o bit x da variável Y
#define cpl_bit(y,bit) (y^=(1<<bit)) 
//troca o estado lógico do bit x da variável Y
#define tst_bit(y,bit) (y&(1<<bit))
//retorna 0 ou 1 conforme leitura do bit

#define DADOS_LCD PORTD
//4 bits de dados do LCD no PORTD
#define nibble_dados 1 
/* 0 para via de dados do LCD nos 4 LSBs do PORT
empregado (Px0-D2, Px1-D3, Px2-D4, Px3-D5), 1 para via de
dados do LCD nos 4 MSBs do PORT empregado (Px4-D2, Px5-D3,
Px6-D4, Px7-D5) */

#define CONTR_LCD PORTB 
//PORT com os pinos de controle do LCD (pino R/W em 0).
#define E PB3
//pino de habilitação do LCD (enable)
#define RS PB4 
//pino para informar se o dado é uma instrução ou caractere
#define tam_vetor 5 
//número de digitos individuais para a conversão por ident_num()
#define conv_ascii 48 
/*48 se ident_num() deve retornar um número no formato ASCII (0 para
formato normal)*/

//sinal de habilitação para o LCD
#define pulso_enable() _delay_us(1); set_bit(CONTR_LCD,E); _delay_us(1); clr_bit(CONTR_LCD,E); _delay_us(45)

//protótipo das funções
void cmd_LCD(unsigned char c, char cd);
void inic_LCD_4bits(); 
void escreve_LCD(char *c);
void escreve_LCD_Flash(const char *c);
void ident_num(unsigned int valor, unsigned char *disp);

//================================================================================ //
// Sub-rotinas para o trabalho com um LCD 16x2 com via de dados de 4 bits //
// Controlador HD44780 - Pino R/W aterrado //
// A via de dados do LCD deve ser ligado aos 4 bits mais significativos ou //
// aos 4 bits menos significativos do PORT do uC //
//================================================================================ //
//-----------------------------------------------------------------------------------
// Sub-rotina para enviar caracteres e comandos ao LCD com via de dados de 4 bits
//-----------------------------------------------------------------------------------
//c é o dado e cd indica se é instrução ou caractere (0 ou 1)

void cmd_LCD(unsigned char c, char cd)
{
  if(cd==0) //instrução
	  clr_bit(CONTR_LCD,RS);
  else //caractere
  	set_bit(CONTR_LCD,RS);

  //primeiro nibble de dados - 4 MSB
  #if (nibble_dados)
  //compila o código para os pinos de dados do LCD nos 4 MSB do PORT
  DADOS_LCD = (DADOS_LCD & 0x0F)|(0xF0 & c);
  #else 
  //compila o código para os pinos de dados do LCD nos 4 LSB do PORT
  DADOS_LCD = (DADOS_LCD & 0xF0)|(c>>4);
  #endif
  pulso_enable();

  //segundo nibble de dados - 4 LSB
  #if (nibble_dados) 
  //compila o código para os pinos de dados do LCD nos 4 MSB do PORT
  DADOS_LCD = (DADOS_LCD & 0x0F) | (0xF0 & (c<<4));
  #else 
  //compila o código para os pinos de dados do LCD nos 4 LSB do PORT
  DADOS_LCD = (DADOS_LCD & 0xF0) | (0x0F & c);
  #endif
  pulso_enable();
  
  if((cd==0) && (c<4)) //se for instrução de retorno ou limpeza espera LCD estar pronto
  	_delay_ms(2);
}
//-----------------------------------------------------------------------------------
//Sub-rotina para inicialização do LCD com via de dados de 4 bits
//-----------------------------------------------------------------------------------

void inic_LCD_4bits()//sequência ditada pelo fabricando do circuito integrado HD44780
{ //o LCD será só escrito. Então, R/W é sempre zero.
  clr_bit(CONTR_LCD,RS);//RS em zero indicando que o dado para o LCD será uma instrução
  clr_bit(CONTR_LCD,E);//pino de habilitação em zero
  _delay_ms(20); /*tempo para estabilizar a tensão do LCD, após VCC
  ultrapassar 4.5 V (na prática pode ser maior).*/

  //interface de 8 bits
  #if (nibble_dados)
  DADOS_LCD = (DADOS_LCD & 0x0F) | 0x30;
  #else
  DADOS_LCD = (DADOS_LCD & 0xF0) | 0x03;
  #endif
  pulso_enable(); //habilitação respeitando os tempos de resposta do LCD
  _delay_ms(5);
  pulso_enable();
  _delay_us(200);
  pulso_enable(); //até aqui ainda é uma interface de 8 bits.

  //interface de 4 bits, deve ser enviado duas vezes (a outra está abaixo)
  #if (nibble_dados)
  DADOS_LCD = (DADOS_LCD & 0x0F) | 0x20;
  #else
  DADOS_LCD = (DADOS_LCD & 0xF0) | 0x02;
  #endif
  pulso_enable();
  
  cmd_LCD(0x28,0); //interface de 4 bits 2 linhas (aqui se habilita as 2 linhas)
  //são enviados os 2 nibbles (0x2 e 0x8)
  cmd_LCD(0x08,0); //desliga o display
  cmd_LCD(0x01,0); //limpa todo o display
  cmd_LCD(0x0C,0); //mensagem aparente cursor inativo não piscando
  cmd_LCD(0x80,0); //inicializa cursor na primeira posição a esquerda - 1a linha
}
//------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD - dados armazenados na RAM
//------------------------------------------------------------------------------------
void escreve_LCD(char *c)
{
	for (; *c!=0;c++) cmd_LCD(*c,1);
}
//------------------------------------------------------------------------------------
//Sub-rotina de escrita no LCD - dados armazenados na FLASH
//------------------------------------------------------------------------------------
void escreve_LCD_Flash(const char *c)
{
	for (;pgm_read_byte(&(*c))!=0;c++) cmd_LCD(pgm_read_byte(&(*c)),1);
}

//------------------------------------------------------------------------------------
//Conversão de um número em seus digitos individuais – função auxiliar
//-----------------------------------------------------------------------------------
void ident_num(unsigned int valor, unsigned char *disp)
{
  unsigned char n;
  for(n=0; n<tam_vetor; n++)
  	disp[n] = 0 + conv_ascii; //limpa vetor para armazenagem dos digitos

  do
  {
    *disp = (valor%10) + conv_ascii; //pega o resto da divisão por 10
    valor /=10; //pega o inteiro da divisão por 10
    disp++;
  }while (valor!=0);
}

//---------------------------------------------------------------------------
//Configura o conversor A/D do ATmega328
//ref = 0. Utilizar tensão de referência Aref
//ref = 1. Utilizar tensão de referência Avcc
//ref = 2. Utilizar tensão de referência interna de 1,1 V
//did: valor para o registrador DIDR0
//---------------------------------------------------------------------------
void adcBegin(uint8_t ref, uint8_t did)
{ ADCSRA = 0; //configuração inicial
ADCSRB = 0; //configuração inicial
DIDR0 = did; //configura DIDR0
if (ref == 0)
{ ADMUX &= ~((1<<REFS1) | (1<<REFS0));}//Aref
if ((ref == 1) || (ref > 2))
{ ADMUX &= ~(1<<REFS1); //Avcc
ADMUX |= (1<<REFS0); //Avcc
}
if (ref == 2)
{ ADMUX |= (1<<REFS1) | (1<<REFS0);}//Tensão interna de ref (1.1V)
ADMUX &= ~(1<<ADLAR); //Alinhamento a direita
ADCSRA |= (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);//Prescaler = 128
}

//-----------------------------------------------------------
//Seleciona canal do A/D
//0 <= channel <= 5 - Leitura dos pinos AD0 a AD5
//channel = 6 - Leitura do sensor de temperatura
//channel = 7 - 1,1V
//channel > 7 - GND
//-----------------------------------------------------------
void adcChannel(uint8_t channel)
{ if (channel <= 5)//seleciona um canal no multiplex
ADMUX = (ADMUX & 0xF0) | channel;
if (channel == 6)//seleciona sensor interno de temperatura
ADMUX = (ADMUX & 0xF0) | 0x08;
if (channel == 7)//seleciona 1,1 V
ADMUX = (ADMUX & 0xF0) | 0x0E;
if (channel > 7)//seleciona GND
ADMUX = (ADMUX & 0xF0) | 0x0F;
}

//---------------------------------------------------
//Inicia conversão
//---------------------------------------------------
void adcSample(void)
{ ADCSRA |= (1<<ADSC);//Inicia conversão
}
//---------------------------------------------------
//Verifica se conversão foi concluída
//Retorna valor 0 se conversão concluída. 64 se não.
//---------------------------------------------------
uint8_t adcOk(void)
{ return (ADCSRA & (1<<ADSC));
}

//---------------------------------------------------------
//Ler o A/D e retorna o valor lido
//---------------------------------------------------------
uint16_t adcReadOnly()
{ return (ADCL | (ADCH<<8));//retorna o valor do A/D
}
//---------------------------------------------------------
//Converte, aguarda, ler e retorna valor lido do A/D
//---------------------------------------------------------
uint16_t adcRead()
{ adcSample(); //Inicia conversão
while(adcOk()); //Aguarda fim da conversão (ADSC = 0)
return adcReadOnly();//retorna o valor do A/D
}

//------------------------------------------------------
//Habilita ou desabilita interrupção do A/D
//Se x = 0, desabilita interrupção
//Caso contrário, habilita interrupção
//------------------------------------------------------
void adcIntEn(uint8_t x)
{ if (x)
ADCSRA |= (1<<ADIE);//habilita interrupção do A/D
else
ADCSRA &= ~(1<<ADIE);//Desabilita interrupção do A/D
}

#define BAUD 9600

//inicializa a porta de comunicação uart do ATmega328
void uartBegin(uint32_t baud, uint32_t freq_cpu)
{ uint16_t myubrr = freq_cpu/16/baud-1;//calcula valor do registrador UBRR
UBRR0H = (uint8_t)(myubrr >> 8); //ajusta a taxa de transmissão
UBRR0L = (uint8_t)myubrr;
UCSR0A = 0;//desabilitar velocidade dupla (no Arduino é habilitado por padrão)
UCSR0B = (1<<RXEN0)|(1<<TXEN0);//habilita a transmissão e recepção. Sem interrupcao
UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);//assíncrono, 8 bits, 1 bit de parada, sem paridade
}

//verifica se novo dado pode ser enviado pela UART
//retorna valor 32 se novo dado pode ser enviado. Zero se não.
uint8_t uartTxOk (void)
{ return (UCSR0A & (1<<UDRE0));}
//Envia um byte pela porta uart
void uartTx(uint8_t dado)
{ UDR0 = dado;//envia dado
}
//Envia uma string pela porta uart. Ultimo valor da string deve ser 0.
void uartString(char *c)
{ for (; *c!=0; c++)
{ while (!uartTxOk());//aguarda último dado ser enviado
uartTx(*c);
}
}

//verifica se UART possui novo dado
//retorna valor 128 se existir novo dado recebido. Zero se não.
uint8_t uartRxOk (void)
{ return (UCSR0A & (1<<RXC0));
}
//Ler byte recebido na porta uart
uint8_t uartRx()
{ return UDR0; //retorna o dado recebido
}

void uartIntRx(uint8_t x)
{ if (x)
UCSR0B |= (1<<RXCIE0);//Habilita interrupção de recepção de dados
else
UCSR0B &= ~(1<<RXCIE0);//Desabilita interrupção de recepção de dados
}
//Habilita ou desabilita interrupção de Transmissão da usart
//x = 0, desabilita interrupção. Outro valor, habilita interrupção
void uartIntTx(uint8_t x)
{ if (x)
UCSR0B |= (1<<TXCIE0);//Habilita interrupção de recepção de dados
else
UCSR0B &= ~(1<<TXCIE0);//Desabilita interrupção de recepção de dados
}
//Tratadores de interrupções da usart
ISR(USART_RX_vect) {/*USART, recepção de dados completa*/}
ISR(USART_TX_vect) {/*USART, transmissão de dados completa*/}
ISR(USART_UDRE_vect) {/*USART, limpeza do registrador de dados*/}

void uartDec2B(uint16_t valor)
{ int8_t disp;
char digitos[5];
int8_t conta = 0;
do //converte o valor armazenando os algarismos no vetor digitos
{ disp = (valor%10) + 48;//armazena o resto da divisao por 10 e soma com 48
valor /= 10;
digitos[conta]=disp;
conta++;
} while (valor!=0);
for (disp=conta-1; disp>=0; disp-- )//envia valores do vetor digitos
{ while (!uartTxOk()); //aguarda último dado ser enviado
uartTx(digitos[disp]);//envia algarismo
}
}

#define AVCC 1 //Tensão de referência = Avcc
#define ADC0 0 //Seleciona ADC0

#define LED2 PD2
#define LED3 PD3

//-----------------------------------------------------------------------------------

int main(void)
{
  DDRD = 0xFF; // PORTD como saída
  DDRB = 0xFF; // PORTB como saída
  inic_LCD_4bits(); // Inicializa o LCD
  
  uint16_t valorADC;
  uartBegin(9600, F_CPU);//Inicializa UART
  adcBegin(AVCC, 0x01); //Inicializa A/D
  adcChannel(ADC0); //seleciona entrada
  
  while(1) {
    
    if(valorADC > 700) {
      escreve_LCD("Vazamento de Gas"); // String armazenada na RAM
      cmd_LCD(0xC0,0); // Segunda linha, última posição
      escreve_LCD("> 700 (Critico) "); // String armazenada na RAM
      cpl_bit(PORTD,LED2); // Pisca led vermelho
      clr_bit(PORTD,LED3); // Apaga led verde
    }
    
    if(valorADC > 500 && valorADC < 700) {
      escreve_LCD("Vazamento de Gas"); // String armazenada na RAM
      cmd_LCD(0xC0,0); // Segunda linha, última posição
      escreve_LCD("Entre 500 e 700 "); // String armazenada na RAM
      set_bit(PORTD,LED2); // Acende led vermelho
      clr_bit(PORTD,LED3); // Apaga led verde
    }
    
    if(valorADC < 500) {
      cmd_LCD(0x01,0); // limpa
      set_bit(PORTD,LED3); // Acende led verde
      clr_bit(PORTD,LED2); // Apaga led vermelho
    }
    
    valorADC = adcRead();// Lê o valor analógico
    uartDec2B(valorADC); // Envia valor lido para o monitor serial
    uartString("\n"); // Pula para uma nova linha no monitor serial
	_delay_ms(200);
    
  }

}
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  