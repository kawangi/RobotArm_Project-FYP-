/*
***********************pin config***********************
  DDRC0      ADC
  DDRC1
  DDRC2
  DDRC3
  DDRC4
  DDRC5      ultrasonic echo5
********************************************************
  DDRD0      Usart RX
  DDRD1      Usart TX
  DDRD2      EXTINT 0
  DDRD3      EXTINT 1
  DDRD4      SS
  DDRD5      ultrasonic echo1
  DDRD6      ultrasonic echo2
  DDRD7      ultrasonic echo3
********************************************************
  DDRB0      ultrasonic trigger pulse
  DDRB1      ultrasonic echo4
  DDRB2      SS
  DDRB3      MOSI
  DDRB4      MISO
  DDRB5      SCK
********************************************************
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <stdio.h>

File myFile;

#define FOSC 16000000 // Clock Speed
#define BAUD 38400
#define MYUBRR FOSC/16/BAUD-1
#define ADC_Start (ADCSRA|=(1 << ADSC))    // start

static const unsigned char b4ByeBye[] = "#1P2490#2P930#3P935#4P1568#5P1260#6P2118#7P1498#8P1509#9P1490#10P0";  //66 char
static const unsigned char ByeBye[8][11] = {  "#2P1045T500", "#2P1179T500", "#2P1319T500",
                                              "#2P1179T500", "#2P1045T500", "#2P1179T500",
                                              "#2P1319T500", "#2P1179T500"
                                           };
static const unsigned char b4Shake[] = "#1P1770#2P811#3P1483#4P1722#5P1416#6P2128#7P1518#8P1400#9P1490#10P0";  //67 char
static const unsigned char Shake[8][11] = { "#1P1900T200", "#1P1800T200", "#1P1700T200",
                                            "#1P1800T200", "#1P1900T200", "#1P1800T200",
                                            "#1P1700T200", "#1P1800T200"
                                          };
static const unsigned char init_state[] = "#1P1659#2P0749#3P1723#4P2291#5P1353#6P2128#7P0990#8P0745#9P1490#10P0";  //68 char

char temprec[90] = ""; //#1P1482\r\n
char *token;
unsigned char mode = 0;
uint8_t reccount = 0;
uint8_t ppl = 0 , m3c = 0;
bool rec = false, objcheck = false, readyy = false, needClear = false, wantShake = false, working = false, play = false;

void delayCTC(int times)      //10us time delay per times
{
  TCCR2A = 0x02;  // ctc mode
  TCCR2B = (1 << CS21); // 8 prescaler
  OCR2A =  19;
  while (times > 0) {
    while ((TIFR2 & ( 1 << OCF2A)) == 0);
    TIFR2 = 0x02;
    times--;
  }
}

void delayms(int times)  // 10us *100 = 1ms
{
  for (int i = 0; i < 100; i++)
    delayCTC(times);
}

void Timer1INT_int_Init() //250 ms per interrupt
{
  TCCR1A = 0x00;
  TCCR1B = 0x0D;//14745600/1024 = 14,400 = 4*16*9*25 => 4*3600
  OCR1A = 0x0E0F; //=> 0x0E0F = 3599
  TIMSK1 = (1 << OCIE1A);
}

void USART_Init(unsigned int ubrr)
{
  /*Set baud rate */
  UBRR0H = (unsigned char)(ubrr >> 8);
  //UBRR0L = (unsigned char)ubrr;
  UBRR0L = (unsigned char)25;
  /*Enable receiver and transmitter */
  UCSR0B = (1 << RXEN0) | (1 << TXEN0); //|(1<<RXCIE0);
  /* Set frame format: 8-bit data, 1stop bit */
  UCSR0C = (0 << USBS0) | (3 << UCSZ00);
}

void ADC_init()
{
  ADCSRA  |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //  prescaler = 128
  ADMUX |= (1 << REFS1) | (1 << REFS0) | (1 << ADLAR);
  //  Internal 1.1V Voltage Reference with external capacitor at AREF pin & left adjust
  ADMUX &= ~(( 1 << MUX3) | ( 1 << MUX2) | ( 1 << MUX1) | ( 1 << MUX0)); //MUX3..0 = 0000  =>  keep read ADC0
  ADCSRA  |=  (1 << ADEN) | (1 << ADIE) ; // enable ADC
  DIDR0 |= 0x1E; // 0b0011 1110 => disable ADC 5,4,3,2,1
  DIDR0 &= ~(1 << ADC0D );
  DDRC &= ~(1 << PORTC0); //data in
  DDRC &= ~(1 << PORTC5);

}

void ExtINTinit()  // open port for exint1(PORTD3) & exint0(PORTD2)
{
  DDRD &= ~((1 << PORTD2) | (1 << PORTD3)); //set ext int port no to 0 to enable data in
  PORTD |= (1 << PORTD2) | (1 << PORTD3);
  EICRA |= (1 << ISC01) | (1 << ISC11); // INT0 and INT1, falling edge
  EIMSK |= (1 << INT0) | (1 << INT1);
}

char usart_rece() {
  while (! (UCSR0A & (1 << RXC0)));     // wait until UDR is not empty
  return UDR0;
}

void usart_send (unsigned char data)
{
  while (!(UCSR0A & ( 1 << UDRE0 )));  /* Wait for empty transmit buffer */
  UDR0 = data ;              /* Put data into buffer, sends the data */
}

void newline()
{
  usart_send('\r');
  usart_send('\n');
}

void action(char move[])
{
  for (int i = 0; i < strlen(move); i++)
    usart_send(move[i]);
}

void stop()
{
  usart_send('#'); usart_send('S'); usart_send('T'); usart_send('O'); usart_send('P');
  newline();
}

void ULK()
{
  unsigned char wordulk[4] = "PULK";
  for (int j = 1; j <= 8; j++)
  {
    usart_send('#');
    usart_send(j + '0');
    for (int k = 0; k < 4; k++ )
      usart_send(wordulk[k]);
    newline();
  }
}

void clrbuf()
{
  for (int i = 0; i < 90 ; i++)
    temprec[i] = "";
  reccount = 0;
}

void returnarm()
{
  unsigned char clr;
  if (Serial.available() > 0)
    for (int i = 0; i < Serial.available(); i++)
      clr = Serial.read();
  clrbuf();
  unsigned char wordread[4] = "PRAD";
  for (int j = 1; j <= 8; j++)
  {
    usart_send('#');
    usart_send(j + '0');
    for (int k = 0; k < 4; k++ )
      usart_send(wordread[k]);
    newline();
    while (Serial.available() == 0);
    delayms(10);
    while (Serial.available() > 0)
    {
      temprec[reccount] = Serial.read();
      if (temprec[reccount] == '#')
      {
        Serial.read();
        Serial.read();
        reccount++;
      } else if (temprec[reccount] != ' ')
        reccount++;
    }
    reccount -=  2;
  }
  for (int i = 0; i < reccount; i++)
    myFile.print(temprec[i]);
  myFile.print('\r'); myFile.print('\n');
}

void init_pos()
{
  for (int i = 0; i < 28 ; i++)
    usart_send(init_state[i]);
  action("#5P1034");
  action("T1000");
  newline();
  delayms(500);
  for (int i = 28; i < 68 ; i++)
    usart_send(init_state[i]);
  action("T1000");
  newline();
  delayms(1000);
}

void record(unsigned char filename[]) // total time in s and the period in x00 ms
{
  UCSR0B  &= ~(1 << RXEN0);
  UCSR0B |= (1 << RXEN0);
  myFile = SD.open(filename, FILE_WRITE);
  for (int i = 0; i < 10; i++ )
  {
    if (myFile)
      returnarm();
    delayms(900);
  }
  myFile.close();
}

void replay(unsigned char filename[])
{
  bool pos = false;
  myFile = SD.open(filename);
  if (myFile)
  {
    // read from the file until there's nothing else in it:
    while (myFile.available())
    {
      unsigned char temp = myFile.read();
      if (temp == '\r')
      {
        temp = myFile.read();
        if (!(pos))
        {
          action("T1500");
          newline();
          delayms(2000);
          pos = true;
        } else
        {
          action("T500");
          newline();
          delayms(490);
        }
      } else
        usart_send(temp);
    }
    // close the file:
    myFile.close();
  } else
  {
    // if the file didn't open, print an error:
    // Serial.println("error opening test.txt");
  }

}
/*
  void replay()
  {
  //char temp1[1000]="";
  //strcpy(temp,temprec);
  token = strtok(temprec, linefeed);
  bool pos = false;
  // walk through other tokens
  char *temp = (char*)malloc(sizeof(char*));
  while( token != NULL ) {
    strcpy(temp,token+5);
    if (!(pos))
    {
      for(int i =0;i<strlen(temp);i++)
        usart_send(temp[i]);
      usart_send('T');usart_send('1');usart_send('5');usart_send('0');usart_send('0');
      newline();
      delayms(1500);
      pos = true;
    }
    for(int i =0;i<strlen(temp);i++)
      usart_send(temp[i]);
    usart_send('T');usart_send('5');usart_send('0');usart_send('0');
    newline();
    delayms(500); //0.5s
    token = strtok(NULL, linefeed);
  }
  free(temp);
  free(token);
  UCSR0B  &= ~(1<<RXEN0);
  UCSR0B |= (1<<RXEN0);
  pos = false;
  }
*/

int main(void)
{
  DDRB  = 0;
  DDRD  = 0;
  DDRC  = 0;
  delayms(500);
  //USART_Init(MYUBRR);
  Serial.begin(38400);
  Timer1INT_int_Init();
  //returnarm();
  ADC_init();
  ExtINTinit();
  //ADC_Start;
  delayms(2000);
  ULK();
  delayms(1000);
  if (!SD.begin(4)) {
    //Serial.println("initialization failed!");
    return;
  }
  if (SD.exists("record.txt"))
    SD.remove("record.txt");
  //action("initialization Done...");
  delayms(50);
  //action("Delay Done...");
  init_pos();
  sei();      //enable interrupt
  //action("interrupt enable Done...");
  while (1)
  {
    delayms(10);
    switch (mode)
    {
      case 1 :      //action bye
        if (!(readyy))
        {
          delayms(500);
          for (int i = 0 ; i < 66 ; i++)
            usart_send(b4ByeBye[i]);
          action("T2000");
          newline();
          delayms(1900);
          readyy = true;
          needClear = true;
        } else if (readyy)
        {
          for (int i = 0; i < 8 ; i++)
          {
            for (int j = 0; j < 11 ; j++)
              usart_send(ByeBye[i][j]);
            newline();
            delayms(500);
          }
        }
        break;
      case 2 :        //action shake
        if (needClear)
        {
          readyy = false;
          needClear = false;
          delayms(500);
        }
        if (!(readyy))
        {
          delayms(500);
          for (int i = 0 ; i < 67 ; i++)
            usart_send(b4Shake[i]);
          action("T2000");
          newline();
          delayms(1900);
          readyy = true;
        } else if ((readyy) && (wantShake))
        {
          for (int i = 0; i < 8 ; i++)
          {
            for (int j = 0; j < 11 ; j++)
              usart_send(Shake[i][j]);
            newline();
            delayms(200);
            wantShake = false;
          }
        }
        break;
      case 3:
        delayms(1000);
        if (rec)
        {
          ULK();
          delayms(1000);
          record("record.txt");
          rec = false;
        }
        else if (play)
        {
          replay("record.txt");
          reccount = 0;
          play = false;
        }
        break;
      case 4:
        for (int i = 0 ; i < 66 ; i++)
          usart_send(b4ByeBye[i]);
        action("T1500");
        newline();
        delayms(1400);
        for (int i = 0; i < 8 ; i++)
        {
          for (int j = 0; j < 11 ; j++)
            usart_send(ByeBye[i][j]);
          newline();
          delayms(500);
        }
        delayms(2000);
        init_pos();
        mode = 0;
        ppl = 0;
        m3c = 0;
        working = false;
        readyy = false;
        needClear = false;
        wantShake = false;
      default :
        mode = 0;
        break;
    }
  }
}


ISR(INT0_vect)
{
  play = true;
}

ISR(INT1_vect)
{
  mode = 3;
  rec = true;
  readyy = false;
}

ISR(ADC_vect) {

  int RADC = 0;
  int R3 = 0;
  char result2[10];
  RADC |= ADCL >> 6 | ADCH << 2;
  itoa(RADC, result2, 10);
  R3 = atoi(result2);
  if (R3 < 500)
    wantShake = true;
  else
    wantShake = false;
  /*for (int i = 0; i < 4 ;i++ )
    usart_send(result2[i]);
    newline();*/
}

ISR(TIMER1_COMPA_vect) //250ms
{
  int count1 = 0 , count2 = 0 , count3 = 0 , count4 = 0 , count5 = 0;
  PORTB |= (1 << PORTB0);
  delayCTC(1);
  PORTB &= ~(1 << PORTB0);
  while (!((PIND & (1 << PORTD5)) | (PIND & (1 << PORTD6)) | (PIND & (1 << PORTD7)) | (PINB & (1 << PORTB1)) | (PINC & (1 << PORTC5))));
  while ((PIND & (1 << PORTD5)) | (PIND & (1 << PORTD6)) | (PIND & (1 << PORTD7)) | (PINB & (1 << PORTB1)) | (PINC & (1 << PORTC5)))
  {
    if ((PIND & (1 << 5)) && (count1 < 2200))
      count1++;
    if ((PIND & (1 << 6)) && (count2 < 2200))
      count2++;
    if ((PIND & (1 << 7)) && (count3 < 2200))
      count3++;
    if ((PINB & (1 << 1)) && (count4 < 2200))
      count4++;
    if ((PINC & (1 << 5)) && (count5 < 2200))
      count5++;
    delayCTC(1);//10us
  }
  if (count1 == 0) count1 = 2200;
  if (count2 == 0) count2 = 2200;
  if (count3 == 0) count3 = 2200;
  if (count4 == 0) count4 = 2200;
  if (count5 == 0) count5 = 2200;
  if (mode != 3)
  {
    if ((!(objcheck)) && (mode == 2))
    {
      ADCSRA |= (1 << ADSC);    // start
    }

    if ((!(working)) && ((count1 < 2000) | (count2 < 2000) | (count3 < 2000) | (count4 < 2000) | (count5 < 2000)))
    {
      if (ppl++ > 11)
      {
        working = true;
        mode = 1;
        readyy = false;
      }
    } else if ((working) && ((count1 < 800) | (count2 < 800) | (count3 < 800) | (count4 < 800) | (count5 < 800)))
    {
      if (needClear)
      {
        mode = 2;
        readyy = false;
      }
    } else if ((working) && ((count1 > 2200) | (count2 > 2200) | (count3 > 2200) | (count4 > 2200) | (count5 > 2200)))
    {
      m3c++;
      if (m3c >= 12)
        mode = 4;
    }

    //      Serial.print("Ultrasound 1 = "); Serial.print((int)count1 * 1.7); Serial.print(" mm   ");
    //      Serial.print("Ultrasound 2 = "); Serial.print((int)count2 * 1.7); Serial.print(" mm   ");
    //      Serial.print("Ultrasound 3 = "); Serial.print((int)count3 * 1.7); Serial.print(" mm   ");
    //      Serial.print("Ultrasound 4 = "); Serial.print((int)count4 * 1.7); Serial.print(" mm   ");
    //      Serial.print("Ultrasound 5 = "); Serial.print((int)count5 * 1.7); Serial.println(" mm   ");

  } else
  {
    if ((count1 >= 2100) & (count2 >= 2100) & (count3 >= 2100) & (count4 >= 2100) & (count5 >= 2100))
    {
      m3c++;
      if (m3c >= 12)
        mode = 4;
    }
    else m3c = 0;
  }
}

