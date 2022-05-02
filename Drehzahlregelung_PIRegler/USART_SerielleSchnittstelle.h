#ifndef USART_SERIELLESCHNITTSTELLE_H_INCLUDED
#define USART_SERIELLESCHNITTSTELLE_H_INCLUDED

void init_usart(void);
void _puts(char st[]);
unsigned char _getch(void);
void _gets(char st[]);
void serialWrite(char st[]);

#endif // USART_SERIELLESCHNITTSTELLE_H_INCLUDED
