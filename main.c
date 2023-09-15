#include "../include/mdv.h"

/* Core Peripherals */
NVIC_t *const NVICC = (NVIC_t *)0xE000E100UL;
STK_t *const STK = (STK_t *)0xE000E010UL;

/* Peripherals */
FLASH_t *const FLASH = (FLASH_t *)0x40022000UL;
RCC_t *const RCC = (RCC_t *)0x40021000UL;
GPIO_t *const GPIOA = (GPIO_t *)0x48000000UL;
TIMgp_t *const TIM2 = (TIMgp_t *)0x40000000UL;
USART_t *const USART2 = (USART_t *)0x40004400UL;
ADC_t *const ADC1 = (ADC_t *)0x50000000UL;
ADCCommon_t *const ADC12_COMMON
    = (ADCCommon_t *)0x50000300UL;

DMA_t *const DMA1 = (DMA_t *)0x40020000UL;
DMA_t *const DMA2 = (DMA_t *)0x40020400UL;

static uint8_t dma_buffer[100];
static uint8_t main_buffer[100];

fsm_t fsmachine = {
  .buf.dmabuf_ptr = dma_buffer,
  .buf.mainbuf_ptr = main_buffer,
  .data.counter = 0,
};

fsm_t *const fsm = &fsmachine;

__STATIC_FORCEINLINE void
delay_ms (uint32_t ms, uint32_t sysclk_hz)
{
  for (size_t i = 0; i < ms; i++)
    {
      for (size_t j = 0; j < (sysclk_hz / 8000); j++)
        __NOP ();
    }
}

__STATIC_FORCEINLINE void
init_gpioa (GPIO_t *gpio)
{
  gpio->MODER.MODER10 = 1;
  gpio->MODER.MODER9 = 1;
  gpio->MODER.MODER8 = 1;

  // PA2 and PA3 alternate function mode for USART2
  gpio->MODER.MODER2 = 2;
  gpio->MODER.MODER3 = 2;

  gpio->OSPEEDR.OSPEEDR1 = 1;
  gpio->OSPEEDR.OSPEEDR2 = 1;
  gpio->OSPEEDR.OSPEEDR3 = 1;

  gpio->AFRL.AFR2 = 0b111;
  gpio->AFRL.AFR3 = 0b111;
}

__STATIC_FORCEINLINE void
init_usart2 (USART_t *usart)
{
  // 8n1 - 8 data, no parity, 1 stop
  usart->CR1.M0 = 0;
  usart->CR1.M1 = 0;
  usart->CR1.PCE = 0;
  usart->CR2.STOP = 0;

  // Oversample by 16
  usart->CR1.OVER8 = 0;

  // Baud-rate: 2MBps at f_clk = 72MHz 0x24UL
  // Baud-rate: 115200 at f_clk = 72MHz 0x271UL
  usart->BRR.BRR = 0x24UL;
  usart->RTOR.RTO = 2000;
  usart->CR2.RTOEN = 1;

  // Enable USART IDLE line interrupt
  usart->CR1.RXNEIE = 1;
  usart->CR1.RTOIE = 1;

  usart->CR3.DMAR = 1;

  usart->CR1.UE = 1;
  usart->CR1.TE = 1;
  usart->CR1.RE = 1;
}
/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

__STATIC_FORCEINLINE void                                        //__STATIC_FORCEINLINE:Makros; Definiert als eine Statische Inline-Funktion
init_dma1 (DMA_t *dma, USART_t *usart,                           //!!! Initialisierung von DMA1, Konfiguration von DMA1-Kanal 6 fuer den Empfang von Daten von USART2
           void *const rx_buffer)
{
  // DMA1_Channel6 - USART2_RX

  /*P-281 DMA channel x memory address register (DMA_CMARx)
    P-280 DMA channel x peripheral address register (DMA_CPARx)*/
  dma->CMAR6.MA = (uintptr_t)rx_buffer;                         //Einstellen DMA channel_6 memory address register, das zum Speichern der von USART empfangenen Daten verwendet wird. Er zeigt auf den Wert, auf den die Adresse in rx_buffer zeigt.
  dma->CPAR6.PA = (uintptr_t)&usart->RDR.RDR;                   //Einstellen DMA channel_6 peripheral address register. Das ist die Adresse von USART-Empfangsdatenregisters

  //P-280 DMA channel x number of data register (DMA_CNDTRx)
  dma->CNDTR6.NDT = 100;                                         //DMA channel 6 number of data register, das die Anzahl von zu uebertragenden Daten als 100 definiert.

  //P-278 DMA channel x configuration register (DMA_CCRx).
  dma->CCR6.PL = 3;                                              //Setzen DMA Channel_6 priority level auf hoch. [10] High
  dma->CCR6.DIR = 0;                                             //Data transfer direction: [0] Read from peripheral

  dma->CCR6.PSIZE = 0;                                           //Peripheral size: [00] 8-bits. Datenbreite für Peripherie.
  dma->CCR6.MSIZE = 0;                                           //Memory size: [00] 8-bits. Datenbreite für Speicher.

  dma->CCR6.MINC = 1;                                            //Memory increment mode: [1] Memory increment mode enabled. !!!Aktivieren DMA Speicher-Inkrementmodus. Dadurch wird nach jeder Übertragung die Speicheradresse erhöht
  dma->CCR6.PINC = 0;                                            //Peripheral increment mode: [0] Peripheral increment mode disabled. !!!Deaktivieren DMA Peripherie-Inkrementmodus, da wir immer dieselbe Peripherieadresse lesen (USART-Empfangsdatenregister)
  dma->CCR6.CIRC = 1;                                            //Circular mode: [1] Circular mode enabled. !!!Aktivieren DMA Zirkularmodus, der eine automatische Wiederholung der Übertragung ermöglicht, wenn alle Daten übertragen wurden.

  dma->CCR6.TCIE = 1;                                            //Transfer complete interrupt enable: [1] TC interrupt enabled. !!!Aktivieren DMA Übertragungsabschluss-Interrupts. Ein Interrupt wird ausgelöst, wenn alle Daten übertragen wurden.
  dma->CCR6.HTIE = 0;                                            //Half transfer interrupt enable: [0] HT interrupt disabled. !!!Deaktivieren DMA Halbtransfer-Interrupts, da wir in diesem Beispiel keinen Interrupt bei Halbtransfer benötigen.
  dma->CCR6.TEIE = 1;                                            //Transfer error interrupt enable: [1] TE interrupt enabled. !!!Aktivieren DMA Übertragungsfehler-Interrupts. Ein Interrupt wird ausgelöst, wenn während der Datenübertragung ein Fehler auftritt.
}
/*
"DMA" ist Direct memory access controller (DMA);
Direct memory access (DMA) is used in order to provide high-speed data transfer between
peripherals and memory as well as memory to memory. Data can be quickly moved by DMA
without any CPU actions. This keeps CPU resources free for other operations.

"USART" ist universal synchronous asynchronous receiver transmitter
The universal synchronous asynchronous receiver transmitter (USART) offers a flexible
means of Full-duplex data exchange with external equipment requiring an industry standard
NRZ asynchronous serial data format. The USART offers a very wide range of baud rates
using a programmable baud rate generator.

"rx_buffer" is a pointer to the receive data buffer

RDR: Receive data register (USART_RDR)
*/

__STATIC_FORCEINLINE void
start_dma1 (DMA_t *dma)                                         //!!! Starten DMA1, Beginnen Datenübertragung.
{
  dma->CCR6.EN = 1;                                             //Channel enable: [1] Channel enabled.
}
/*
P-278 DMA channel x configuration register (DMA_CCRx)
*/

__STATIC_FORCEINLINE void
init_adc1 (ADC_t *adc, ADCCommon_t *adc_common)                 //!!! Initialisierung von ADC1
{
  adc->CR.ADEN = 0;                                             //Sicherstellen, dass der ADC nicht aktiviert ist. ADEN: ADC enable control: [0] is disabled (OFF state)

  // Wait until end of ADC operation
  while (adc->CR.ADEN == 1)                                    //Falls aktiviert, deaktivieren
    __NOP ();

  //
  if (adc->CR.ADVREGEN == 2)
    {
      adc->CR.ADVREGEN = 0b00; // intermediate state
      adc->CR.ADVREGEN = 0b01; // enabled state
    }

  delay_ms (1, 72e6);

  // ADC Calibration
  adc->CR.ADEN = 0;
  // Wait until end of ADC operation
  while (adc->CR.ADEN == 1)
    __NOP ();

  adc->CR.ADCALDIF = 0;
  adc->CR.ADCAL = 1;
  while (adc->CR.ADCAL == 1)
    __NOP ();

  delay_ms (1, 72e6);

  // Enable ADC
  adc->CR.ADEN = 1;
  while (adc->ISR.ADRDY != 1)
    __NOP ();
  delay_ms (1, 72e6);

  adc->CFGR.EXTSEL = 0b1011;
  adc->CFGR.EXTEN = 1;

  // Single regular conversion
  adc->CFGR.CONT = 0;
  // ADC resolution 10-bit
  adc->CFGR.RES = 1;

  adc->CFGR.OVRMOD = 1;

  // Regular channel, one conversion at Channel 2
  adc->SQR1.L = 0;
  adc->SQR1.SQ1 = 2;

  adc->IER.EOCIE = 1;
}
/*
"ADC" ist Analog-to-digital converters (ADC)
 ADC1 and ADC2 are tightly coupled and can operate in dual mode (ADC1 is master).
*/

__STATIC_FORCEINLINE void
init_tim2 (TIMgp_t *tim, uint16_t psc, uint32_t arr)          //!!! Initialisierung von TIM2
{
  tim->PSC.PSC = psc;                                           //P-664 TIMx prescaler (TIMx_PSC):Prescaler value. Die Zählfrequenz des Timers entspricht der Systemtaktfrequenz geteilt durch diesen Wert.
  tim->CNT = 0;                                                 //P-663 TIMx counter (TIMx_CNT). Timer-Count-Wert auf null setzen.
  tim->ARR = 0;                                                 //P-664 TIMx auto-reload register (TIMx_ARR).Überlastungswert (automatisches Überlastregister) auf null setzen.
  tim->ARR = arr;                                               // Setzt den Überlastungswert des Timers

  // Clear URS & UDIS.
  //P-647 TIMx control register 1 (TIMx_CR1)
  tim->CR1.URS = 0;                                             //Update request source:[0] Any of the following events generate an update interrupt or DMA request if enabled:1.Counter overflow/underflow; 2.Setting the UG bit; 3. Update generation through the slave mode controller
  tim->CR1.UDIS = 0;                                            //Update disable:[0] UEV enabled. The Update (UEV) event is generated by one of the following events:1.Counter overflow/underflow; 2.Setting the UG bit; 3. Update generation through the slave mode controller


  tim->CR1.DIR = 0;                                             //Direction:[0] Counter used as upcounter. Die Zählrichtung des Timers auf aufwärts zählen setzen
  tim->CR2.MMS = 3;                                             //Master mode selection:[010] Update - The update event is selected as trigger output (TRGO). For instance a master timer can then be used as a prescaler for a slave timer. d.h. bei einem Update-Ereignis (UEV) wird ein Impuls am TRGO ausgegeben

  // Auto-reload preload enable
  tim->CR1.ARPE = 1;                                            //Auto-reload preload enable:[1]TIMx_ARR register is buffered
  // Enable update interrupt
  tim->DIER.UIE = 1;                                            //P-653 TIMx DMA/Interrupt enable register (TIMx_DIER). UIE: Update interrupt enable: [1] Update interrupt enabled.
}
/*
"TIM2" ist General-purpose timers (TIM2/TIM3/TIM4)
The general-purpose timers consist of a 16-bit or 32-bit auto-reload counter driven by a
programmable prescaler.

"uint16_t psc": This is the value of the prescaler. In the STM32 timer, the prescaler is used
to slow down the timer count. The count frequency of the timer is equal to the system clock
frequency divided by the prescaler value.

 "uint32_t arr": This is the value of the auto-reload register. When the timer count value reaches
 this value, the counter is automatically reloaded (back to zero) and counting starts again. At the same
 time, the timer generates an update event which may trigger an interrupt (if the update interrupt is switched on).
*/

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
__STATIC_FORCEINLINE void
start_adc1 (ADC_t *adc)
{
  adc->CR.ADSTART = 1;
}

__STATIC_FORCEINLINE void
start_tim2 (TIMgp_t *tim)
{
  tim->CNT = 0;
  tim->CR1.CEN = 1;
}

__STATIC_FORCEINLINE void
transfer_hw (USART_t *usart, uint16_t val)
{
  adc_data tx;
  tx.hw = val;
  for (size_t i = 0; i < 2; i++)
    {
      USART2->TDR.TDR = tx.b[i];
      while (!USART2->ISR.TC)
        __NOP ();
    }

  while (!USART2->ISR.TC)
    __NOP ();
}

__STATIC_FORCEINLINE void
init_system (FLASH_t *flash, RCC_t *rcc, STK_t *stk)
{
  flash->ACR.LATENCY = 2;
  while (flash->ACR.LATENCY != 2)
    __NOP ();
  flash->ACR.PRFTBE = 1;
  while (flash->ACR.PRFTBS != 1)
    __NOP ();

  rcc->CR.PLLON = 0;
  while (rcc->CR.PLLRDY == 1)
    __NOP ();
  __COMPILER_BARRIER ();

  rcc->CFGR.PLLSRC = 1;
  rcc->CFGR.PLLMUL = 7;
  rcc->CFGR.SW = 2;
  rcc->CFGR.HPRE = 0;
  rcc->CFGR.PPRE1 = 0b100;
  rcc->CFGR.PPRE2 = 0;
  rcc->CFGR2.ADC12PRES = 0b10000;
  rcc->CFGR3.TIM2SW = 1;
  rcc->CFGR3.USART2SW = 1;

  rcc->CR.PLLON = 1;
  while (rcc->CR.PLLRDY == 0)
    __NOP ();

  stk->LOAD.RELOAD = 7200000;
  stk->VAL.CURRENT = 0;
  stk->CTRL.CLKSOURCE = 1;
  stk->CTRL.TICKINT = 1;
  stk->CTRL.ENABLE = 0;

  rcc->APB1ENR.PWREN = 1;
  rcc->APB2ENR.SYSCFGEN = 1;
  rcc->AHBENR.IOPAEN = 1;
  rcc->APB1ENR.USART2EN = 1;
  rcc->AHBENR.DMA1EN = 1;
  rcc->AHBENR.ADC12EN = 1;
  rcc->APB1ENR.TIM2EN = 1;
}

int
main (void)
{
  init_system (FLASH, RCC, STK);
  __COMPILER_BARRIER ();

  init_gpioa (GPIOA);
  init_dma1 (DMA1, USART2, dma_buffer);
  start_dma1 (DMA1);
  init_usart2 (USART2);
  init_adc1 (ADC1, ADC12_COMMON);

  NVICC->ISER.SETENA16 = 1;
  NVICC->ISER.SETENA17 = 1;
  NVICC->ISER.SETENA18 = 1;
  NVICC->ISER.SETENA28 = 1;
  NVICC->ISER.SETENA38 = 1;
  __DSB ();

  LED_YGR (0, 0, 1);

  for (;;)
    {
      __WFI ();
      if (fsm->state.rto_detected)
        {
          fsm->state.rto_detected = false;
          __disable_irq ();
          memcpy (fsm->buf.mainbuf_ptr,
                  fsm->buf.dmabuf_ptr, 100);
          memset (fsm->buf.dmabuf_ptr, 0,
                  sizeof (uint8_t) * 100);
          command_check (fsm);
          LED_YGR (0, 0, 0);
          __enable_irq ();

          if (fsm->state.cmd_valid)
            {
              LED_GREEN = 1;
              __disable_irq ();
              USART2->CR2.RTOEN = 0;
              init_tim2 (TIM2, fsm->cmd.psc, fsm->cmd.arr);
              start_adc1 (ADC1);
              __enable_irq ();
              start_tim2 (TIM2);
              __DSB ();
              break;
            }
        }
    }

  for (;;)
    {
      if (fsm->state.sampling_instant)
        {
          fsm->state.sampling_instant = false;
          transfer_hw (USART2, fsm->data.adc_k);
          fsm->data.counter++;
          if (fsm->data.counter == fsm->cmd.count)
            fsm->state.reset = true;
        }

      if (fsm->state.reset)
        __NVIC_SystemReset ();
    }
}
