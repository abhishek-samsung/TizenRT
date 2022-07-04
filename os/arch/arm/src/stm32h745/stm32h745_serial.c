/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Name: up_earlyserialinit
 *
 * Description:
 *   Performs the low level USART initialization early in debug so that the
 *   serial console will be available during bootup.  This must be called
 *   before up_serialinit.
 *
 ****************************************************************************/

#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void)
{
}
#endif /* USE_EARLYSERIALINIT */

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes
 *   that up_earlyserialinit was called previously.
 *
 ****************************************************************************/

void up_serialinit(void)
{
}

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
#if CONSOLE_UART > 0
  struct stm32l4_serial_s *priv = g_uart_devs[CONSOLE_UART - 1];
  uint16_t ie;

  //stm32l4serial_disableusartint(priv, &ie);

  /* Check for LF */
  if (ch == '\n') {
      /* Add CR */
      up_lowputc('\r');
  }

  up_lowputc((char)ch);
  //stm32l4serial_restoreusartint(priv, ie);
#endif
  return ch;
}

/****************************************************************************
 * Name: up_getc
 *
 * Description:
 *   Read one byte from the serial console
 *
 * Input Parameters:
 *   none
 *
 * Returned Value:
 *   int value, -1 if error, 0~255 if byte successfully read
 *
 ****************************************************************************/
int up_getc(void)
{
}
#else /* USE_SERIALDRIVER */

/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Output one byte on the serial console
 *
 * Input Parameters:
 *   ch - chatacter to output
 *
 * Returned Value:
 *  sent character
 *
 ****************************************************************************/
int up_putc(int ch)
{
#ifdef HAVE_SERIAL_CONSOLE
#if CONSOLE_UART > 0
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      up_lowputc('\r');
    }

  up_lowputc((char)ch);
#endif
  return ch;
#endif
}

/****************************************************************************
 * Name: up_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 * Input Parameters:
 *   ch - chatacter to output
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
void up_lowputc(char ch)
{
}
#endif /* USE_SERIALDRIVER */
