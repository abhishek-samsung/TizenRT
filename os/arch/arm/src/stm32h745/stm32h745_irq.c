/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  uint32_t regaddr;
  int num_priority_registers;
  int i;

  /* Disable all interrupts */


#ifdef CONFIG_ARCH_RAMVECTORS
  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

#endif

  /* Set all interrupts (and exceptions) to the default priority */

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(STM32L4_IRQ_SVCALL, up_svcall, NULL);
  irq_attach(STM32L4_IRQ_HARDFAULT, up_hardfault, NULL);

  /* Set the priority of the SVCall interrupt */

#ifdef CONFIG_ARCH_IRQPRIO
  /* up_prioritize_irq(STM32L4_IRQ_PENDSV, NVIC_SYSH_PRIORITY_MIN); */
#endif
#ifdef CONFIG_ARMV7M_USEBASEPRI
  stm32l4_prioritize_syscall(NVIC_SYSH_SVCALL_PRIORITY);
#endif

  /* If the MPU is enabled, then attach and enable the Memory Management
   * Fault handler.
   */

#ifdef CONFIG_ARMV7M_MPU
  irq_attach(STM32L4_IRQ_MEMFAULT, up_memfault, NULL);
  up_enable_irq(STM32L4_IRQ_MEMFAULT);
#else
  irq_attach(STM32L4_IRQ_MEMFAULT, up_memfault, NULL);
#endif
  irq_attach(STM32L4_IRQ_BUSFAULT, up_busfault, NULL);
  irq_attach(STM32L4_IRQ_USAGEFAULT, up_usagefault, NULL);

  /* Attach all other processor exceptions (except reset and sys tick) */

#ifdef CONFIG_DEBUG
  irq_attach(STM32L4_IRQ_NMI, stm32l4_nmi, NULL);
  irq_attach(STM32L4_IRQ_PENDSV, stm32l4_pendsv, NULL);
  irq_attach(STM32L4_IRQ_DBGMONITOR, stm32l4_dbgmonitor, NULL);
  irq_attach(STM32L4_IRQ_RESERVED, stm32l4_reserved, NULL);
#endif

#ifndef CONFIG_SUPPRESS_INTERRUPTS

  /* And finally, enable interrupts */

  irqenable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
}

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
}
#endif
