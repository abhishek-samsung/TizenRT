/************************************************************************************
 * Name: up_flashinitialize
 *
 * Description:
 *   Return an initialize MTD device instance.  MTD devices are not registered
 *   in the file system, but are created as instances that can be bound to
 *   other functions (such as a block or character driver front end).
 *
 ************************************************************************************/
FAR struct mtd_dev_s *up_flashinitialize(void)
{
}

#endif							/* defined(CONFIG_STM32_STM32F10XX) || defined(CONFIG_STM32_STM32F40XX) */
