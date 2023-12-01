/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <assert.h>
#include <errno.h>

#include <tinyara/irq.h>
#include <tinyara/audio/i2s.h>
#include <tinyara/audio/ub6470.h>
#include <tinyara/gpio.h>
#include "amebasmart_i2s.h"

#include "objects.h"
#include "PinNames.h"
#include "gpio_api.h"

extern FAR struct i2s_dev_s *amebasmart_i2s_initialize(uint16_t port);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* i2c config */
#define UB6470_I2C_PORT			1 /* 1 or 2 */
#define UB6470_I2C_FREQ			100000
#define UB6470_I2C_ADDRLEN		7

#define UB6470_I2C_ADDR_H		0x1B /* TODO: need to confirm this, the data sheet shows different values, also can be changed based on 2 gpio pin configs */
#define UB6470_I2C_ADDR_L		0x28 

/* i2s config */
#define UB6470_I2S_PORT			2 /* TODO: need to see if anything new to be done for port number 3 */
#define UB6470_I2S_IS_MASTER		1
#define UB6470_I2S_IS_OUTPUT		1

/*other pin config */
#define UB6470_GPIO_RESET_PIN 		PA_21 /* TODO: need to get the reset pin from the pin mappings chart */

#define UB6470_AVAILABLE_MINOR_MIN 0
#define UB6470_AVAILABLE_MINOR_MAX 25

#ifdef CONFIG_AUDIO_UB6470

extern struct i2s_dev_s *amebad_i2s_initialize(uint16_t port);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rtl8730e_ub6470_audioinfo_s {
	struct ub6470_lower_s lower;
	ub6470_handler_t handler;
	FAR void *arg;
	gpio_t reset;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void ub6470_control_reset_pin(bool active);
static void ub6470_control_powerdown(bool powerdown);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct rtl8730e_ub6470_audioinfo_s g_ub6470info = {
	.lower = {
		.i2c_config = {
			.frequency = UB6470_I2C_FREQ,
			.address = UB6470_I2C_ADDR_H,
			.addrlen = UB6470_I2C_ADDRLEN,
		},
	},
	.reset.pin = UB6470_GPIO_RESET_PIN,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ub6470_control_reset_pin(bool active)
{
#if 0
	if(active)
	{
		/* Need to hold it low for some time of around 10ms for reset */
		gpio_write(&(g_ub6470info.reset), 0);
		up_mdelay(20);
		gpio_write(&(g_ub6470info.reset), 1);
		up_mdelay(10);
	}
	else
	{
		audvdbg("exit hw reset\n");
	}
#endif
}

static void ub6470_control_powerdown(bool enter_powerdown)
{
#ifdef CONFIG_UB6470_EXCLUDE_PDN_CONTROL
#else
	if(enter_powerdown)
	{
		//amebad_gpio_write(_PA_0, 0);
		audvdbg("enter powerdown\n");
	}
	else
	{
		//amebad_gpio_write(_PA_0, 1);
		audvdbg("exit powerdown\n");
	}
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cy4390x_ub6470_initialize
 *
 * Description:
 *   This function is called by platform-specific, setup logic to configure
 *   and register the UB6470 device.  This function will register the driver
 *   as /dev/ub6470[x] where x is determined by the minor device number.
 *
 * Input Parameters:
 *   minor - The input device minor number
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/
int rtl8730e_ub6470_initialize(int minor)
{
	FAR struct audio_lowerhalf_s *ub6470;
	FAR struct i2c_dev_s *i2c;
	FAR struct i2s_dev_s *i2s = NULL;
	static bool initialized = false; // for compilation sake
	char devname[12];
	int ret;

	lldbg("set SDB pin(PA22) of amp to 1,\n");
	gpio_t reset;
	gpio_init(&reset, PA_22);
	gpio_dir(&reset, PIN_OUTPUT);
	gpio_mode(&reset, PullNone);
	gpio_write(&reset, 1);
	lldbg("now AMP should be on\n");

	audvdbg("minor %d\n", minor);
	DEBUGASSERT(minor >= UB6470_AVAILABLE_MINOR_MIN && minor <= UB6470_AVAILABLE_MINOR_MAX);

#define UB6470_AVAIALBLE_MINOR_MIN 0
#define UB6470_AVAIALBLE_MINOR_MAX 25

	/* Have we already initialized?  Since we never uninitialize we must prevent
	 * multiple initializations.  This is necessary, for example, when the
	 * touchscreen example is used as a built-in application in NSH and can be
	 * called numerous time.  It will attempt to initialize each time.
	 */

	if (!initialized) {
		/* Reset pin initialization */
		gpio_init(&(g_ub6470info.reset), UB6470_GPIO_RESET_PIN);
		gpio_dir(&(g_ub6470info.reset), PIN_OUTPUT);
		gpio_mode(&(g_ub6470info.reset), PullNone);

		/* Get an instance of the I2C interface for the UB6470 chip select */
		i2c = up_i2cinitialize(UB6470_I2C_PORT);
		if (!i2c) {
			auddbg("ERROR: Failed to initialize CODEC I2C%d\n");
			ret = -ENODEV;
			goto errout_with_i2c;
		}
		auddbg("i2c init done\n");
		auddbg("calling init i2s\n");
		/* Get an instance of the I2S interface for the UB6470 data channel */
		i2s = amebasmart_i2s_initialize(UB6470_I2S_PORT);
		if (!i2s) {
			auddbg("ERROR: Failed to initialize I2S\n");
			ret = -ENODEV;
			goto errout_with_i2s;
		}
		
		auddbg("calling init i2s done\n");

#ifdef CONFIG_UB6470_EXCLUDE_PDN_CONTROL
		g_ub6470info.lower.control_powerdown = ub6470_control_powerdown;
#else
		g_ub6470info.lower.control_powerdown = ub6470_control_powerdown;
		g_ub6470info.lower.control_powerdown(false);
#endif

		g_ub6470info.lower.control_hw_reset = ub6470_control_reset_pin;
		g_ub6470info.lower.control_hw_reset(true); /*power on*/
		g_ub6470info.lower.control_hw_reset(false); /*does nothing*/

		/* Now we can use these I2C and I2S interfaces to initialize the
		* UB6470 which will return an audio interface.
		*/
		/* Create a device name */
		snprintf(devname, sizeof(devname), "pcmC%uD%u%c", minor, 0, 'p');
		auddbg("calling init lower\n");
		ub6470 = ub6470_initialize(i2c, i2s, &g_ub6470info.lower);
		if (!ub6470) {
			auddbg("ERROR: Failed to initialize the UB6470\n");
			ret = -ENODEV;
			goto errout_with_ub6470;
		}
		auddbg("calling init lower done\n");
		/* Finally, we can register the PCM/UB6470/I2C/I2S audio device.
		 *
		 * Is anyone young enough to remember Rube Goldberg?
		 */
		auddbg("registering audio device here\n");
		ret = audio_register(devname, ub6470);
		auddbg("registrred the device\n");
		if (ret < 0) {
			auddbg("ERROR: Failed to register /dev/%s device: %d\n", devname, ret);
			goto errout_with_pcm;
		}

		/* Now we are initialized */

		initialized = true;
	}

	return OK;

	/* Error exits.  Unfortunately there is no mechanism in place now to
	 * recover resources from most errors on initialization failures.
	 */

errout_with_pcm:
errout_with_i2c:
errout_with_ub6470:
errout_with_i2s:
	return ret;
}
#endif
