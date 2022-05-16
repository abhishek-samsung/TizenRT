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
#include "amebad_i2s.h"

#include "objects.h"
#include "PinNames.h"
#include "component/common/mbed/hal/gpio_api.h"

extern FAR struct i2s_dev_s *amebad_i2s_initialize(uint16_t port);
extern int amebad_gpio_write(PinName pin, FAR unsigned int value);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* i2c config */
#define UB6470_I2C_PORT        0
#define UB6470_I2C_FREQ        100000
#define UB6470_I2C_ADDRLEN     7

#define UB6470_GPIO_A0_FAULT       9 //I2C_STRAP_PIN
#define UB6470_GPIO_A0_FAULT_VAL   1 //HIGH
#define UB6470_I2C_ADDR_H          0x1A
#define UB6470_I2C_ADDR_L          0x1A 

/* i2s config */
#define UB6470_I2S_PORT 0
#define UB6470_I2S_IS_MASTER 1
#define UB6470_I2S_IS_OUTPUT 1

/*other pin config */
#ifdef CONFIG_UB6470_RESET_GPIO
#define UB6470_GPIO_RESET_PIN CONFIG_UB6470_RESET_GPIO  //active low
#else
#define UB6470_GPIO_RESET_PIN 12  /* active low */
#endif
#define UB6470_GPIO_PDN 0  /* active low */

#define UB6470_AVAILABLE_MINOR_MIN 0
#define UB6470_AVAILABLE_MINOR_MAX 25

#ifdef CONFIG_AUDIO_UB6470

extern struct i2s_dev_s *amebad_i2s_initialize(uint16_t port);

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cy4390x_ub6470_audioinfo_s {
	/* Standard MW8904 interface */
	struct ub6470_lower_s lower;
	/* Extensions for the cy4390x board */
	ub6470_handler_t handler;
	FAR void *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void ub6470_control_reset_pin(bool active);
static void ub6470_control_powerdown(bool powerdown);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct cy4390x_ub6470_audioinfo_s g_ub6470info = {
	.lower = {
		.i2c_config = {
			.frequency = UB6470_I2C_FREQ,
#if (UB6470_GPIO_A0_FAULT_VAL)
			.address = UB6470_I2C_ADDR_H,
#else
			.address = UB6470_I2C_ADDR_L,
#endif
			.addrlen = UB6470_I2C_ADDRLEN,
		},
	},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

gpio_t gpio_pa0;

static void ub6470_control_reset_pin(bool active)
{
	//The reset pins are shared with cx20921 and ub6470. the reset pin is controlled here  //CONFIG_AUDIO_UB6470_CX20921_SHARED_RESET_PIN
	if(active)
	{
		lldbg("reset low\n");
		//amebad_gpio_write(_PA_0, 0);
		gpio_write(&gpio_pa0, 0);
		for (int i = 0; i < 100000000; i++) {
                        int ans;
                        for (int j = 0; j <1000; j++) ans = j;
                        //if (i %10000000 == 0) lldbg("waiting after power off %d\n", ans);
                }
		//amebad_gpio_write(_PA_0, 1);
		gpio_write(&gpio_pa0, 1);
		//lldbg("power on\n");
		for (int i = 0; i < 100000000; i++) {
			int ans;
			for (int j = 0; j <1000; j++) ans = j;
			//if (i %10000000 == 0) lldbg("waiting after power up %d\n", ans);
		}
		lldbg("reset high\n");
	}
	else
	{
		audvdbg("exit hw reset\n");
	}
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
int rtl_ub6470_initialize(int minor)
{
	FAR struct audio_lowerhalf_s *ub6470;
	FAR struct i2c_dev_s *i2c;
	FAR struct i2s_dev_s *i2s = NULL;
	static bool initialized = false; // for compilation sake
	char devname[12];
	int ret;

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
		/* Get an instance of the I2C interface for the UB6470 chip select */
		lldbg("calling init\n");
		lldbg("i2c init called\n");
		i2c = up_i2cinitialize(UB6470_I2C_PORT);
		if (!i2c) {
			auddbg("ERROR: Failed to initialize CODEC I2C%d\n");
			ret = -ENODEV;
			goto errout_with_i2c;
		}
		lldbg("i2c init done\n");
		lldbg("calling init i2s\n");
		/* Get an instance of the I2S interface for the UB6470 data channel */
		i2s = amebad_i2s_initialize(UB6470_I2S_PORT);
		if (!i2s) {
			auddbg("ERROR: Failed to initialize I2S\n");
			ret = -ENODEV;
			goto errout_with_i2s;
		}
		
		lldbg("calling init i2s done\n");

		gpio_init(&gpio_pa0, PA_0);
        	gpio_write(&gpio_pa0, 0);
        	gpio_dir(&gpio_pa0, PIN_OUTPUT);
        	gpio_mode(&gpio_pa0, PullNone);
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
		lldbg("calling init lower\n");
		ub6470 = ub6470_initialize(i2c, i2s, &g_ub6470info.lower);
		if (!ub6470) {
			auddbg("ERROR: Failed to initialize the UB6470\n");
			ret = -ENODEV;
			goto errout_with_ub6470;
		}
		lldbg("calling init lower done\n");
		/* Finally, we can register the PCM/UB6470/I2C/I2S audio device.
		 *
		 * Is anyone young enough to remember Rube Goldberg?
		 */
		lldbg("registering audio device here\n");
		ret = audio_register(devname, ub6470);
		lldbg("registrred the device\n");
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
