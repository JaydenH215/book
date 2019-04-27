/**
 * Dimmer Node - behaviour based on the Bluetooth mesh Generic Level Client Model
 * 
 **/

#include <misc/printk.h>
#include <board.h>
#include <gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

// GPIO for the buttons
#define PIN_A SW0_GPIO_PIN
#define PIN_B SW1_GPIO_PIN
#define PORT SW0_GPIO_NAME
#define EDGE (GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE)

static struct gpio_callback gpio_btnA_cb;
static struct gpio_callback gpio_btnB_cb;

// for use with k_work_submit which we use to handle button presses in a background thread to avoid holding onto an IRQ for too long
static struct k_work buttonA_work;
static struct k_work buttonB_work;

// addresses


// security keys

// indexes

// other 
#define MAX_DELTA_LEVEL_32 65535
#define MAX_DELTA_LEVEL_16 32767
#define LEVEL_BAND_SIZE 3276

// device UUID

// provisioning properties and capabilities


// -------------------------------------------------------------------------------------------------------
// Configuration Client
// --------------------

// -------------------------------------------------------------------------------------------------------
// Configuration Server
// --------------------


// -------------------------------------------------------------------------------------------------------
// Health Server
// -------------


// -------------------------------------------------------------------------------------------------------
// Generic OnOff Client Model
// --------------------------

// handler functions for this model's RX messages


// operations supported by this model. opcode, min msglen, message handler

// -------------------------------------------------------------------------------------------------------
// Composition
// -----------

// Generic Level Client - TX message producer functions
// -----------------------------------------------------------


// -------------------------------------------------------------------------------------------------------
// Self Provisioning
// -----------------


// -------------------------------------------------------------------------------------------------------
// Self Configuring
// ----------------


// -------------------------------------------------------------------------------------------------------
// misc functions
// --------------------

static s32_t mapDimmerSettingToDelta(s8_t setting_change) {
  s32_t delta = ((s32_t) setting_change * LEVEL_BAND_SIZE);
  printk("setting_change=%d mapped to delta=%d\n",setting_change,delta);
  return delta;
}

s16_t get_random_level() {
	// generate a random, positive level value. Positive because we're only supporting positive values in this "device".
	u32_t r = sys_rand32_get();
	s16_t lvl = (s16_t) (r % 32767);
	return lvl;
}

void buttonA_work_handler(struct k_work *work)
{
	printk("buttonA_work_handler\n");
}

void buttonB_work_handler(struct k_work *work)
{
	printk("buttonB_work_handler\n");
}

void button_A_pressed(struct device *gpiob, struct gpio_callback *cb,
											u32_t pins)
{
	// printk("Button A pressed at %d\n", k_cycle_get_32());
	k_work_submit(&buttonA_work);
}

void button_B_pressed(struct device *gpiob, struct gpio_callback *cb,
											u32_t pins)
{
	// printk("Button B pressed at %d\n", k_cycle_get_32());
	k_work_submit(&buttonB_work);
}

// -------------------
// mesh initialisation
// -------------------


// -------------------------------------------------------------------------------------------------------
// Buttons
// -------

void configureButtons(void)
{
	struct device *gpiob;

	printk("Press button A or button B\n");
	gpiob = device_get_binding(PORT);
	if (!gpiob)
	{
		printk("error\n");
		return;
	}

	// Button A
	k_work_init(&buttonA_work, buttonA_work_handler);
	gpio_pin_configure(gpiob, PIN_A, GPIO_DIR_IN | GPIO_INT | EDGE);
	gpio_init_callback(&gpio_btnA_cb, button_A_pressed, BIT(PIN_A));
	gpio_add_callback(gpiob, &gpio_btnA_cb);
	gpio_pin_enable_callback(gpiob, PIN_A);

	// Button B
	k_work_init(&buttonB_work, buttonB_work_handler);
	gpio_pin_configure(gpiob, PIN_B, GPIO_DIR_IN | GPIO_INT | EDGE);
	gpio_init_callback(&gpio_btnB_cb, button_B_pressed, BIT(PIN_B));
	gpio_add_callback(gpiob, &gpio_btnB_cb);
	gpio_pin_enable_callback(gpiob, PIN_B);
}

void main(void)
{

}
