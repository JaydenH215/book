/**
 * On/Off Switch Node - behaviour predominantly based on the Bluetooth mesh Generic OnOff Client Model
 * 
 **/

#include <misc/printk.h>
#include <board.h>
#include <gpio.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>
#include <display/mb_display.h>

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

u8_t onoff[] = {
		0,
		1};

// handler functions for this model's RX messages


// message types defined by this model.

// -------------------------------------------------------------------------------------------------------
// Composition
// -----------

// Generic OnOff Client - TX message producer functions
// -----------------------------------------------------------


// -------------------------------------------------------------------------------------------------------
// Self Provisioning
// -----------------


// -------------------------------------------------------------------------------------------------------
// Self Configuring
// ----------------


void buttonA_work_handler(struct k_work *work)
{
	printk("Button A work handler\n");
  // genericOnOffSetUnAck(onoff[1]);
	// genericOnOffSet(onoff[1]);
}

void buttonB_work_handler(struct k_work *work)
{
	printk("Button B work handler\n");
  // genericOnOffSetUnAck(onoff[0]);
	// genericOnOffSet(onoff[0]);
 	// genericOnOffGet();
}

void button_A_pressed(struct device *gpiob, struct gpio_callback *cb,
											u32_t pins)
{
	printk("Button A pressed at %d\n", k_cycle_get_32());
	k_work_submit(&buttonA_work);
}

void button_B_pressed(struct device *gpiob, struct gpio_callback *cb,
											u32_t pins)
{
	printk("Button B pressed at %d\n", k_cycle_get_32());
	k_work_submit(&buttonB_work);
}

// -------------------------------------------------------------------------------------------------------
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
