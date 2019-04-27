/**
 * Light Node - behaviour based on the Bluetooth mesh Generic OnOff Server Generic Level Models
 * 
 **/
#include <stdlib.h>
#include <misc/printk.h>
#include <board.h>
#include <gpio.h>
#include <display/mb_display.h>
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

// states and state changes
u8_t  onoff_state;
u8_t  target_onoff_state;

s16_t level_state;
s16_t target_level_state;

u8_t transition_in_progress = 0;
u8_t transition_stop_requested = 0;
s64_t transition_start_timestamp;

u8_t transition_time = 0;
u32_t total_transition_duration = 0; 

u8_t delay = 0;
s32_t delta_level = 0;

u8_t target_led_increments  = 0;  // number of LED lights to be switched on/off by the transition
u8_t count_led_increments   = 0;  // number of LED lights switched on/off by the transition so far
u32_t led_interval = 0;           // time between each LED being switch on/off during a transition
s16_t one_led_level_increment = 0;    // amount to add to the level state to switch on/off one extra LED. +/- the level_one_step_value
u8_t steps_multiplier = 0;
u8_t resolution = 0;
u16_t level_one_step_value = 0;
// initial level value to be adjusted by received delta values in a transaction
s16_t initial_trans_level;
u8_t last_delta_tid;

s64_t last_message_timestamp;
u16_t last_message_src;
u16_t last_message_dst;
u8_t last_message_tid;

static struct k_work onoff_set_work;
static struct k_work onoff_status_work;

static struct k_work level_set_work;
static struct k_work level_status_work;

// addresses
u16_t remote_addr = 0;
#define NODE_ADDR 0x0007
#define GROUP_ADDR 0xc000

// security keys
static const u8_t net_key[16] = {
		0x01,
		0x23,
		0x45,
		0x67,
		0x89,
		0xab,
		0xcd,
		0xef,
		0x01,
		0x23,
		0x45,
		0x67,
		0x89,
		0xab,
		0xcd,
		0xef,
};
static const u8_t dev_key[16] = {
		0x01,
		0x23,
		0x45,
		0x67,
		0x89,
		0xab,
		0xcd,
		0xef,
		0x01,
		0x23,
		0x45,
		0x67,
		0x89,
		0xab,
		0xcd,
		0xef,
};
static const u8_t app_key[16] = {
		0x01,
		0x23,
		0x45,
		0x67,
		0x89,
		0xab,
		0xcd,
		0xef,
		0x01,
		0x23,
		0x45,
		0x67,
		0x89,
		0xab,
		0xcd,
		0xef,
};

// indexes
static const u16_t net_idx;
static const u16_t app_idx;
static const u32_t iv_index;

// other 
#define MAX_LEVEL 32767
static u8_t flags;
static u16_t addr = NODE_ADDR;

// micro:bit display patterns for OFF and ON
static const struct mb_image led_patterns[] = {
		MB_IMAGE({0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0}),
		MB_IMAGE({0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0},
						 {0, 0, 1, 0, 0},
						 {0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0}),
		MB_IMAGE({0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0},
						 {0, 1, 1, 0, 0},
						 {0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0}),
		MB_IMAGE({0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0},
						 {0, 1, 1, 1, 0},
						 {0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0}),
		MB_IMAGE({0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0},
						 {1, 1, 1, 1, 0},
						 {0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0}),
		MB_IMAGE({0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0},
						 {1, 1, 1, 1, 1},
						 {0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0}),
		MB_IMAGE({0, 0, 0, 0, 0},
						 {0, 0, 1, 0, 0},
						 {1, 1, 1, 1, 1},
						 {0, 0, 0, 0, 0},
						 {0, 0, 0, 0, 0}),
		MB_IMAGE({0, 0, 0, 0, 0},
						 {0, 0, 1, 0, 0},
						 {1, 1, 1, 1, 1},
						 {0, 0, 1, 0, 0},
						 {0, 0, 0, 0, 0}),
		MB_IMAGE({0, 0, 1, 0, 0},
						 {0, 0, 1, 0, 0},
						 {1, 1, 1, 1, 1},
						 {0, 0, 1, 0, 0},
						 {0, 0, 0, 0, 0}),
		MB_IMAGE({0, 0, 1, 0, 0},
						 {0, 0, 1, 0, 0},
						 {1, 1, 1, 1, 1},
						 {0, 0, 1, 0, 0},
						 {0, 0, 1, 0, 0}),
		MB_IMAGE({0, 0, 1, 0, 0},
						 {0, 1, 1, 0, 0},
						 {1, 1, 1, 1, 1},
						 {0, 0, 1, 0, 0},
						 {0, 0, 1, 0, 0}),
		MB_IMAGE({0, 0, 1, 0, 0},
						 {0, 1, 1, 0, 0},
						 {1, 1, 1, 1, 1},
						 {0, 0, 1, 1, 0},
						 {0, 0, 1, 0, 0}),
		MB_IMAGE({0, 0, 1, 0, 0},
						 {0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {0, 0, 1, 1, 0},
						 {0, 0, 1, 0, 0}),
		MB_IMAGE({0, 0, 1, 0, 0},
						 {0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0},
						 {0, 0, 1, 0, 0}),
		MB_IMAGE({0, 1, 1, 0, 0},
						 {0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0},
						 {0, 0, 1, 0, 0}),
		MB_IMAGE({0, 1, 1, 0, 0},
						 {0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0},
						 {0, 0, 1, 1, 0}),
		MB_IMAGE({0, 1, 1, 1, 0},
						 {0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0},
						 {0, 0, 1, 1, 0}),
		MB_IMAGE({0, 1, 1, 1, 0},
						 {0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0},
						 {0, 1, 1, 1, 0}),
		MB_IMAGE({0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0},
						 {0, 1, 1, 1, 0}),
		MB_IMAGE({0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0}),
		MB_IMAGE({0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0}),
		MB_IMAGE({0, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0}),
		MB_IMAGE({1, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 0}),
		MB_IMAGE({1, 1, 1, 1, 0},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 1}),
		MB_IMAGE({1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {0, 1, 1, 1, 1}),
		MB_IMAGE({1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1},
						 {1, 1, 1, 1, 1})
};

// device UUID
// cfa0ea7e-17d9-11e8-86d1-5f1ce28adea7
static const uint8_t dev_uuid[16] = {0xcf, 0xa0, 0xea, 0x7e, 0x17, 0xd9, 0x11, 0xe8, 0x86, 0xd1, 0x5f, 0x1c, 0xe2, 0x8a, 0xde, 0xa7};
static const struct bt_mesh_prov prov = {
		.uuid = dev_uuid,
};

void derive_transition_time_values(u8_t transition_time) {

}

// -------------------------------------------------------------------------------------------------------
// Generic OnOff Server
// --------------------

// message opcodes
#define BT_MESH_MODEL_OP_gen_onoff_get	      BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET	      BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS   	BT_MESH_MODEL_OP_2(0x82, 0x04)

// need to forward declare as we have circular dependencies
void generic_onoff_status(u8_t on_or_off, u16_t dest_addr, u8_t transitioning, u8_t target_on_or_off, u8_t remaining_time);

void onoff_work_handler(struct k_work *work)
{
}

static void generic_onoff_get(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}

static void generic_onoff_set(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}

static void generic_onoff_set_unack(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}


static const struct bt_mesh_model_op generic_onoff_op[] = {
		{BT_MESH_MODEL_OP_gen_onoff_get, 0, generic_onoff_get},
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_SET, 2, generic_onoff_set},
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK, 2, generic_onoff_set_unack},
		BT_MESH_MODEL_OP_END,
};

// model publication context
// See access.h. Contains a large number of parameters relating to publication including the pub addr.
// Ref 4.2.2 in the spec "Model Publication"
static struct bt_mesh_model_pub generic_onoff_pub;

// change onoff state

// handler functions for this model's RX messages

// operations supported by this model. opcode, min msglen, message handler

// ----------------------------------------------------------------------------------------------------
// generic onoff status TX message producer

// -------------------------------------------------------------------------------------------------------
// Generic Level Server
// --------------------

// message opcodes
#define BT_MESH_MODEL_OP_GENERIC_LEVEL_GET	      BT_MESH_MODEL_OP_2(0x82, 0x05)
#define BT_MESH_MODEL_OP_GENERIC_LEVEL_SET	      BT_MESH_MODEL_OP_2(0x82, 0x06)
#define BT_MESH_MODEL_OP_GENERIC_LEVEL_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x07)
#define BT_MESH_MODEL_OP_GENERIC_LEVEL_STATUS   	BT_MESH_MODEL_OP_2(0x82, 0x08)

#define BT_MESH_MODEL_OP_GENERIC_DELTA_SET	      BT_MESH_MODEL_OP_2(0x82, 0x09)
#define BT_MESH_MODEL_OP_GENERIC_DELTA_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x0A)

#define BT_MESH_MODEL_OP_GENERIC_MOVE_SET	        BT_MESH_MODEL_OP_2(0x82, 0x0B)
#define BT_MESH_MODEL_OP_GENERIC_MOVE_SET_UNACK	  BT_MESH_MODEL_OP_2(0x82, 0x0C)

// need to forward declare as we have circular dependencies
void generic_level_status(u16_t dest_addr);

// change level state
static void set_level_state(s16_t level)
{
	struct mb_display *disp = mb_display_get();
	level_state = level;
	u8_t level_inx = 0;
	if (level_state > 0) {
	  level_inx = (u8_t) (level_state / level_one_step_value) + 1;
		if (level_inx > 25) {
			level_inx = 25;
		}
	}
	printk("level_state=%d level_inx=%d\n",level_state,level_inx);
  mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, K_FOREVER, &led_patterns[level_inx], 1);
}

// transition timer

// need to forward declare as we have circular dependencies
void level_transition_timer_handler(struct k_timer *dummy);

K_TIMER_DEFINE(level_transition_timer, level_transition_timer_handler, NULL);

void level_transition_work_handler(struct k_work *work)
{
	s64_t now = k_uptime_get();
	printk("%lld> target_level_state=%d level_state=%d\n", now, target_level_state, level_state);
	// have we gone through all required transition steps or reached a limit?
	s32_t new_level = level_state + one_led_level_increment;
	if (new_level > MAX_LEVEL)
	{
		new_level = 0;
	}
	else if (new_level < 0)
	{
		new_level = MAX_LEVEL;
	}
	level_state = new_level;
	set_level_state(level_state);
	if ((transition_stop_requested == 1))
	{
		printk("Stopping transition timer\n");
		transition_in_progress = 0;
		k_timer_stop(&level_transition_timer);
	}
}

K_WORK_DEFINE(level_transition_work, level_transition_work_handler);

void level_work_handler(struct k_work *work)
{
}

void level_transition_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&level_transition_work);
}

static void generic_level_set(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}

static void generic_level_set_unack(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}


static void generic_delta_set(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}

static void generic_delta_set_unack(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}

static void generic_move_set(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}

static void generic_move_set_unack(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}

static void generic_level_get(struct bt_mesh_model *model,struct bt_mesh_msg_ctx *ctx,struct net_buf_simple *buf)
{
}


static const struct bt_mesh_model_op generic_level_op[] = {
		{BT_MESH_MODEL_OP_2(0x82, 0x05), 0, generic_level_get},
		{BT_MESH_MODEL_OP_2(0x82, 0x06), 0, generic_level_set},
		{BT_MESH_MODEL_OP_2(0x82, 0x07), 0, generic_level_set_unack},
		{BT_MESH_MODEL_OP_2(0x82, 0x09), 0, generic_delta_set},
		{BT_MESH_MODEL_OP_2(0x82, 0x0A), 0, generic_delta_set_unack},
		{BT_MESH_MODEL_OP_2(0x82, 0x0B), 0, generic_move_set},
		{BT_MESH_MODEL_OP_2(0x82, 0x0C), 0, generic_move_set_unack},
		BT_MESH_MODEL_OP_END,
};

// model publication context
// See access.h. Contains a large number of parameters relating to publication including the pub addr.
// Ref 4.2.2 in the spec "Model Publication"
static struct bt_mesh_model_pub generic_level_pub;

// handler functions for this model's RX messages

// operations supported by this model. opcode, min msglen, message handler

// Configuration Client Model
// -------------------------------------------------------------------------------------------------------
static struct bt_mesh_cfg_cli cfg_cli = {
};


// -------------------------------------------------------------------------------------------------------
// Configuration Server
// --------------------
static struct bt_mesh_cfg_srv cfg_srv = {
		.relay = BT_MESH_RELAY_DISABLED,
		.beacon = BT_MESH_BEACON_DISABLED,
		.frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
		.gatt_proxy = BT_MESH_GATT_PROXY_NOT_SUPPORTED,
		.default_ttl = 7,
		/* 3 transmissions with 20ms interval */
		.net_transmit = BT_MESH_TRANSMIT(2, 20),
};

// -------------------------------------------------------------------------------------------------------
// Health Server
// -------------
BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);
static struct bt_mesh_health_srv health_srv = {};

// -------------------------------------------------------------------------------------------------------
// Buttons - switch and off using buttons for convenience when testing
// -------------

// ON
void buttonA_work_handler(struct k_work *work)
{
	// onoff_state = 1;
	struct mb_display *disp = mb_display_get();
  mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, K_FOREVER, &led_patterns[1], 1);
}

// OFF
void buttonB_work_handler(struct k_work *work)
{
	// onoff_state = 0;
	struct mb_display *disp = mb_display_get();
  mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, K_FOREVER, &led_patterns[0], 1);
}

void button_A_pressed(struct device *gpiob, struct gpio_callback *cb,
											u32_t pins)
{
	k_work_submit(&buttonA_work);
}

void button_B_pressed(struct device *gpiob, struct gpio_callback *cb,
											u32_t pins)
{
	k_work_submit(&buttonB_work);
}

// -------------------------------------------------------------------------------------------------------
// Composition
// -----------

static struct bt_mesh_model sig_models[] = {
		BT_MESH_MODEL_CFG_SRV(&cfg_srv),
		BT_MESH_MODEL_CFG_CLI(&cfg_cli),
		BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
		BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, generic_onoff_op,
									&generic_onoff_pub, NULL),
		BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_SRV, generic_level_op,
									&generic_level_pub, NULL)};

// node contains elements.note that BT_MESH_MODEL_NONE means "none of this type" ands here means "no vendor models"
static struct bt_mesh_elem elements[] = {
		BT_MESH_ELEM(0, sig_models, BT_MESH_MODEL_NONE),
};

// node
static const struct bt_mesh_comp comp = {
		.elem = elements,
		.elem_count = ARRAY_SIZE(elements),
};

// ----------------------------------------------------------------------------------------------------
// generic level status TX message producer

// -------------------------------------------------------------------------------------------------------

void generic_level_status(u16_t dest_addr) {
       
}


// Self Provisioning

static int selfProvision(void) 
{
	// now we provision ourselves... this is not how it would normally be done!
	int err = bt_mesh_provision(net_key, net_idx, flags, iv_index, addr, dev_key);
	if (err)
	{
		printk("Provisioning failed (err %d)\n", err);
		return err;
	}
	printk("Provisioning completed\n");

  return 0;
}

// -------------------------------------------------------------------------------------------------------
static int selfConfigure(void)
{
	int err;
	printk("configuring...\n");

	/* Add Application Key */
	err = bt_mesh_cfg_app_key_add(net_idx, addr, net_idx, app_idx, app_key, NULL);
	if (err)
	{
		printk("ERROR adding appkey (err %d)\n", err);
		return err;
	}
	else
	{
		printk("added appkey\n");
	}

	/* Bind to generic onoff server model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, addr, addr, app_idx, BT_MESH_MODEL_ID_GEN_ONOFF_SRV, NULL);
	if (err)
	{
		printk("ERROR binding to generic onoff server model (err %d)\n", err);
		return err;
	}
	else
	{
		printk("bound appkey to generic onoff server model\n");
	}

	/* Bind to generic level server model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, addr, addr, app_idx, BT_MESH_MODEL_ID_GEN_LEVEL_SRV, NULL);
	if (err)
	{
		printk("ERROR binding to generic level server model (err %d)\n", err);
		return err;
	}
	else
	{
		printk("bound appkey to generic onoff server model\n");
	}

	/* Bind to Health model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, addr, addr, app_idx, BT_MESH_MODEL_ID_HEALTH_SRV, NULL);
	if (err)
	{
		printk("ERROR binding to health server model (err %d)\n", err);
		return err;
	}
	else
	{
		printk("bound appkey to health server model\n");
	}

	// subscribe to the group address
	err = bt_mesh_cfg_mod_sub_add(net_idx, NODE_ADDR, NODE_ADDR, GROUP_ADDR, BT_MESH_MODEL_ID_GEN_ONOFF_SRV, NULL);
	if (err)
	{
		printk("ERROR subscribing to group address (err %d)\n", err);
		return err;
	}
	else
	{
		printk("subscribed to group address\n");
	}

	return 0;
}

static void bt_ready(int err)
{
	if (err)
	{
		printk("bt_enable init failed with err %d\n", err);
		return;
	}

	printk("Bluetooth initialised OK\n");

	// self-provision and initialise with node composition data
	err = bt_mesh_init(&prov, &comp);
	if (err)
	{
		printk("bt_mesh_init failed with err %d\n", err);
		return;
	}

  err = selfProvision();
  if (err) {
		printk("ERROR: SELF-PROVISIONING FAILED");
	} else {
   	printk("self-provisioned OK\n");
	}

	err = selfConfigure();

  if (err) {
		printk("ERROR: INITIALISATION FAILED");
	} else {
   	printk("Mesh initialised OK\n");
	}
}

// -------------------------------------------------------------------------------------------------------
// Buttons

void configureButtons(void)
{
	struct device *gpiob;

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
	int err;
	printk("generic onoff server for node %d\n",addr);
	level_one_step_value = MAX_LEVEL / 25;

  	// reset the display to all LEDs off
	struct mb_display *disp = mb_display_get();
	mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, K_FOREVER, &led_patterns[0], 1);

	configureButtons();

	// set up the workers for handling set operations
	k_work_init(&onoff_set_work, onoff_work_handler);
	k_work_init(&onoff_status_work, onoff_work_handler);

	k_work_init(&level_set_work, level_work_handler);
	k_work_init(&level_status_work, level_work_handler);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err)
	{
		printk("bt_enable failed with err %d\n", err);
	}

}
