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
#define NODE_ADDR 0x0002
#define GROUP_ADDR 0xc000
static u16_t target = GROUP_ADDR;
static u16_t node_addr = NODE_ADDR;

// security keys
// 0123456789abcdef0123456789abcdef
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

// 0123456789abcdef0123456789abcdef
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

// 0123456789abcdef0123456789abcdef
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
// 4.3.1.1 Key indexes
static const u16_t net_idx;
static const u16_t app_idx;
// 3.8.4 IV Index
static const u32_t iv_index;

// other 
#define MAX_DELTA_LEVEL_32 65535
#define MAX_DELTA_LEVEL_16 32767
#define LEVEL_BAND_SIZE 3276
static u8_t flags;
static u8_t tid;

// device UUID

// cfa0ea7e-17d9-11e8-86d1-5f1ce28adea2
static const uint8_t dev_uuid[16] = { 0xcf, 0xa0, 0xea, 0x7e, 0x17, 0xd9, 0x11, 0xe8, 0x86, 0xd1, 0x5f, 0x1c, 0xe2, 0x8a, 0xde, 0xa2};

// provisioning properties and capabilities

static const struct bt_mesh_prov prov = {
	.uuid = dev_uuid,
};


// -------------------------------------------------------------------------------------------------------
// Configuration Client
// --------------------
static struct bt_mesh_cfg_cli cfg_cli = {};

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
// Generic Level Client Model
// --------------------------

// handler functions for this model's RX messages

static void generic_level_status(struct bt_mesh_model *model,								   struct bt_mesh_msg_ctx *ctx,								   struct net_buf_simple *buf)
{
  printk("generic_level_status\n");
	u8_t buflen = buf->len;
  s16_t present_level = (s16_t) net_buf_simple_pull_le16(buf);
  printk("present_level=%d\n",present_level);
  if (buflen > 3) {
    s16_t target_level = (s16_t) net_buf_simple_pull_le16(buf);
    printk("target_level=%d\n",target_level);
    if (buflen > 4) {
	      u8_t remaining_time = net_buf_simple_pull_u8(buf);
        printk("remaining_time=%d\n",remaining_time);
    } else {
	      printk("ERROR: remaining time field is missing\n");
    }
  }
}


// operations supported by this model. opcode, min msglen, message handler
#define BT_MESH_MODEL_OP_GEN_LEVEL_GET       BT_MESH_MODEL_OP_2(0x82, 0x05)
#define BT_MESH_MODEL_OP_GEN_LEVEL_SET       BT_MESH_MODEL_OP_2(0x82, 0x06)
#define BT_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x07)
#define BT_MESH_MODEL_OP_GEN_LEVEL_STATUS    BT_MESH_MODEL_OP_2(0x82, 0x08)
#define BT_MESH_MODEL_OP_GEN_DELTA_SET       BT_MESH_MODEL_OP_2(0x82, 0x09)
#define BT_MESH_MODEL_OP_GEN_DELTA_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x0A)
#define BT_MESH_MODEL_OP_GEN_MOVE_SET        BT_MESH_MODEL_OP_2(0x82, 0x0B)
#define BT_MESH_MODEL_OP_GEN_MOVE_SET_UNACK  BT_MESH_MODEL_OP_2(0x82, 0x0C)

static const struct bt_mesh_model_op gen_level_cli_op[] = {
	{BT_MESH_MODEL_OP_GEN_LEVEL_STATUS, 1, generic_level_status},
	BT_MESH_MODEL_OP_END,
};

// -------------------------------------------------------------------------------------------------------
// Composition
// -----------

static struct bt_mesh_model sig_models[] = {
		BT_MESH_MODEL_CFG_SRV(&cfg_srv),
		BT_MESH_MODEL_CFG_CLI(&cfg_cli),
		BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
		BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_LEVEL_CLI, gen_level_cli_op,
									NULL, 0),
};

// node contains elements. Note that BT_MESH_MODEL_NONE means "none of this type" and here means "no vendor models"
static struct bt_mesh_elem elements[] = {
		BT_MESH_ELEM(0, sig_models, BT_MESH_MODEL_NONE),
};

// node
static const struct bt_mesh_comp comp = {
		.elem = elements,
		.elem_count = ARRAY_SIZE(elements),
};

// Generic Level Client - TX message producer functions
// -----------------------------------------------------------

void generic_level_get() 
{
  // 2 bytes for the opcode
	// 0 bytes parameters: 
	// 4 additional bytes for the TransMIC
	
	NET_BUF_SIMPLE_DEFINE(msg, 2 + 0 + 4);
	struct bt_mesh_msg_ctx ctx = {
			.net_idx = net_idx,
			.app_idx = app_idx,
			.addr = target,
			.send_ttl = BT_MESH_TTL_DEFAULT,
	};
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GEN_LEVEL_GET);
	if (bt_mesh_model_send(&sig_models[3], &ctx, &msg, NULL, NULL))
	{
		printk("Unable to send generic level get message\n");
	}
	printk("level get message sent\n");
}

void generic_level_set(s16_t level, u32_t opcode) 
{
  // 2 bytes for the opcode
  // 3 bytes of parameters / payload : 16 bits level, 8 bits TID
  // 4 additional bytes for the TransMIC
  NET_BUF_SIMPLE_DEFINE(msg, 9);
  bt_mesh_model_msg_init(&msg, opcode);
  net_buf_simple_add_le16(&msg, level);
  net_buf_simple_add_u8(&msg, tid);

  struct bt_mesh_msg_ctx ctx = {
		.net_idx = net_idx,
		.app_idx = app_idx,
		.addr = target,
		.send_ttl = BT_MESH_TTL_DEFAULT,
  };

  if (bt_mesh_model_send(&sig_models[3], &ctx, &msg, NULL, NULL))
  {
	printk("Unable to send generic level set (ack/unack) message\n");
  }
  printk("level set (ack/unack) message %d sent\n",level);
}

void generic_delta_set(s32_t delta_level, u32_t opcode) 
{
  // 2 bytes for the opcode
  // 5 bytes of parameters / payload : first 32 bits is the delta level, remaining 8 bits is for the TID. We don't use the optional transition time or delay fields.
  // 4 additional bytes for the TransMIC
  NET_BUF_SIMPLE_DEFINE(msg, 2 + 5 + 4);
  struct bt_mesh_msg_ctx ctx = {
		.net_idx = net_idx,
		.app_idx = app_idx,
		.addr = target,
		.send_ttl = BT_MESH_TTL_DEFAULT,
  };
  bt_mesh_model_msg_init(&msg, opcode);
  net_buf_simple_add_le32(&msg, delta_level);
  net_buf_simple_add_u8(&msg, tid);
  tid++;
  if (bt_mesh_model_send(&sig_models[3], &ctx, &msg, NULL, NULL))
  {
  	printk("Unable to send generic delta set (ack/unack) message\n");
  }
  printk("delta set (ack/unack) message %d sent\n",delta_level);
}

void generic_move_set(s16_t delta_level, u8_t transition_time, u8_t delay, u32_t opcode) 
{
  // 2 bytes for the opcode
  // 5 bytes of parameters / payload : 16 bits delta level, 
  // 8 bits TID, 8 bits transition time, 8 bits delay.
  // 4 additional bytes for the TransMIC
  struct bt_mesh_msg_ctx ctx = {
		.net_idx = net_idx,
		.app_idx = app_idx,
		.addr = target,
		.send_ttl = BT_MESH_TTL_DEFAULT,
  };
  if (transition_time > 0) {
    NET_BUF_SIMPLE_DEFINE(msg, 2 + 5 + 4);
    bt_mesh_model_msg_init(&msg, opcode);
    net_buf_simple_add_le16(&msg, delta_level);
    net_buf_simple_add_u8(&msg, tid);
    net_buf_simple_add_u8(&msg, transition_time);
    net_buf_simple_add_u8(&msg, delay);
    if (bt_mesh_model_send(&sig_models[3], &ctx, &msg, NULL, NULL))
    {
      printk("Unable to send generic move set (ack/unack) message\n");
    }
  } else {
    NET_BUF_SIMPLE_DEFINE(msg, 2 + 3 + 4);  
    bt_mesh_model_msg_init(&msg, opcode);
    net_buf_simple_add_le16(&msg, delta_level);
    net_buf_simple_add_u8(&msg, tid);
    if (bt_mesh_model_send(&sig_models[3], &ctx, &msg, NULL, NULL))
    {
      printk("Unable to send generic move set (ack/unack) message\n");
    }
  }
  tid++;
  printk("move set (ack/unack) message %d sent\n",delta_level);
}

// -------------------------------------------------------------------------------------------------------
// Self Provisioning
// -----------------
static int selfProvision(void) 
{
	// now we provision ourselves... this is not how it would normally be done!
	int err = bt_mesh_provision(net_key, net_idx, flags, iv_index, node_addr, dev_key);
	if (err)
	{
		printk("Provisioning failed (err %d)\n", err);
		return err;
	}
	printk("Provisioning completed\n");

  return 0;
}


// -------------------------------------------------------------------------------------------------------
// Self Configuring
// ----------------

static int selfConfigure(void)
{
	int err;
	tid = 0;
	printk("self-configuring...\n");

	/* Add Application Key */
	err = bt_mesh_cfg_app_key_add(net_idx, node_addr, net_idx, app_idx, app_key, NULL);
	if (err)
	{
		printk("ERROR adding appkey (err %d)\n", err);
		return err;
	}
	else
	{
		printk("added appkey\n");
	}

	/* Bind to Health model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, node_addr, node_addr, app_idx, BT_MESH_MODEL_ID_HEALTH_SRV, NULL);
	if (err)
	{
		printk("ERROR binding to health server model (err %d)\n", err);
		return err;
	}
	else
	{
		printk("bound appkey to health server model\n");
	}

	/* Bind to level client model */
	err = bt_mesh_cfg_mod_app_bind(net_idx, node_addr, node_addr, app_idx, BT_MESH_MODEL_ID_GEN_LEVEL_CLI, NULL);
	if (err)
	{
		printk("ERROR binding to level client model (err %d)\n", err);
		return err;
	}
	else
	{
		printk("bound appkey to level client model\n");
	}

	printk("self-configuration complete\n");
	return 0;
}

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
  // set unack random
  // s16_t level = get_random_level();
  // generic_level_set(level,BT_MESH_MODEL_OP_GEN_LEVEL_SET_UNACK);

  // set random
  // s16_t level = get_random_level();
  // generic_level_set(level,BT_MESH_MODEL_OP_GEN_LEVEL_SET);

  // delta set positive
  // generic_delta_set(mapDimmerSettingToDelta(1),BT_MESH_MODEL_OP_GEN_DELTA_SET); 	

	// delta set unack positive
  // generic_delta_set(mapDimmerSettingToDelta(1),BT_MESH_MODEL_OP_GEN_DELTA_SET_UNACK); 	

  // move set positive
  // delta level of 32767 to be transitioned in 2.5 seconds
  // with a 1 second delay (200x5ms) 
  // transition time is 0b011001 (steps) then 
  // 0b00 (resolution=100ms) = 0b00011001 = decimal 25
  generic_move_set(MAX_DELTA_LEVEL_16, 25, 200, BT_MESH_MODEL_OP_GEN_MOVE_SET); 	

  // move set unack positive
//  generic_move_set(MAX_DELTA_LEVEL_16, 25, 200, BT_MESH_MODEL_OP_GEN_MOVE_SET_UNACK); 	
    // generic_move_set(16383, 25, 200, BT_MESH_MODEL_OP_GEN_MOVE_SET_UNACK); 	

  // move set unack positive with no transition time or delay
//  generic_move_set(MAX_DELTA_LEVEL_16, 0, 0, BT_MESH_MODEL_OP_GEN_MOVE_SET_UNACK); 	
  
}

void buttonB_work_handler(struct k_work *work)
{
	printk("buttonB_work_handler\n");
  // generic_level_get();

  // delta set negative
  // generic_delta_set(mapDimmerSettingToDelta(-1),BT_MESH_MODEL_OP_GEN_DELTA_SET);

  // delta set unack negative
  // generic_delta_set(mapDimmerSettingToDelta(-1),BT_MESH_MODEL_OP_GEN_DELTA_SET_UNACK); 	

  // move set negative
  // delta level of -32767 to be transitioned in 2.5 seconds
  // with a 1 second delay (200x5ms) 
  // transition time is 0b011001 (steps) then 
  // 0b00 (resolution=100ms) = 0b00011001 = decimal 25
  // generic_move_set(-1 * MAX_DELTA_LEVEL_16, 25, 200, BT_MESH_MODEL_OP_GEN_MOVE_SET);

  // move set unack negative
  // generic_move_set(-1 * MAX_DELTA_LEVEL_16, 25, 200, BT_MESH_MODEL_OP_GEN_MOVE_SET_UNACK); 

  // move set unack with zero delta level to stop a continuously running move on the light
  generic_move_set(0, 25, 200, BT_MESH_MODEL_OP_GEN_MOVE_SET_UNACK); 
  
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


// -------------------
// mesh initialisation
// -------------------

static void bt_ready(int err)
{
  if (err)
  {
    printk("bt_enable init failed with err %d\n", err);
    return;
  }
  printk("Bluetooth initialised OK\n");
  err = bt_mesh_init(&prov, &comp);
  if (err)
  {
    printk("bt_mesh_init failed with err %d\n", err);
    return;
  }
  printk("Mesh initialised OK\n");

  err = selfProvision();

  selfConfigure();
  printk("provisioned, configured and ready\n");

}

void main(void)
{
  int err;
  printk("dimmer\n");
  configureButtons();

  err = bt_enable(bt_ready);
  if (err)
  {
    printk("bt_enable failed with err %d\n", err);
  }
}
