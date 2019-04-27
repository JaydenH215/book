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
#define NODE_ADDR 0x0001
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

// 4.3.1.1 Key indices
static const u16_t net_idx;
static const u16_t app_idx;
// 3.8.4 IV Index
static const u32_t iv_index;

// other 

static u8_t flags;
static u8_t tid;

// device UUID
// cfa0ea7e-17d9-11e8-86d1-5f1ce28adea1
static const uint8_t dev_uuid[16] = {0xcf, 0xa0, 0xea, 0x7e, 0x17, 0xd9, 0x11, 0xe8, 0x86, 0xd1, 0x5f, 0x1c, 0xe2, 0x8a, 0xde, 0xa1};

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
// Generic OnOff Client Model
// --------------------------

u8_t onoff[] = {
	0,
	1};

// handler functions for this model's RX messages

static void gen_onoff_status(struct bt_mesh_model *model,
														struct bt_mesh_msg_ctx *ctx,
														struct net_buf_simple *buf)
{
	printk("gen_onoff_status");
	u8_t onoff_state = net_buf_simple_pull_u8(buf);
	printk("gen_onoff_status onoff=%d\n",onoff_state);
	struct mb_display *disp = mb_display_get();
  mb_display_print(disp, MB_DISPLAY_MODE_SINGLE, K_SECONDS(1), "%d", onoff_state);
}

// message types defined by this model.

#define BT_MESH_MODEL_OP_GENERIC_ONOFF_GET	      BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET	      BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK	BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS   	BT_MESH_MODEL_OP_2(0x82, 0x04)

static const struct bt_mesh_model_op gen_onoff_cli_op[] = {
		{BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS, 1, gen_onoff_status},
		BT_MESH_MODEL_OP_END,
};


// -------------------------------------------------------------------------------------------------------
// Composition
// -----------
static struct bt_mesh_model sig_models[] = {
				BT_MESH_MODEL_CFG_SRV(&cfg_srv),
				BT_MESH_MODEL_CFG_CLI(&cfg_cli),
				BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
				BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_onoff_cli_op,
																		NULL, &onoff[0]),
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


// Generic OnOff Client - TX message producer functions
// -----------------------------------------------------------

void genericOnOffGet() 
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

	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GENERIC_ONOFF_GET);

	if (bt_mesh_model_send(&sig_models[2], &ctx, &msg, NULL, NULL))
	{
		printk("Unable to send generic onoff get message\n");
	}

	printk("onoff get message sent\n");
}

int sendGenOnOffSet(u8_t on_or_off, u16_t message_type) 
{
  // 2 bytes for the opcode
	// 2 bytes parameters: first is the onoff value, second is for the TID
	// 4 additional bytes for the TransMIC
	
	NET_BUF_SIMPLE_DEFINE(msg, 2 + 2 + 4);

	struct bt_mesh_msg_ctx ctx = {
			.net_idx = net_idx,
			.app_idx = app_idx,
			.addr = target,
			.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	bt_mesh_model_msg_init(&msg, message_type);
	net_buf_simple_add_u8(&msg, on_or_off);
	net_buf_simple_add_u8(&msg, tid);
	tid++;

	return bt_mesh_model_send(&sig_models[2], &ctx, &msg, NULL, NULL);
}

void genericOnOffSetUnAck(u8_t on_or_off) 
{
	if (sendGenOnOffSet(on_or_off, BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK))
	{
		printk("Unable to send generic onoff set unack message\n");
	}
	printk("onoff set unack message %d sent\n",on_or_off);
}


void genericOnOffSet(u8_t on_or_off) 
{
	if (sendGenOnOffSet(on_or_off, BT_MESH_MODEL_OP_GENERIC_ONOFF_SET))
	{
		printk("Unable to send generic onoff set unack message\n");
	}
	printk("onoff set unack message %d sent\n",on_or_off);
}

void buttonA_work_handler(struct k_work *work)
{
	printk("Button A work handler\n");
  genericOnOffSetUnAck(onoff[1]);
	// genericOnOffSet(onoff[1]);
}

void buttonB_work_handler(struct k_work *work)
{
	printk("Button B work handler\n");
  genericOnOffSetUnAck(onoff[0]);
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


// -------------------------------------------------------------------------------------------------------
// Self Configuring
// ----------------

static void selfConfigure(void)
{
		int err;

		printk("self-configuring...\n");

		/* Add Application Key */
		bt_mesh_cfg_app_key_add(net_idx, node_addr, net_idx, app_idx, app_key, NULL);

		/* Bind to generic onoff client model */
		err = bt_mesh_cfg_mod_app_bind(net_idx, node_addr, node_addr, app_idx, BT_MESH_MODEL_ID_GEN_ONOFF_CLI, NULL);
		if (err)
		{
				printk("ERROR binding to generic onoff client model (err %d)\n", err);
				return;
		}
		else
		{
				printk("bound appkey to generic onoff server model\n");
		}

		/* Bind to Health model */
		bt_mesh_cfg_mod_app_bind(net_idx, node_addr, node_addr, app_idx, BT_MESH_MODEL_ID_HEALTH_SRV, NULL);

		printk("self-configuration complete\n");
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

static void bt_ready(int err)
{
		if (err)
		{
				printk("bt_enable init failed with err %d\n", err);
				return;
		}

		printk("Bluetooth initialised OK\n");

		// prov is a bt_mesh_prov struct.
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
		printk("switch\n");

		configureButtons();

		err = bt_enable(bt_ready);
		if (err)
		{
				printk("bt_enable failed with err %d\n", err);
		}
}
