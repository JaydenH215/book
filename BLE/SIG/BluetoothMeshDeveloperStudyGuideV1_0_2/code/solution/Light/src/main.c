/**
 * Light Node - behaviour predominantly based on the Bluetooth mesh Generic OnOff Server and Generic Level Server Models
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
u8_t onoff_state;
u8_t target_onoff_state;

s16_t level_state;
s16_t target_level_state;

u8_t transition_in_progress = 0;
u8_t transition_stop_requested = 0;
s64_t transition_start_timestamp;

u8_t transition_time = 0;
u32_t total_transition_duration = 0;

u8_t delay = 0;
s32_t delta_level = 0;
// initial level value to be adjusted by received delta values in a transaction
s16_t initial_trans_level;
u8_t last_delta_tid;

u8_t delta_level_as_led_change_count = 0; // number of LED lights changing over the specified delta level in a move set message
u8_t count_led_increments = 0;						// number of LED lights switched on/off by the transition so far
u32_t led_interval = 0;										// time between each LED being switch on/off during a transition
s16_t one_led_level_increment = 0;				// amount to add to the level state to switch on/off one extra LED. +/- the level_one_step_value
u8_t steps_multiplier = 0;
u8_t resolution = 0;
u16_t level_one_step_value = 0;

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
						 {1, 1, 1, 1, 1})};

// device UUID
// cfa0ea7e-17d9-11e8-86d1-5f1ce28adea7
static const uint8_t dev_uuid[16] = {0xcf, 0xa0, 0xea, 0x7e, 0x17, 0xd9, 0x11, 0xe8, 0x86, 0xd1, 0x5f, 0x1c, 0xe2, 0x8a, 0xde, 0xa7};
static const struct bt_mesh_prov prov = {
		.uuid = dev_uuid,
};

void derive_transition_time_values(u8_t transition_time)
{
	resolution = (transition_time >> 6);
	steps_multiplier = transition_time & 0x3F;
	printk("steps=%d,resolution=%d\n", steps_multiplier, resolution);
	// The number of Steps = 6 bits : 0x01–0x3E
	// Resolution = 2 bits = time taken for each transition step (e.g. 0b00 means 100ms, 0b01 means 1 second)
	// so the elapsed transition time is Number of Steps * Resolution Meaning
	// Example
	// Delta Level = 1000
	// Steps = 100, Resolution = 100ms
	// After (delay * 5ms), every 100ms, level state is increased by (1000 / 100) = 10 until the complete level delta change has completed
	//calculate led_interval

	// how many LEDs will need to be switched on or off for this delta?
	delta_level_as_led_change_count = 0;
	if (delta_level != 0)
	{
		delta_level_as_led_change_count = (int)(abs(delta_level) / level_one_step_value) + 1;
		if (delta_level_as_led_change_count > 25)
		{
			delta_level_as_led_change_count = 25;
		}
	}

	count_led_increments = 0;
	printk("delta_level_as_led_change_count=%d delta_level=%d level_one_step_value=%d\n", delta_level_as_led_change_count, delta_level, level_one_step_value);

	total_transition_duration = 0;

	switch (resolution)
	{
	// 100ms
	case 0:
		total_transition_duration = steps_multiplier * 100;
		break;
	// 1 second
	case 1:
		total_transition_duration = steps_multiplier * 1000;
		break;
	// 10 seconds
	case 2:
		total_transition_duration = steps_multiplier * 10000;
		break;
	// 10 minutes
	case 3:
		total_transition_duration = steps_multiplier * 600000;
		break;
	}

	led_interval = total_transition_duration / delta_level_as_led_change_count;

	printk("transition time=%d to switch %d LEDs each taking %d ms\n", total_transition_duration, delta_level_as_led_change_count, led_interval);

	// only used by level operations. not relevant to onoff.
	one_led_level_increment = delta_level / delta_level_as_led_change_count;
	printk("delta_level=%d delta in terms of led increments=%d one led level increment duration=%d\n", delta_level, delta_level_as_led_change_count, one_led_level_increment);
}

// -------------------------------------------------------------------------------------------------------
// Generic OnOff Server
// --------------------

// message opcodes
#define BT_MESH_MODEL_OP_gen_onoff_get BT_MESH_MODEL_OP_2(0x82, 0x01)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET BT_MESH_MODEL_OP_2(0x82, 0x02)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x03)
#define BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS BT_MESH_MODEL_OP_2(0x82, 0x04)

// need to forward declare as we have circular dependencies
void generic_onoff_status(u8_t on_or_off, u16_t dest_addr, u8_t transitioning, u8_t target_on_or_off, u8_t remaining_time);

// generic onoff worker

void onoff_work_handler(struct k_work *work)
{
	s64_t now = k_uptime_get();

	if (work == &onoff_set_work)
	{
		printk("%lld> handling an onoff set work item\n", now);
		// we're not implemented timed transitions otherwise we might pause here to handle the delay field
		now = k_uptime_get();
		printk("%lld> transitioning onoff state to %d\n", now, target_onoff_state);

		onoff_state = target_onoff_state;
		transition_in_progress = 0;

		struct mb_display *disp = mb_display_get();
		mb_display_image(disp, MB_DISPLAY_MODE_DEFAULT, K_FOREVER, &led_patterns[(onoff_state * 25)], 1);
		return;
	}

	if (work == &onoff_status_work)
	{
		printk("%lld> sending an onoff status work item to %d\n", now, remote_addr);
		generic_onoff_status(onoff_state, remote_addr, 0, 0, 0);
		return;
	}
}

static void set_onoff_state(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	u8_t buflen = buf->len;
	target_onoff_state = net_buf_simple_pull_u8(buf);
	u8_t tid = net_buf_simple_pull_u8(buf);
	printk("set onoff state: onoff=%u TID=%u\n", target_onoff_state, tid);
	transition_time = 0;

	if (buflen > 2)
	{
		printk("message contains transition_time field - processing not implemented");
	}

	delay = 0;
	if (buflen > 3)
	{
		printk("message contains delay field - processing not implemented");
	}

	s64_t now = k_uptime_get();
	printk("%lld> starting worker\n", now);
	k_work_submit(&onoff_set_work);
}

static u8_t calculate_remaining_time()
{
	if (transition_in_progress == 0)
	{
		return 0;
	}
	s64_t now = k_uptime_get();
	u32_t duration_remainder = total_transition_duration - (now - transition_start_timestamp);
	u8_t steps = 0;
	u8_t resolution = 0;
	if (duration_remainder > 620000)
	{
		// > 620 seconds -> resolution=0b11 [10 minutes]
		resolution = 0x03;
		steps = duration_remainder / 600000;
	}
	else if (duration_remainder > 62000)
	{
		// > 62 seconds -> resolution=0b10 [10 seconds]
		resolution = 0x02;
		steps = duration_remainder / 10000;
	}
	else if (duration_remainder > 6200)
	{
		// > 6.2 seconds -> resolution=0b01 [1 seconds]
		resolution = 0x01;
		steps = duration_remainder / 1000;
	}
	else
	{
		// <= 6.2 seconds -> resolution=0b00 [100 ms]
		resolution = 0x00;
		steps = duration_remainder / 100;
	}
	printk("calculated steps=%d,resolution=%d\n", steps, resolution);
	return ((resolution << 6) | steps);
}

static void generic_onoff_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("gen_onoff_get\n");
	generic_onoff_status(onoff_state, ctx->addr, transition_in_progress, target_onoff_state,
											 calculate_remaining_time());
}

static void generic_onoff_set(struct bt_mesh_model *model,
															struct bt_mesh_msg_ctx *ctx,
															struct net_buf_simple *buf)
{
	remote_addr = ctx->addr;
	set_onoff_state(model, ctx, buf);
	//respond with STATUS message
	k_work_submit(&onoff_status_work);
}

static void generic_onoff_set_unack(struct bt_mesh_model *model,
																		struct bt_mesh_msg_ctx *ctx,
																		struct net_buf_simple *buf)
{
	remote_addr = 0;
	set_onoff_state(model, ctx, buf);
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

// -------------------------------------------------------------------------------------------------------
// Generic Level Server
// --------------------

// message opcodes
#define BT_MESH_MODEL_OP_GENERIC_LEVEL_GET BT_MESH_MODEL_OP_2(0x82, 0x05)
#define BT_MESH_MODEL_OP_GENERIC_LEVEL_SET BT_MESH_MODEL_OP_2(0x82, 0x06)
#define BT_MESH_MODEL_OP_GENERIC_LEVEL_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x07)
#define BT_MESH_MODEL_OP_GENERIC_LEVEL_STATUS BT_MESH_MODEL_OP_2(0x82, 0x08)

#define BT_MESH_MODEL_OP_GENERIC_DELTA_SET BT_MESH_MODEL_OP_2(0x82, 0x09)
#define BT_MESH_MODEL_OP_GENERIC_DELTA_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x0A)

#define BT_MESH_MODEL_OP_GENERIC_MOVE_SET BT_MESH_MODEL_OP_2(0x82, 0x0B)
#define BT_MESH_MODEL_OP_GENERIC_MOVE_SET_UNACK BT_MESH_MODEL_OP_2(0x82, 0x0C)

// need to forward declare as we have circular dependencies
void generic_level_status(u16_t dest_addr);

static void set_level_state(s16_t level)
{
	struct mb_display *disp = mb_display_get();
	level_state = level;
	u8_t level_inx = 0;
	if (level_state > 0)
	{
		level_inx = (u8_t)(level_state / level_one_step_value) + 1;
		if (level_inx > 25)
		{
			level_inx = 25;
		}
	}
	printk("level_state=%d level_inx=%d\n", level_state, level_inx);
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
	s64_t now = k_uptime_get();
	if (work == &level_set_work)
	{
		printk("%lld> handling an level set work item\n", now);
		printk("sleeping: delay %d ms\n", (delay * 5));
		k_sleep(delay * 5);
		now = k_uptime_get();
		printk("%lld> transitioning level state to %d\n", now, target_level_state);
		k_work_submit(&level_transition_work);
		return;
	}

	if (work == &level_status_work)
	{
		printk("%lld> sending a level status work item to %d\n", now, remote_addr);
		generic_level_status(remote_addr);
		return;
	}
}

void level_transition_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&level_transition_work);
}

static void generic_level_set_common(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("generic_level_set_common\n");
	u8_t buflen = buf->len;
	// Level(2), TID(1), Transition Time(optional, 1), Delay (conditional, 1)
	target_level_state = (s16_t)net_buf_simple_pull_le16(buf);

	// The TID field is a transaction identifier indicating whether the message is a new message or a retransmission of a previously sent message
	u8_t tid = net_buf_simple_pull_u8(buf);

	// set the Generic Level state to the Level field of the message, unless the message has the same values for the SRC, DST, and TID fields as the
	// previous message received within the last 6 seconds.

	s64_t now = k_uptime_get(); // elapsed time since the system booted, in milliseconds.
	if (ctx->addr == last_message_src && ctx->recv_dst == last_message_dst && tid == last_message_tid && (now - last_message_timestamp <= 6000))
	{
		printk("Ignoring message - same transaction during 6 second window\n");
		return;
	}

	last_message_timestamp = now;
	last_message_src = ctx->addr;
	last_message_dst = ctx->recv_dst;
	last_message_tid = tid;

	transition_time = 0;
	if (buflen > 3)
	{
		printk("message contains transition_time field - processing not implemented");
	}
	delay = 0;
	if (buflen > 4)
	{
		printk("message contains delay field - processing not implemented");
	}
	set_level_state(target_level_state);
	if (remote_addr != 0)
	{
		k_work_submit(&level_status_work);
	}
}

static void generic_level_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	remote_addr = ctx->addr;
	generic_level_set_common(model, ctx, buf);
}

static void generic_level_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("generic_level_set_unack\n");
	remote_addr = 0;
	generic_level_set_common(model, ctx, buf);
}

static void generic_delta_set_common(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	// Delta(4), TID(1), Transition Time(optional, 1), Delay (conditional, 1)
	u8_t buflen = buf->len;
	delta_level = (s32_t)net_buf_simple_pull_le32(buf);
	u8_t tid_field = net_buf_simple_pull_u8(buf);
	if (tid_field != last_delta_tid)
	{
		// new transaction
		initial_trans_level = level_state;
	}

	s64_t now = k_uptime_get(); // elapsed time since the system booted, in milliseconds.

	transition_time = 0;
	if (buflen > 5)
	{
		printk("message contains transition_time field - processing not implemented");
	}
	delay = 0;
	if (buflen > 6)
	{
		printk("message contains delay field - processing not implemented");
	}
	if (ctx->addr == last_message_src && ctx->recv_dst == last_message_dst && tid_field == last_message_tid && (now - last_message_timestamp <= 6000))
	{
		printk("Ignoring message - same transaction during 6 second window\n");
		return;
	}
	last_message_timestamp = now;
	last_message_src = ctx->addr;
	last_message_dst = ctx->recv_dst;
	last_message_tid = tid_field;

	s32_t new_level = level_state + delta_level;
	if (new_level < 0)
	{
		new_level = 0;
	}
	else if (new_level > MAX_LEVEL)
	{
		new_level = MAX_LEVEL;
	}
	target_level_state = new_level;

	set_level_state(target_level_state);
	if (remote_addr != 0)
	{
		k_work_submit(&level_status_work);
	}
}

static void generic_delta_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	remote_addr = ctx->addr;
	generic_delta_set_common(model, ctx, buf);
}

static void generic_delta_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	remote_addr = 0;
	generic_delta_set_common(model, ctx, buf);
}

void start_level_transition_timer()
{
	transition_in_progress = 1;
	transition_start_timestamp = k_uptime_get();
	s64_t now = k_uptime_get();
	printk("%lld> k_timer_start called\n", now);
	k_timer_start(&level_transition_timer, K_MSEC(5 * delay), K_MSEC(led_interval));
}

static int generic_move_set_common(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("generic_move_set_common\n");
	u8_t buflen = buf->len;
	// Delta(2), TID(1), Transition Time(optional, 1), Delay (conditional, 1)
	delta_level = (s16_t)net_buf_simple_pull_le16(buf);
	u8_t tid_field = net_buf_simple_pull_u8(buf);
	if (tid_field != last_delta_tid)
	{
		// new transaction
		initial_trans_level = level_state;
		target_level_state = level_state + delta_level;
	}

	printk("buflen=%d delta_level=%d tid=%d\n", buflen, delta_level, tid_field);
	s64_t now = k_uptime_get(); // elapsed time since the system booted, in milliseconds.
	if (ctx->addr == last_message_src && ctx->recv_dst == last_message_dst && tid_field == last_message_tid && (now - last_message_timestamp <= 6000))
	{
		printk("Ignoring message - same transaction during 6 second window\n");
		return 1;
	}

	last_message_timestamp = now;
	last_message_src = ctx->addr;
	last_message_dst = ctx->recv_dst;
	last_message_tid = tid_field;
	transition_time = 0;
	steps_multiplier = 0;

	printk("delta_level=%d transition_in_progress=%d\n", delta_level, transition_in_progress);
	// transition_in_progress will already have been set to 0 so we don't test it for being equal to 1 here
	if (delta_level == 0)
	{
		printk("generic move set has requested running transition be stopped\n");
		if (k_timer_remaining_get(&level_transition_timer) == 0)
		{
			printk("stopping timer\n");
			k_timer_stop(&level_transition_timer);
		}
		// prevent the timer from being started for this case
		return 1;
	}

	if (buflen > 4)
	{
		transition_time = net_buf_simple_pull_u8(buf);
		delay = net_buf_simple_pull_u8(buf);
		printk("transition_time=%d\n", transition_time);
		printk("delay=%d\n", delay);
		derive_transition_time_values(transition_time);
		return 0;
	}
	else
	{
		// we do not support the Generic Default Transition Time state and therefore require both the transition time field and the delay field
		// Consequently, per the spec, we shall not initiate any Generic Level state change
		printk("Ignoring message - transition time and delay are absent\n");
		return 1;
	}
}

static void generic_move_set(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	if (transition_in_progress == 1)
	{
		k_timer_stop(&level_transition_timer);
		transition_in_progress = 0;
	}

	remote_addr = ctx->addr;
	// ref 3.3.2.2.4 Upon receiving a Generic Move Set message, the Generic Level Server shall respond
	// with a Generic Level Status message (see Section 3.3.2.2.5). The target Generic Level state is
	// the upper limit of the Generic Level state when the transition speed is positive, or the lower
	// limit of the Generic Level state when the transition speed is negative.

	// extract fields from message
	int err = generic_move_set_common(model, ctx, buf);
	if (err)
	{
		return;
	}
	// save the real target_level_state from the message
	s16_t msg_target_level_state = target_level_state;
	// set the target level state to the max/min value for the status message
	if (delta_level >= 0)
	{
		target_level_state = 32767;
	}
	else
	{
		target_level_state = -32768;
	}
	// send the status message
	generic_level_status(remote_addr);
	// restore the target level value
	target_level_state = msg_target_level_state;
	// kick off timer to control transition process
	transition_in_progress = 1;
	start_level_transition_timer();
}

static void generic_move_set_unack(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	if (transition_in_progress == 1)
	{
		k_timer_stop(&level_transition_timer);
		transition_in_progress = 0;
	}
	remote_addr = 0;
	// extract fields from message
	int err = generic_move_set_common(model, ctx, buf);
	if (err)
	{
		return;
	}

	// kick off timer to control transition process
	transition_in_progress = 1;
	start_level_transition_timer();
}

static void generic_level_get(struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf)
{
	printk("generic_level_get\n");
	remote_addr = ctx->addr;
	generic_level_status(remote_addr);
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

// ----------------------------------------------------------------------------------------------------
// generic level status TX message producer

// Configuration Client Model
// -------------------------------------------------------------------------------------------------------
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
// generic onoff status TX message producer

void generic_onoff_status(u8_t present_on_or_off, u16_t dest_addr, u8_t transitioning, u8_t target_on_or_off, u8_t remaining_time)
{
	// 2 bytes for the opcode
	// 1 bytes parameters: present onoff value
	// 2 optional bytes for target onoff and remaining time
	// 4 additional bytes for the TransMIC

	struct bt_mesh_msg_ctx ctx = {
			.net_idx = net_idx,
			.app_idx = app_idx,
			.addr = dest_addr,
			.send_ttl = BT_MESH_TTL_DEFAULT,
	};

	u8_t buflen = 7;
	if (transitioning == 1)
	{
		buflen = 9;
	}

	NET_BUF_SIMPLE_DEFINE(msg, buflen);

	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GENERIC_ONOFF_STATUS);
	net_buf_simple_add_u8(&msg, present_on_or_off);

	if (transitioning == 1)
	{
		net_buf_simple_add_u8(&msg, target_on_or_off);
		net_buf_simple_add_u8(&msg, remaining_time);
	}

	if (bt_mesh_model_send(&sig_models[3], &ctx, &msg, NULL, NULL))
	{
		printk("Unable to send generic onoff status message\n");
	}

	printk("onoff status message %d sent\n", present_on_or_off);
}

// ----------------------------------------------------------------------------------------------------
// generic level status message producer
void generic_level_status(u16_t dest_addr)
{

	struct bt_mesh_msg_ctx ctx = {
			.net_idx = net_idx,
			.app_idx = app_idx,
			.addr = dest_addr,
			.send_ttl = BT_MESH_TTL_DEFAULT,
	};
	u8_t params_len = 2;
	printk("generic_level_status transition_in_progress=%d\n", transition_in_progress);
	if (transition_in_progress == 1)
	{
		params_len = 5;
	}

	NET_BUF_SIMPLE_DEFINE(msg, 2 + params_len + 4);
	bt_mesh_model_msg_init(&msg, BT_MESH_MODEL_OP_GENERIC_LEVEL_STATUS);

	printk("generic_level_status transition_in_progress=%d\n", transition_in_progress);
	if (transition_in_progress == 0)
	{
		// 2 bytes for the opcode
		// 2 bytes parameters: present level value
		// 4 additional bytes for the TransMIC
		net_buf_simple_add_le16(&msg, level_state);
	}
	else
	{
		// 2 bytes for the opcode
		// 5 bytes parameters: present level value, target level value, remaining time
		// 4 additional bytes for the TransMIC
		net_buf_simple_add_le16(&msg, level_state);
		net_buf_simple_add_le16(&msg, target_level_state);
		net_buf_simple_add_u8(&msg, calculate_remaining_time());
	}

	if (bt_mesh_model_send(&sig_models[4], &ctx, &msg, NULL, NULL))
	{
		printk("Unable to send generic level status message\n");
	}

	s64_t now = k_uptime_get();
	printk("%lld> level status message %d sent\n", now, level_state);
}

// -------------------------------------------------------------------------------------------------------
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
	if (err)
	{
		printk("ERROR: SELF-PROVISIONING FAILED");
	}
	else
	{
		printk("self-provisioned OK\n");
	}

	err = selfConfigure();

	if (err)
	{
		printk("ERROR: INITIALISATION FAILED");
	}
	else
	{
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
	printk("light node %d\n", addr);
	level_one_step_value = MAX_LEVEL / 25;

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
