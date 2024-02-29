
// From https://github.com/buxtronix/arduino/blob/master/libraries/Rotary/Rotary.cpp

enum
{   R_START = 0,
	R_CW_FINAL,
	R_CW_BEGIN,
	R_CW_NEXT,
	R_CCW_BEGIN,
	R_CCW_FINAL,
	R_CCW_NEXT,

	CW_STEP = 0x10,
	CCW_STEP = 0x20
};

constexpr char ttable[7][4] =
{   // R_START
	{R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
	// R_CW_FINAL
	{R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | CW_STEP},
	// R_CW_BEGIN
	{R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
	// R_CW_NEXT
	{R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
	// R_CCW_BEGIN
	{R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
	// R_CCW_FINAL
	{R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | CCW_STEP},
	// R_CCW_NEXT
	{R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

class Rotary_encoder
{
	static char state;
	static char pin1;
	static char pin2;

	static int process()
	{   // Grab state of input pins.
		char state_1 = gpio_get(pin1);
		char state_2 = gpio_get(pin2);

		char pinstate = (state_2 << 1) | state_1;

		// Determine new state from the pins and state table.
		state = ttable[state & 0xF][pinstate];

		if(state & CW_STEP) return 1;
		if(state & CCW_STEP) return -1;
		return 0;
	}

public:
	static int value;

	static void initialize(char pin1_, char pin2_)
	{
		pin1 = pin1_;
		pin2 = pin2_;

		state = R_START;
		value = 0;

		gpio_init(pin1); gpio_set_dir(pin1, GPIO_IN); gpio_pull_up(pin1);
		gpio_init(pin2); gpio_set_dir(pin2, GPIO_IN); gpio_pull_up(pin2);

		auto change = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
		gpio_set_irq_enabled_with_callback(pin1, change, true, &Rotary_encoder::isr);
		gpio_set_irq_enabled(pin2, change, true);
	}

	static void isr(uint gpio, uint32_t events)
	{
		value += process();
	}
};

int Rotary_encoder::value = 0;
char Rotary_encoder::pin1;
char Rotary_encoder::pin2;
char Rotary_encoder::state;
