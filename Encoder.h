
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
	char state;
	char pin1;
	char pin2;

public:
	Rotary_encoder(char pin1_, char pin2_)
	{
		pin1 = pin1_;
		pin2 = pin2_;

		gpio_init(pin1); gpio_set_dir(pin1, GPIO_IN); gpio_pull_up(pin1);
		gpio_init(pin2); gpio_set_dir(pin2, GPIO_IN); gpio_pull_up(pin2);

		state = R_START;
	}

	int process()
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
};
