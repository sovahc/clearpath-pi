// 
// https://github.com/martinkooij/pi-pico-LCD

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <vector>
using std::vector;
using std::byte;

#include "bsp/board.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"

#include "tusb.h"

#include "pi-pico-LCD/lcd_display/lcd_display.hpp"

constexpr int ENCODER_PIN_A = 2;
constexpr int ENCODER_PIN_B = 3;
constexpr int ENCODER_BUTTON_PIN = 4;

constexpr int LCD_D4 = 8;
constexpr int LCD_D5 = 9;
constexpr int LCD_D6 = 10;
constexpr int LCD_D7 = 11;
constexpr int LCD_RS = 12;
constexpr int LCD_E  = 13;

//LCDdisplay lcd(LCD_D4, LCD_D5, LCD_D6, LCD_D7, LCD_RS, LCD_E, 8, 2);

//

template<bool enabled> class X_log
{
	public:

	void operator ()(const char* str, ...)
	{	if(!enabled) return;
	
		va_list va; 
		va_start(va, str);
	
		vprintf(str, va);
	
		va_end(va);
	}
};

X_log<true> LOG;

#include "Encoder.h"

//

template<bool enabled> class X_byte_logger
{
public:

	void operator ()(void* buf, size_t count)
	{	if(!enabled) return;

		for(auto i = 0; i < count; ++i) printf(" %02X", ((byte*)buf)[i]);
	}

	void operator ()(const char* prefix, const vector<byte>& array)
	{	if(!enabled) return;

		printf("%s", prefix);

		for(auto b : array) printf(" %02X", b);

		printf("\n");
	}

	void hex(const char* prefix, const char* hex)
	{	if(!enabled) return;

		printf("%s %s\n", prefix, hex);
	}

};

X_byte_logger<true> DUMP_BYTES;

class X_rare_error_tracking
{
	// TODO? https://github.com/earlephilhower/arduino-pico/tree/master/libraries/EEPROM
	// TODO? https://github.com/fabriziop/EEWL
	static int errors[];
	static const int TOTAL_LINES;

public:
	void clear()
	{	memset(&errors, 0, TOTAL_LINES * sizeof(errors[0]));
	}

	X_rare_error_tracking()
	{	clear();
	}

	void operator () (unsigned line)
	{
		LOG("\nE%d ", line);
	
		++errors[line];
	}
};

X_rare_error_tracking ERROR_AT;

//

class LED_BLINKING
{
	static inline volatile uint32_t last_blink;
public:
	static void blink()
	{	last_blink = board_millis();
		board_led_write(1);
	}

	static void task()
	{	if(board_millis() - last_blink < 25) return;
		board_led_write(0);
	}
};

//

void convert_7_to_8_bit(const vector<byte>& input, vector<byte>& output, int integer_size)
{
	if(input.empty()) return;

    int acc = 0;
    int bitCount = 0;

	byte filler = byte(0);
	
	if(integer_size != 0)
	{	bool last_bit = byte(0) != (input.back() & byte(0x40));
		filler = byte(last_bit ? 0xFF : 0);
	}

    for (auto b : input)
    {
        acc |= int(b) << bitCount;
        bitCount += 7;

        if (bitCount >= 8)
        {
            output.push_back(byte(acc));
            acc >>= 8;
            bitCount -= 8;
        }
    }

    if (bitCount > 0)
	{	
		acc |= int(filler) << bitCount;

		output.push_back(byte(acc));
	}

	while(output.size() < integer_size)
	{	output.push_back(filler);
	}
}

void convert_8_to_7_bit(const vector<byte>& input, vector<byte>& output)
{
    int acc = 0;
    int bitCount = 0;

    for (auto b : input)
    {
        acc |= int(b) << bitCount;
        bitCount += 8;

        while (bitCount >= 7)
        {
            output.push_back(byte(acc & 0x7F));
            acc >>= 7;
            bitCount -= 7;
        }
    }

    if (bitCount > 0) output.push_back(byte(acc & 0x7F));
}

void int_to_vector(vector<byte>& data, int i)
{
	data.push_back(byte(i));
	data.push_back(byte(i >> 8));
	data.push_back(byte(i >> 16));
	data.push_back(byte(i >> 24));
}

int int_from_vector(vector<byte>& data)
{	if(data.size() < sizeof(int))
	{	ERROR_AT(__LINE__);
		return 0;
	}

	return int(data[0]) |
		(int(data[1]) << 8) |
		(int(data[2]) << 16) |
		(int(data[3]) << 24);
}

class Timeout
{
	uint32_t start;
	uint32_t end;

public:
	Timeout(int timeout_ms)
	{	start = board_millis();
		end = start + timeout_ms;
  	}

	int remaining_time()
	{	return end - board_millis();
	}
};

class Message
{
	vector<byte> _7bit;
	vector<byte> _8bit;

public:
	static void cdc_write(byte b)
	{
		if (!tuh_cdc_mounted(0) ) return ERROR_AT(__LINE__);

		tuh_cdc_write(0, &b, 1);

		LED_BLINKING::blink();
	}

	static void cdc_write(const vector<byte>& data)
	{
		if (!tuh_cdc_mounted(0) ) return ERROR_AT(__LINE__);

		tuh_cdc_write(0, data.data(), data.size());

		LED_BLINKING::blink();
	}

	static void cdc_flush()
	{
		if (!tuh_cdc_mounted(0) ) return ERROR_AT(__LINE__);

		tuh_cdc_write_flush(0);
	}
private:

	static byte checksum(const vector<byte>& data, size_t limit)
	{
		if(limit > data.size())
		{	ERROR_AT(__LINE__);
			limit = data.size();
		}

		int r = 0;

		for(auto i = 0; i < limit; ++i) r += int(data[i]);

		r = ((0xFF - r) + 1) & 0x7F;
		return byte(r);
	}

	bool is_full_message()
	{	if(_7bit.size() < 2) return false;

		int length = int(_7bit[1]);
		if(_7bit.size() < 2 + length + 1) return false;

		return true;
	}

	void strip_headers(vector<byte>& message)
	{	if(message.size() < 3) return ERROR_AT(__LINE__);

		std::move(message.begin() + 2, message.end(), message.begin()); // 0x80 and length
		message.resize(message.size() - 3);
	}

	void add_headers(vector<byte>& message, byte b1 = byte(0x80))
	{
		byte length = byte(message.size());

		message.insert(message.begin(), length);
		message.insert(message.begin(), b1); // first
		message.push_back(checksum(message, message.size()));
	}

public:

	void receive(vector<byte>& data, bool as_integer)
	{
		data.clear();

		Timeout timeout(3000);

		for(;;)
		{
			int r = timeout.remaining_time();
			if(r <= 0) return ERROR_AT(__LINE__);

			tuh_task_ext(r, false); // WARNING: tuh_task_ext doesn't actually use timeout_ms

			if(is_full_message()) break;
		}

		// Full message is received

		auto answer_end = 2 + int(_7bit[1]) + 1;
		auto received_checksum = _7bit[answer_end-1];
		auto calculated_checksum = checksum(_7bit, answer_end-1);

		if(calculated_checksum != received_checksum)
		{
			DUMP_BYTES("!CHECKSUM", _7bit);
			_7bit.clear();
			return ERROR_AT(__LINE__);
		}

		//DUMP_BYTES("R7", _7bit);

		strip_headers(_7bit);

		convert_7_to_8_bit(_7bit, data, as_integer ? sizeof(int) : 0);

		//DUMP_BYTES("R8", data);

		_7bit.clear();
	}

	void send_message(const vector<byte>& data)
	{
		//DUMP_BYTES("S8", data);

		_7bit.clear();
		convert_8_to_7_bit(data, _7bit);

		add_headers(_7bit);

		//DUMP_BYTES("S7", _7bit);

		cdc_write(_7bit);
		cdc_flush();
	}

public:

	void byte_received(byte b) // called from TUH callback
	{
		if(_7bit.empty() && (int(b) & 0x80) == 0) return; // synchronize with the start of the answer
	
		_7bit.push_back(b);
	}
};

//

class Clearpath_motor
{
	Message output_message;
	Message input_message;

	vector<byte> tmp;

	static byte hex_to_dec(char c)
	{
		if ('0' <= c && c <= '9') return byte(c - '0');
		else if ('a' <= c && c <= 'f') return byte(c - 'a' + 10);
		else if ('A' <= c && c <= 'F') return byte(c - 'A' + 10);
		else
		{	ERROR_AT(__LINE__);
			return byte(-1);
		}
	}

	static void hex_to_bytes(const char* hex, vector<byte>& output)
	{	// This function also accepts spaces in the input

		for (;;)
		{ 
			byte result;

			char hi = *hex++;
			if(hi == ' ') continue; // scan for the next character
			if(hi == 0) break;

			// start conversion
			char lo = *hex++;
			if(lo == 0 || lo == ' ') // if we have only one character
			{	result = hex_to_dec(hi);
			}
			else
			{	result = (hex_to_dec(hi) << 4) | hex_to_dec(lo);
			}

			output.push_back(result);

			if(lo == 0) break;
		}
	}

	void send_hex8(const char* hex, bool recieve_answer = true)
	{
		//DUMP_BYTES.hex("SH8", hex);

		tmp.clear();
		hex_to_bytes(hex, tmp);
		output_message.send_message(tmp);

		LED_BLINKING::task();

		if(recieve_answer) input_message.receive(tmp, false);
	}

	void send_integer(byte b1, byte b2, int integer, bool recieve_answer = true)
	{
		tmp.clear();
		tmp.push_back(b1);
		tmp.push_back(b2);
		int_to_vector(tmp, integer);
		output_message.send_message(tmp);

		if(recieve_answer) input_message.receive(tmp, false);
	}

	void send_integer_2(byte b1, int integer, byte b2, bool recieve_answer = true)
	{
		tmp.clear();
		tmp.push_back(b1);
		int_to_vector(tmp, integer);
		tmp.push_back(b2);
		output_message.send_message(tmp);

		if(recieve_answer) input_message.receive(tmp, false);
	}

public:
	void send_initialization_sequence()
	{
		LOG("initialization\n");

		// I don't know what most of these bytes mean ¯\_(ツ)_/¯

		// needed
		Message::cdc_write(byte(0xD0));
		Message::cdc_write(byte(0x00));
		Message::cdc_write(byte(0x30));
		Message::cdc_flush();
		input_message.receive(tmp, false);

		send_hex8("09 0C"); // ISC_CMD_ALERT_LOG
		send_hex8("09 18"); // ISC_CMD_ALERT_LOG

		sleep_ms(100); // timeout without this

		send_hex8("16"); // ISC_CMD_GET_SET_MONITOR
		send_hex8("0D 00"); // ??
		send_hex8("0D 01");
		send_hex8("0D 02");
		send_hex8("0D 03");
		send_hex8("0D 04");
		send_hex8("0D 05");
		send_hex8("0D 06");
		send_hex8("0D 07");
		send_hex8("0D 08");
		send_hex8("0D 09");
		send_hex8("0D 0A");
		send_hex8("0D 0B");
		send_hex8("0D 0C");
		send_hex8("0D 0D");
		send_hex8("0D 0E");
		send_hex8("0D 0F");
		send_hex8("0D 10");
		send_hex8("0D 11");
		send_hex8("0D 12");

		send_hex8("0D 14");
		send_hex8("0D 15");

		send_hex8("1E"); // ??

		send_hex8("16"); // repeated
		send_hex8("1E"); // repeated

		send_hex8("17"); // ISC_CMD_GET_SET_STIMULUS
		
		send_hex8("06"); // ISC_CMD_USER_ID
		send_hex8("10"); // ISC_CMD_MOTOR_FILE

		send_hex8("03 7A 00 00 00 00"); // ISC_CMD_SET_PARAM1

		// this stops the motor from spinning if it was spinning
		send_hex8("01 18 54 00 18 91"); // ISC_CMD_SET_PARAM CPM_P_APP_CONFIG_REG?
		send_hex8("01 98 54 00 18 91"); // ISC_CMD_SET_PARAM ?

		send_hex8("17"); // ISC_CMD_GET_SET_STIMULUS
	}

	void motor_enable()
	{	LOG("motor_enable\n");

		//send_hex8("01 67 00 00 00 00"); // motor disable
		send_hex8("01 67 00 00 01 00"); // ISC_CMD_SET_PARAM
	}

	void motor_disable()
	{	LOG("motor_disable\n");

		send_hex8("01 67 00 00 00 00"); // ISC_CMD_SET_PARAM
	}

	void move(int distance, int velocity_limit, int acceleration_limit)
	{
		LOG("move %d\n", distance);

		// CPM_P_ACC_LIM (Trapezoidal acceleration limit)
		send_integer(byte(0x01), byte(0x3E), acceleration_limit);
		send_integer(byte(0x01), byte(0xBE), acceleration_limit); // ... ?? repeated
		
		// CPM_P_VEL_LIM (Trapezoidal velocity limit)
		send_integer(byte(0x01), byte(0x3D), velocity_limit);
		send_integer(byte(0x01), byte(0xBD), velocity_limit); // ... ?? repeated

		send_hex8("01 45 52 01"); // CPM_P_MOVE_DWELL

		send_integer(byte(0x46), byte(0x01), distance);
		// ISC_CMD_MOVE_POSN_EX
		// 0 - absolute, 1 - relative

		send_hex8("01 45 FF FF"); // CPM_P_MOVE_DWELL

		send_hex8("17"); // ISC_CMD_GET_SET_STIMULUS
	}

	void set_velocity(int v, int acceleration_limit = 2000)
	{
		LOG("set_velocity %d %d\n", v, acceleration_limit);

		// CPM_P_ACC_LIM (Trapezoidal acceleration limit)
		send_integer(byte(0x01), byte(0x3E), acceleration_limit);
		send_integer(byte(0x01), byte(0xBE), acceleration_limit); // ... ?? repeated

		send_integer_2(byte(0x47), v, byte(0x10)); // ISC_CMD_MOVE_VEL_EX
		// 0x10 - MG_MOVE_VEL_IMMEDIATE Regular velocity that starts immediately
	}

	void get_position()
	{
		send_hex8("00 53", false); // ISC_CMD_GET_PARAM CPM_P_POSN_MEAS
		input_message.receive(tmp, true);

		position = int_from_vector(tmp);

		// Sometimes encoder jump +128 when crossing zero. I think it's a motor bug.
	}

	void get_velocity()
	{	
		send_hex8("00 55", false); // ISC_CMD_GET_PARAM CPM_P_VEL_MEAS
		input_message.receive(tmp, true);

		velocity = int_from_vector(tmp);
	}

	void byte_received(byte b)
	{
		input_message.byte_received(b);
	}

	volatile bool connected = false;
	volatile bool initialized = false;

	int position = 0;
	int velocity = 0;

	const int MAX_VELOCITY = 0x205FFFF;
	const int MAX_DISTANCE = 0x7FFffff;

} motor;

// Tiny USB host callbacks

void tuh_cdc_rx_cb(uint8_t idx)
{
	LED_BLINKING::blink();

	byte buf[64];
	auto count = tuh_cdc_read(idx, buf, sizeof(buf));

	for(auto i = 0; i < count; ++i)
	{	motor.byte_received(buf[i]);
	}
}

void tuh_cdc_mount_cb(uint8_t idx)
{
	tuh_cdc_itf_info_t itf_info = { 0 };
	tuh_cdc_itf_get_info(idx, &itf_info);

	LOG("CDC Interface is mounted: address = %u, itf_num = %u\n", itf_info.daddr, itf_info.bInterfaceNumber);

#ifdef CFG_TUH_CDC_LINE_CODING_ON_ENUM
	// CFG_TUH_CDC_LINE_CODING_ON_ENUM must be defined for line coding is set by tinyusb in enumeration
	// otherwise you need to call tuh_cdc_set_line_coding() first
	cdc_line_coding_t line_coding = { 0 };
	if ( tuh_cdc_get_local_line_coding(idx, &line_coding) )
	{
		LOG("Baudrate: %lu, Stop Bits : %u\n", line_coding.bit_rate, line_coding.stop_bits);
		LOG("Parity	: %u, Data Width: %u\n", line_coding.parity	, line_coding.data_bits);
	}
#endif

	motor.initialized = false;
	motor.connected = true;
}

void tuh_cdc_umount_cb(uint8_t idx)
{
	motor.connected = false;

	tuh_cdc_itf_info_t itf_info = { 0 };
	tuh_cdc_itf_get_info(idx, &itf_info);

	LOG("CDC Interface is unmounted: address = %u, itf_num = %u\n", itf_info.daddr, itf_info.bInterfaceNumber);
}

//

template<int Pin> class My_button
{
public:

	void initialize()
	{	gpio_init(Pin);
		gpio_set_dir(Pin, GPIO_IN);
		gpio_pull_up(Pin);

		//sleep_ms(10); // wait for pull up
	}

	bool pressed() // no debouncing here
	{	return gpio_get(Pin) == 0;
	}
};

//

class Lcd_emulation
{
public:
	void goto_pos(int pos, int line)
	{
		//LOG("goto_pos %d %d\n", pos, line);
	}
	void print(const char* str)
	{	LOG("print %s\n", str);
	}
} lcd;

class User_interface
{
	// generates a sawtooth wave
	static int saw(int a, int b, unsigned half_period, unsigned t)
	{	
		t += half_period / 2; // start from (a+b)/2
		
		unsigned f = t % (half_period * 2);
		if(f < half_period)
        	return a + f * (b - a) / half_period;
    	else
        	return b - (f - half_period) * (b - a) / half_period;
	}

	Rotary_encoder re;
	My_button<ENCODER_BUTTON_PIN> re_button = My_button<ENCODER_BUTTON_PIN>();

	int motor_speed = 0;

	int limit_value(int v, int min, int max)
	{	if(v < min) return min;
		if(v > max) return max;
		return v;
	}

public:

	void initialize()
	{	
		re.initialize(ENCODER_PIN_A, ENCODER_PIN_B);
		re_button.initialize();
	}

	void task()
	{
		auto e = re.process();
		if(e != 0)
		{	if(e > 0) motor_speed += 10;
			else motor_speed -= 10;

			motor_speed = limit_value(motor_speed, -100, 100);

			LOG("motor_speed %+d\n", motor_speed);

			if(motor_speed == 0)
			{	motor.motor_disable();
			}
			else
			{	motor.motor_enable();
				//motor.move(motor.MAX_DISTANCE * sign_p, motor.MAX_VELOCITY * p / 100, 2000);

				auto v = (long long)motor.MAX_VELOCITY * motor_speed / 100;
				motor.set_velocity(v);
			}
		}
	}

} user_interface;

int main()
{
	board_init();

	// settings for picoprobe
	uart_init(uart0, 9600);
	gpio_set_function(0, GPIO_FUNC_UART);
	gpio_set_function(1, GPIO_FUNC_UART);
	//

	tuh_init(BOARD_TUH_RHPORT);

	bool last_button_state = false;

	auto last_motor_query_time = board_millis();
	int last_motor_position = 0;

	user_interface.initialize();

	while (true)
	{
		tuh_task();
		LED_BLINKING::task();

		if(motor.connected)
		{
			if(!motor.initialized)
			{	motor.initialized = true;
				motor.send_initialization_sequence();
			}

			if(board_millis() - last_motor_query_time >= 250)
			{	last_motor_query_time = board_millis();
		
				motor.get_position();

				if(motor.position != last_motor_position)
				{	last_motor_position = motor.position;

					LOG("P %d\n", motor.position);
				}
			}
		}

		user_interface.task();
/*
		auto b = user_button.pressed();
		if(b != last_button_state)
		{
			LOG("button %d -> %d\n", last_button_state, b);
		
			last_button_state = b;

			if(b)
			{	motor.motor_enable();
				//motor.move(+5000, motor.MAX_VELOCITY/10, 1000);
				motor.move(+motor.MAX_DISTANCE, motor.MAX_VELOCITY, 2000);
						}
			else
			{	motor.motor_disable();
			}
		}*/
	}
}

// ! This definitions should be at the end of file !
const int X_rare_error_tracking::TOTAL_LINES = __LINE__;
int X_rare_error_tracking::errors[X_rare_error_tracking::TOTAL_LINES];
//