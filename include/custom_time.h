#ifndef custom_time_h
#define custom_time_h

#include <stdint.h>

class Time {
public:
	Time(const uint8_t& hours = 0,
		 const uint8_t& mins = 0,
		 const uint8_t& secs = 0);

	void reset();

	uint8_t get_hours() const;
	uint8_t get_mins() const;
	uint8_t get_secs() const;

	void set_hours(const uint8_t& hours);
	void set_mins(const uint8_t& mins);
	void set_secs(const uint8_t& secs);

	Time& operator++();

private:
	uint8_t m_hours = 0;
	uint8_t m_mins = 0;
	uint8_t m_secs = 0;
};

#endif