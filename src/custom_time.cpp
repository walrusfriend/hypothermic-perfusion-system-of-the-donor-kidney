#include "custom_time.h"

Time::Time(const uint8_t& hours = 0,
           const uint8_t& mins = 0,
           const uint8_t& secs = 0)
    : m_hours(hours)
	, m_mins(mins)
	, m_secs(secs) {};

void Time::reset() {
    m_hours = 0;
    m_mins = 0;
    m_secs = 0;
}

uint8_t Time::get_hours() const { 
    return m_hours; 
}

uint8_t Time::get_mins() const {
    return m_mins;
}

uint8_t Time::get_secs() const {
    return m_secs;
}

void Time::set_hours(const uint8_t& hours) { 
    m_hours = hours; 
}

void Time::set_mins(const uint8_t& mins) { 
    m_mins = mins; 
}

void Time::set_secs(const uint8_t& secs) { 
    m_secs = secs; 
}

Time& Time::operator++() {
    ++m_secs;

    if (m_secs > 59)
    {
        m_secs = 0;
        ++m_mins;
    }

    if (m_mins > 59)
    {
        m_mins = 0;
        ++m_hours;
    }

    return *this;
}