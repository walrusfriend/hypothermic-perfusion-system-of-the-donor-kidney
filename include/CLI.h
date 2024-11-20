#ifndef CLI_H
#define CLI_H

#include <Arduino.h>

struct Command
{
	String name;
	void(*handler)(const String&);

	/** TODO: Replace string with string_view */
	Command(String &&command_name, void(*command_handler)(const String &))
		: name(command_name)
		, handler(command_handler)
	{}

	~Command() {}
};

#endif