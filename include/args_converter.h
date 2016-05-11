#ifndef ARGS_CONVERTER_H
#define ARGS_CONVERTER_H

#include <string>
#include <vector>

class args_converter
{
public:
	args_converter(const std::string& exe_name, const std::string& cmdline_args);
	args_converter(const std::string& cmd);
	int get_argc();
	char** get_argv();
	~args_converter();
private:
	void init(std::string cmd);
	void split(const std::string& cmd, std::vector<std::string>& strs);
	int argc;
	char** argv;
};

#endif