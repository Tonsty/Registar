#include "../include/args_converter.h"
#include <vector>
#include <boost/algorithm/string.hpp>

using namespace std;

args_converter::args_converter(const std::string& exe_name, const std::string& cmdline_args)
: argc(0)
, argv(NULL)
{
    init(exe_name + " " + cmdline_args);
}

args_converter::args_converter(const std::string& cmd)
: argc(0)
, argv(NULL)
{
    init(cmd);
}

void args_converter::split(const std::string& cmd, std::vector<string>& strs)
{
    int start_pos = 0;
    bool colon_flag = false;
    for (size_t i = 0; i < cmd.size(); ++ i)
    {
        if (colon_flag)
        {
            if (cmd[i] == '"')
            {
                colon_flag = false;                   
            }               
        }
        else
        {
            if (cmd[i] == ' ')
            {
                string str = cmd.substr(start_pos, i - start_pos);
                boost::erase_all(str, "\"");
                if (!str.empty())
                {
                    strs.push_back(str);
                }
                start_pos = i + 1;
            }
            else if (cmd[i] == '"')
            {
                colon_flag = true;
            }
        }
    }
    {
        string str = cmd.substr(start_pos, cmd.size() - start_pos);
        boost::erase_all(str, "\"");
        if (!str.empty())
        {
            strs.push_back(str);
        }
    }
}

void args_converter::init(std::string cmd)
{
    boost::trim(cmd);
    vector<string> strs;
    split(cmd, strs);   
    argc = strs.size();
    argv = new char*[strs.size() + 1];
    argv[argc] = NULL;
    for (size_t i = 0; i < strs.size(); ++ i)
    {
        argv[i] = new char[strs[i].length() + 1];
        strcpy(argv[i], strs[i].c_str());
    }
}

int args_converter::get_argc()
{
    return argc;
}

char** args_converter::get_argv()
{
    return argv;
}

args_converter::~args_converter()
{
    for (int i = 0; i < argc; ++ i)
    {
        delete[] argv[i];
    }
    delete[] argv;
}
