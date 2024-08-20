#pragma once 

#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>
#include <ctime>
#include <memory>

#include "oswrappers.h"

#define FONT_RESET           "\033[0m"
#define FONT_BLACK           "\033[30m"              // Black 
#define FONT_RED             "\033[31m"              // Red 
#define FONT_GREEN           "\033[32m"              // Green 
#define FONT_YELLOW          "\033[33m"              // Yellow 
#define FONT_BLUE            "\033[34m"              // Blue 
#define FONT_MAGENTA         "\033[35m"              // Magenta 
#define FONT_CYAN            "\033[36m"              // Cyan 
#define FONT_WHITE           "\033[37m"              // White 
#define FONT_BOLDBLACK       "\033[1m\033[30m"       // Bold Black 
#define FONT_BOLDRED         "\033[1m\033[31m"       // Bold Red 
#define FONT_BOLDGREEN       "\033[1m\033[32m"       // Bold Green 
#define FONT_BOLDYELLOW      "\033[1m\033[33m"       // Bold Yellow 
#define FONT_BOLDBLUE        "\033[1m\033[34m"       // Bold Blue 
#define FONT_BOLDMAGENTA     "\033[1m\033[35m"       // Bold Magenta 
#define FONT_BOLDCYAN        "\033[1m\033[36m"       // Bold Cyan 
#define FONT_BOLDWHITE       "\033[1m\033[37m"       // Bold White 

/* Classic log */
#define LOG(level, message)            IOL::get_instance().log(level, message)   

enum class Level
{
    LINFO, 
    LWARNING,
    LERROR,
};

/* Forward declaration */

/* Class Logger */
class IOL
{
public:
    IOL() = default;
    virtual ~IOL() = default;

    /* Explicit delete */
    IOL(const IOL&) = delete;
    IOL& operator=(const IOL&) = delete;
    IOL(IOL&&) = delete;
    IOL& operator=(IOL&&) = delete;

    /**
     * Return static IOL instance 
     * @return Return IOL instance  
     */
    static IOL& get_instance(void);

    /**
     * Basic IOL level parser
     * @param level Level for logging
     * @param str String for console output
     * @return void
     */
    void log(Level level, const std::string& str);

    /**
     * Time function
     * @return std::string with actual time and date
     */
    std::string actual_time(void);

};

/* Eof */