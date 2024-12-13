#pragma once

/**
 * Logger class
 * ^^^^^^^^^^^^
 */

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

#define STRING_LENGTH        15

/* Classic log */
#define LOG(level, topic, message)            IOL::get_instance().log(level, topic, message) 
/* Set log level */
#define LOG_LEVEL(level)                      IOL::get_instance().set_log_level(level)

/* Usage */
// LOG(Level::LDEBUG, Topic::P2F2P, "Log content on topic debug.");
// Log(Level::LINFO, Topic::P2F2P, "Log content on topic info.");
// Log(Level::LWARNING, Topic::P2F2P, "Log content on topic warning.");
// Log(Level::LERROR, Topic::P2F2P, "Log content on topic error.");

enum class Level
{
    LDEBUG,
    LINFO, 
    LWARNING,
    LERROR,
    // more levels ...
};

enum class Topic 
{
    P2F2P,
    // more topics ...
};

class IOL
{
public:
    IOL() : current_level(Level::LDEBUG) {}  // Default level is LDEBUG
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
    void log(Level level, Topic topic, const std::string& str);

    /**
     * Set log level
     * @param level Log level to set
     */
    void set_log_level(Level level);

private:
    Level current_level;

    static std::string praefix_build(const Level& level, const Topic& topic);
    static std::ostringstream actual_time(void);
    static std::ostringstream level(const Level& level);
    static std::ostringstream topic(const Topic& topic);

};

/* Eof */