#include "iol.h"

IOL& IOL::get_instance(void)
{
    static IOL instance;
    return instance;
}

void IOL::log(Level level, Topic topic, const std::string& message) 
{
    if (level >= current_level) 
    {
        // Print log message only if level is greater than or equal to current_level
        std::flush(std::cout << IOL::praefix_build(level, topic) << message << std::endl);
    }
    return;
}

void IOL::set_log_level(Level level) 
{
    current_level = level;
}

std::string IOL::praefix_build(const Level& level, const Topic& topic)
{
    std::stringstream oss;
    std::string level_topic = IOL::level(level).str() + "/" + IOL::topic(topic).str();
    if (level_topic.length() < (18 + STRING_LENGTH))
    {
        level_topic.append((18 + STRING_LENGTH) - level_topic.length(), ' ');
    }
    else
    {
        level_topic = level_topic.substr(0, 20);
    }
    oss << IOL::actual_time().str() << level_topic;
    return oss.str();
}

std::ostringstream IOL::actual_time(void) 
{
    auto now = std::chrono::system_clock::now();
    auto now_as_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm now_tm;
    /* Use operating system wrapper */
    oswrapper::localtime(&now_as_time_t, &now_tm);
    std::ostringstream oss;
    oss << FONT_BLUE << std::put_time(&now_tm, "[%Y-%m-%d %H:%M:%S.");
    oss << std::setfill('0') << std::setw(3) << now_ms.count();
    /* Fixed timezone */
    oss << " (UTC+0100)] " << FONT_RESET; 
    return oss;
}

std::ostringstream IOL::level(const Level& level)
{
    std::ostringstream oss;
    switch(level)
    {
        case Level::LDEBUG:
            oss << FONT_MAGENTA << "debug" << FONT_RESET;
            break;
        case Level::LINFO:
            oss << FONT_GREEN << "info" << FONT_RESET;
            break;
        case Level::LWARNING:
            oss << FONT_YELLOW << "warn" << FONT_RESET;
            break;
        case Level::LERROR:
            oss << FONT_RED << "error" << FONT_RESET;
            break;
            // more levels ...
        default:
            oss << FONT_WHITE << "unknown" << FONT_RESET;
            break;
    }
    return oss;
}

std::ostringstream IOL::topic(const Topic& topic)
{
    std::ostringstream oss;
    switch(topic)
    {
        case Topic::P2F2P:
            oss << FONT_GREEN << "p2f2p" << FONT_RESET;
            break;
            // more topics ...
        default:
            oss << FONT_WHITE << "unknown" << FONT_RESET;
            break;
    }
    return oss;
}

/* Eof */