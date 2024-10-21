#include "iol.h"

IOL& IOL::get_instance(void)
{
    static IOL instance;
    return instance;
}

void IOL::log(Level level, const std::string& message) 
{
    std::string prefix = actual_time();
    
    switch(level) 
    {
        case Level::LINFO:
            std::cout << FONT_BLUE << prefix << FONT_GREEN 
            << std::setw(20) << std::left << " info/sota" << FONT_RESET 
            << message 
            << std::endl;
            break;
        case Level::LWARNING:
            std::cout << FONT_BLUE << prefix << FONT_YELLOW 
            << std::setw(20) << std::left << " warn/sota" << FONT_RESET 
            << message 
            << std::endl;
            break;
        case Level::LERROR:
            std::cout << FONT_BLUE << prefix << FONT_RED 
            << std::setw(20) << std::left << " error/sota" << FONT_RESET 
            << message 
            << std::endl;
            break;
    }
}

std::string IOL::actual_time(void) 
{
    auto now = std::chrono::system_clock::now();
    auto now_as_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    
    std::tm now_tm;

    oswrapper::localtime(&now_as_time_t, &now_tm);

    std::ostringstream oss;

    oss << std::put_time(&now_tm, "[%Y-%m-%d %H:%M:%S.");
    oss << std::setfill('0') << std::setw(3) << now_ms.count();
    /* Fixed timezone */
    oss << " (UTC+0100)]"; 
    return oss.str();
}

/* Eof */