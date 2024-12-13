#pragma once

/**
 * Operating system wrappers class
 * ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 * This class implements the interface Ioswrappers and provides the functionalities.
 */

#include <ctime>

namespace oswrapper
{
    /**
     * Operating system wrappers
     */
    
    /* Work with time */
    void localtime(const time_t* time_time_t, tm* time_tm);

    /* More local cases ... */

} // namespace osw

/* Eof */