#ifndef _utils_excuteOnce_hpp
#define _utils_excuteOnce_hpp

#define excuteOnce(statement)                \
    {                                        \
        static char excuteOnceFlag = [&]() { \
            { statement; }                   \
            return 0;                        \
        }();                                 \
    }

#endif  // _utils_excuteOnce_hpp