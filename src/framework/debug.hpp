#ifndef FRAMEWORK_DEBUG_H_
#define FRAMEWORK_DEBUG_H_

#define FRMWRK_DBGSTYLE_RESET		"\033[0m"
#define FRMWRK_DBGSTYLE_BLACK		"\033[30m"
#define FRMWRK_DBGSTYLE_RED			"\033[31m"
#define FRMWRK_DBGSTYLE_GREEN		"\033[32m"
#define FRMWRK_DBGSTYLE_YELLOW		"\033[33m"
#define FRMWRK_DBGSTYLE_BLUE		"\033[34m"
#define FRMWRK_DBGSTYLE_MAGENTA		"\033[35m"
#define FRMWRK_DBGSTYLE_CYAN		"\033[36m"
#define FRMWRK_DBGSTYLE_WHITE		"\033[37m"
#define FRMWRK_DBGSTYLE_BOLD		"\033[1m"
#define FRMWRK_DBGSTYLE_UNDERLINE	"\033[4m"


namespace frmwrk
{
    class Debug
    {
    public:
        static void log(const char * format, ...);
        static void logInfo(const char * format, ...);
        static void logSuccess(const char * format, ...);
        static void logWarning(const char * format, ...);
        static void logError(const char * format, ...);
        static void logGlfwError(int error, const char * description);
    };
}


#endif