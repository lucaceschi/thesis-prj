#include "framework/debug.hpp"

#include <stdio.h>
#include <stdarg.h>

using namespace frmwrk;


void Debug::log(const char * format, ...)
{
	va_list argptr;

	printf("%s[log]%s\t\t", FRMWRK_DBGSTYLE_WHITE, FRMWRK_DBGSTYLE_RESET);
	va_start(argptr, format);
	vprintf(format, argptr);
	va_end(argptr);
	printf("\n");
}

void Debug::logInfo(const char * format, ...)
{
	va_list argptr;

	printf("%s[info]%s\t\t", FRMWRK_DBGSTYLE_CYAN, FRMWRK_DBGSTYLE_RESET);
	va_start(argptr, format);
	vprintf(format, argptr);
	va_end(argptr);
	printf("\n");
}

void Debug::logSuccess(const char * format, ...)
{
	va_list argptr;

	printf("%s[success]%s\t", FRMWRK_DBGSTYLE_GREEN, FRMWRK_DBGSTYLE_RESET);
	va_start(argptr, format);
	vprintf(format, argptr);
	va_end(argptr);
	printf("\n");
}

void Debug::logWarning(const char * format, ...)
{
	va_list argptr;

	printf("%s[warning]%s\t", FRMWRK_DBGSTYLE_YELLOW, FRMWRK_DBGSTYLE_RESET);
	va_start(argptr, format);
	vprintf(format, argptr);
	va_end(argptr);
	printf("\n");
}

void Debug::logError(const char * format, ...)
{
	va_list argptr;

	printf("%s%s[error]%s%s\t\t",
		FRMWRK_DBGSTYLE_RED, FRMWRK_DBGSTYLE_BOLD, FRMWRK_DBGSTYLE_RESET, FRMWRK_DBGSTYLE_RED);
	va_start(argptr, format);
	vprintf(format, argptr);
	va_end(argptr);
	printf("%s\n", FRMWRK_DBGSTYLE_RESET);
}

void Debug::logGlfwError(int error, const char * description)
{
    Debug::logError("GLFW ERROR #%i: %s", error, description);
}