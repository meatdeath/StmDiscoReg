@ECHO OFF
SET BUILD_VERSION = 0
IF EXIST ".\buildversion.cmd" (
    CALL ".\buildversion.cmd"
)
SET /A BUILD_VERSION=%BUILD_VERSION%+1
ECHO Current Build Version is %BUILD_VERSION%
ECHO #define BUILD_VERSION %BUILD_VERSION% > ".\version.h"
ECHO SET BUILD_VERSION=%BUILD_VERSION% > ".\buildversion.cmd"
