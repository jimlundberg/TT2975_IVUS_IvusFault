@ECHO OFF
if not exist "IvusComm.h" (echo #include "IvusFault_Test.h">"IvusComm.h")
if not exist "IvusControl.h" (echo #include "IvusFault_Test.h">"IvusControl.h")
if not exist "IVUS.h" (echo #include "IvusFault_Test.h">"IVUS.h")
if not exist "Fault.h" (echo #include "IvusFault_Test.h">"Fault.h")
