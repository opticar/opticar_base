//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************

#include <stdint.h>

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif
