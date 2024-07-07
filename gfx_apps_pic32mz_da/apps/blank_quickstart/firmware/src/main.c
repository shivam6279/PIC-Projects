/*******************************************************************************
  Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file contains the "main" function for a project.

  Description:
    This file contains the "main" function for a project.  The
    "main" function calls the "SYS_Initialize" function to initialize the state
    machines of all modules in the system
 *******************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes


// *****************************************************************************
// *****************************************************************************
// Section: Main Entry Point
// *****************************************************************************
// *****************************************************************************

int main ( void )
{
    /* Initialize all modules */
    SYS_Initialize ( NULL );
    
    TRISDbits.TRISD7 = 0;
    LATDbits.LATD7 = 0;
    
    SRCON0A = 0;
    SRCON1A = 0;
    
    SRCON0B = 0;
    SRCON1B = 0;
    
    SRCON0C = 0;
    SRCON1C = 0;
    
    SRCON0D = 0;
    SRCON1D = 0;
    
    SRCON0E = 0;
    SRCON1E = 0;
    
    SRCON0F = 0;
    SRCON1F = 0;
    
    SRCON0G = 0;
    SRCON1G = 0;
    
    SRCON0H = 0;
    SRCON1H = 0;
    
    SRCON0J = 0;
    SRCON1J = 0;
    
    SRCON0K = 0;
    SRCON1K = 0;

    while ( true )
    {
        /* Maintain state machines of all polled MPLAB Harmony modules. */
        SYS_Tasks ( );
    }

    /* Execution should not come here during normal operation */

    return ( EXIT_FAILURE );
}


/*******************************************************************************
 End of File
*/

