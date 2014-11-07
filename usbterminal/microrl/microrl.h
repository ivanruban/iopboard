#ifndef _MICRORL_H_
#define _MICRORL_H_

// microrl struct, contain internal library data
typedef struct
{
	char * prompt_str;                 // pointer to prompt string
	char cmdline [80];  // cmdline buffer
	int cmdlen;                        // last position in command line
	int cursor;                        // input cursor
	int (*execute) (int argc, const char * const * argv );            // ptr to 'execute' callback
	void (*print) (const char *);                                     // ptr to 'print' callback
} microrl_t;

// init internal data, calls once at start up
void microrl_init (microrl_t * pThis, void (*print)(const char*));

// pointer to callback func, that called when user press 'Enter'
// execute func param: argc - argument count, argv - pointer array to token string
void microrl_set_execute_callback (microrl_t * pThis, int (*execute)(int, const char* const*));

// insert char to cmdline (for example call in usart RX interrupt)
void microrl_insert_char (microrl_t * pThis, int ch);

#endif
