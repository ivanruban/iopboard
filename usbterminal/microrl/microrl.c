#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "microrl.h"
#include <stdio.h>

void microrl_init (microrl_t * pThis, void (*print) (const char *))
{
	memset(pThis->cmdline, 0, sizeof(pThis->cmdline));
	pThis->cmdlen =0;
	pThis->cursor = 0;
	pThis->execute = NULL;
	pThis->print = print;
}

void microrl_set_execute_callback (microrl_t * pThis, int (*execute)(int, const char* const*))
{
	pThis->execute = execute;
}

#define MAX_ARGS 4

void microrl_insert_char(microrl_t * pThis, int ch)
{
	char outstr [] = {0,0};
	char* p;
	char* delims = { " " };
	char* args[MAX_ARGS];
	int i;

	switch (ch)
	{
		case '\n':
		case '\r':
			pThis->print("\n");
		  if(0 < pThis->cmdlen)
		  {
				p = strtok(pThis->cmdline, delims);
				i = 0;
				while(p != NULL)
				{
               if(i<MAX_ARGS)
               {
                  args[i++] = p;
               }
					else
					{
						break;
					}
					p = strtok( NULL, delims );
				}
				if(0<i)
            {
					pThis->execute(i, args);
				}

				pThis->cmdlen = 0;
				memset(pThis->cmdline, 0, sizeof(pThis->cmdline));
			}
		break;

		case '\b':
		  if(0 < pThis->cmdlen)
         {
				pThis->cmdline[--pThis->cmdlen] = '\0';
			}
		  pThis->print("\b");
		break;

		default:
		  if(iscntrl(ch))
			{
				break;
			}

			outstr[0] = ch;
		  pThis->print(outstr);

			pThis->cmdline[pThis->cmdlen++] = ch;
			if(pThis->cmdlen == sizeof(pThis->cmdline))
		  {
				pThis->cmdlen = 0;
				memset(pThis->cmdline, 0, sizeof(pThis->cmdline));
			}
		break;
	}
}
