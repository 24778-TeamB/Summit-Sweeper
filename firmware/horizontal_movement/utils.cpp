#include "utils.h"
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>

bool isInteger(const char *str)
{
	uint32_t i = 0;
	if (str[0] == '-')
		i++;

	for (; str[i] != '\0' && str[i] != '\n' && str[i] != '\r'; ++i)
	{
		if (!isdigit(str[i]))
			return false;
	}

	return true;
}

bool isFloat(const char *str)
{
	uint32_t i = 0, num_e = 0, num_dp = 0;
	if (str[0] == '-')
		i++;

	for (; str[i] != '\0' && str[i] != '\n' && str[i] != '\r'; ++i)
	{
		if (!isdigit(str[i]))
		{
			if (str[i] == 'e')
			{
				num_e++;
				if (num_e == 2)
					return false;
				if (str[i + 1] == '-' && isdigit(str[i + 2]))
				{
					i++;
					continue;
				}
			}
			else if (str[i] == '.')
			{
				num_dp++;
				if (num_dp == 2)
					return false;
			}
			else
				return false;
		}
	}

	return true;
}
