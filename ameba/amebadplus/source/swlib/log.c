/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ameba_soc.h"
#include "log.h"
#include <string.h>

static const char *const TAG = "LOG";
/* Define default log-display level*/
rtk_log_level_t rtk_log_default_level = RTK_LOG_DEFAULT_LEVEL;

/* Define cache aray*/
rtk_log_tag_t rtk_log_tag_array[LOG_TAG_CACHE_ARRAY_SIZE] = {0};
/* Count cache array usage */
static volatile uint32_t rtk_log_entry_count = 0;

#define get_num_va_args(_args, _lcount)				\
	(((_lcount) > 1)  ? va_arg(_args, long long int) :	\
	(((_lcount) == 1) ? va_arg(_args, long int) :		\
			    va_arg(_args, int)))

#define get_unum_va_args(_args, _lcount)				\
	(((_lcount) > 1)  ? va_arg(_args, unsigned long long int) :	\
	(((_lcount) == 1) ? va_arg(_args, unsigned long int) :		\
			    va_arg(_args, unsigned int)))

#define get_char_upper_lower(_upper, _rem)				\
	(((_rem) < 0xa)   ?  ('0' + _rem) : 					\
	(((_upper) == 1)   ?  ('A' + (_rem - 0xa)) : 			\
	('a' + (_rem - 0xa))))

#define is_digit(c) ((c >= '0') && (c <= '9'))

static int print_string(const char *str)
{
	int count = 0;

	//assert_param(str != NULL);
	if (str == NULL) {
		return -1;
	}
	for (; *str != '\0'; str++) {
		DiagPutChar(*str);
		count++;
	}
	return count;
}

static int print_unsigned_num(unsigned long long int unum, unsigned int radix,
							  char padc, int padn, int upper, int sign)
{
	/* Just need enough space to store 64 bit decimal integer */
	char num_buf[20];
	int i = 0, count = 0;
	unsigned int rem;
	/* 1. convert num to str*/
	do {
		rem = unum % radix;
		num_buf[i] = get_char_upper_lower(upper, rem);
		i++;
		unum /= radix;
	} while (unum > 0U);
	/* 2. If there is a sign bit, subtract the padn by 1. */
	if (sign) {
		count++;
		padn--;
	}

	/* 3. If fill 0, need to print the sign first, and then fill in 0 */
	if (sign && (padc == '0')) {
		DiagPutChar('-');
	}
	/* 4. Fill the character padc */
	if (padn > 0) {
		while (i < padn) {
			DiagPutChar(padc);
			count++;
			padn--;
		}
	}
	/* 5.If fill a space, need to wait for step 4 to complete before filling the sign. */
	if (sign && (padc == ' ' || padc == '\0')) {
		DiagPutChar('-');
	}
	/* 6. Print the numbers*/
	while (--i >= 0) {
		DiagPutChar(num_buf[i]);
		count++;
	}

	return count;
}

static inline int pad_char_control(int padn, char padc)
{
	int count = 0;
	while (padn > 0) {
		DiagPutChar(padc);
		padn--;
		count++;
	}
	return count;
}

static inline int pad_num_control(unsigned long long int unum, unsigned int radix, char padc,
								  int padn, int upper, int sign, int pad_on_right)
{
	int count = 0;
	/* 1. if '-' left align was set, pad character in the right of num*/
	if (pad_on_right) {
		int width = print_unsigned_num(unum, radix, padc, 0, upper, sign);
		count += width;
		//if padc is equal to '\space' and pad number is great than width of num, execute pad opration. Otherwise, no operation.
		if ((padn > width) && (padc  == ' ')) {
			count += pad_char_control(padn - width, padc);
		}
		/* 2. Normally, pad operation is executed on the left of num.*/
	} else {
		count += print_unsigned_num(unum, radix, padc, padn, upper, sign);
	}
	return count;
}
/*******************************************************************
 * Reduced format print for Trusted firmware.
 * The following type specifiers are supported by this print
 * %lx - hexadecimal format
 * %s - string format
 * %lu or %i - signed decimal format
 * %u - unsigned decimal format
 * %p - pointer format
 *
 * The following length specifiers are supported by this print
 * %l - long int (64-bit on AArch64)
 * %ll - long long int (64-bit on AArch64)
 * %z - size_t sized integer formats (64 bit on AArch64)
 *
 * The following padding specifiers are supported by this print
 * %0NN - Left-pad the number with 0s (NN is a decimal number)
 * %NN - Left-pad the number with spaces
 * %-NN - Right-pad the number with spaces
 *
 * The print exits on all other formats specifiers other than valid
 * combinations of the above specifiers.
 *******************************************************************/
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
int DiagVprintf(const char *fmt, va_list args)
{
	int l_count;
	long long int num;
	unsigned long long int unum;
	char *str;

	int count = 0; 		/* Number of printed characters */
	int upper = 0;

	// pad control
	char padc = '\0'; 	/* Padding character */
	int padn; 			/* Number of characters to pad */
	int pad_on_right;
	int sign = 0;

	char c = '\0';
	int percent_flag = 0;
	char *tempstr = NULL;
	int tempcount = 0;

	while (*fmt != '\0') {
		l_count = 0;
		percent_flag = pad_on_right = padn = sign = 0;
		padc = '\0';

		if (*fmt == '%') {
			percent_flag = 1;
			fmt++;
			/* Check the format specifier */
loop:
			switch (*fmt) {
			case 'i': /* Fall through to next one */
			case 'd':
				num = get_num_va_args(args, l_count);
				if (num < 0) {
					sign = 1;
					unum = (unsigned long long int) - num;
				} else {
					unum = (unsigned long long int)num;
				}
				count += pad_num_control(unum, 10, padc, padn, upper, sign, pad_on_right);
				break;
			case 'u':
				unum = get_unum_va_args(args, l_count);
				count += pad_num_control(unum, 10, padc, padn, upper, sign, pad_on_right);
				break;
			case 'B':
			case 'b':
				unum = get_unum_va_args(args, l_count);
				count += pad_num_control(unum, 2, padc, padn, upper, sign, pad_on_right);
				break;
			case 'O':
			case 'o':
				unum = get_unum_va_args(args, l_count);
				count += pad_num_control(unum, 8, padc, padn, upper, sign, pad_on_right);
				break;
			case 'X':
				upper = 1;
			case 'x':
				unum = get_unum_va_args(args, l_count);
				count += pad_num_control(unum, 16, padc, padn, upper, sign, pad_on_right);
				break;
			case 'p':
				upper = 1;
				unum = (uintptr_t)va_arg(args, void *);
				if (sizeof(uintptr_t) == 4U) {
					padc = '0';
					padn = 8;
				} else {
					padc = '0';
					padn = 16;
				}
				count += pad_num_control(unum, 16, padc, padn, upper, sign, pad_on_right);
				break;
			case 's':
				str = va_arg(args, char *);
				tempstr = str;
				tempcount = 0;
				while (*tempstr != '\0') {
					tempstr++;
					tempcount++;
				}
				// left pad
				if (!pad_on_right && (padn - tempcount > 0)) {
					count += pad_char_control(padn - tempcount, padc);
				}
				// print string
				count += print_string(str);
				// right pad
				if (pad_on_right && (padn - tempcount > 0)) {
					count += pad_char_control(padn - tempcount, padc);
				}
				break;
			case 'c':
			case 'C':
				c = va_arg(args, int);
				// left pad
				if (!pad_on_right && (padn > 1)) {
					count += pad_char_control(padn - 1, padc);
				}
				DiagPutChar(c);
				count++;
				//right pad
				if (pad_on_right && (padn > 1)) {
					count += pad_char_control(padn - 1, padc);
				}
				break;
			case 'z':
				if (sizeof(size_t) == 8U) {
					goto not_support;
				}
				fmt++;
				goto loop;
			case 'L':
			case 'l':
				l_count++;
				fmt++;
				goto loop;
			case '-':
				fmt++;
				pad_on_right++;
				goto loop;
			/*% is followed by a %. */
			case '%':
				if (percent_flag) {
					DiagPutChar('%');
				}
				count++;
				break;
			case '0':
				padc = '0';
				padn = 0;
				fmt++;
				goto pad_count;
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				padc = ' ';
				padn = 0;
				goto pad_count;
			/*% is followed by a spaces, so padc = \space. */
			case ' ':
				padc = ' ';
				padn = 0;
				fmt++;
pad_count:
				for (;;) {
					char ch = *fmt;
					if (ch == '\0') { // deal with case '% \0'.
						goto exit;
					} else if (!is_digit(ch)) {
						goto loop;
					}
					padn = (padn * 10) + (ch - '0');
					fmt++;
				}
			case '\0':
				break;
			default:
not_support:
				/* Exit on any other format specifier */
				return -1;
			}
			fmt++;
			continue;
		}
		DiagPutChar(*fmt);
		fmt++;
		count++;
	}
exit:
	return count;
}

static inline int print_decimal_num(unsigned int unum, int radix, char padc, int padn, int sign)
{
	/* Just need enough space to store 31 bit decimal integer */
	char num_buf[10];
	int i = 0, count = 0;
	unsigned int rem;
	/* 1. convert num to str*/
	do {
		rem = unum % radix;
		num_buf[i] = get_char_upper_lower(0, rem);
		i++;
		unum /= radix;
	} while (unum > 0U);

	if (sign) {
		count++;
		padn--;
	}

	if (sign && (padc == '0')) {
		DiagPutChar('-');
	}

	/* 2. Fill the character padc */
	if (padn > 0) {
		while (i < padn) {
			DiagPutChar(padc);
			count++;
			padn--;
		}
	}
	if (sign && (padc == ' ' || padc == '\0')) {
		DiagPutChar('-');
	}
	/* 3. Print the numbers*/
	while (--i >= 0) {
		DiagPutChar(num_buf[i]);
		count++;
	}

	return count;
}

/*************************************************************************
 * Internal Nano format print to reduce stack size, num must within 32bits
 * The following type specifiers are supported by this print
 * %x - hexadecimal format
 * %s - string format
 * %c - char format
 * %d or %i - signed decimal format
 *
 * The following padding specifiers are supported by this print
 * %0NN - Left-pad the number with 0s (NN is a decimal number)
 * %NN - Left-pad the number with space
 *
 * The print exits on all other formats specifiers other than valid
 * combinations of the above specifiers.
 **************************************************************************/
int DiagVprintfNano(const char *fmt, va_list args)
{
	char *str;

	int count = 0; 		/* Number of printed characters */
	int sign = 0, num = 0;
	// pad control
	char padc = '\0'; 	/* Padding character */
	int padn; 			/* Number of characters to pad */
	int pad_on_right;

	char c = '\0';
	int percent_flag = 0;
	char *tempstr = NULL;
	int tempcount = 0;

	unsigned int unum;
	while (*fmt != '\0') {
		percent_flag = pad_on_right = padn = 0;
		padc = '\0';
		sign = 0;

		if (*fmt == '%') {
			percent_flag = 1;
			fmt++;
loop:
			/* Check the format specifier */
			switch (*fmt) {
			case 'i':
			case 'd':
				num = get_num_va_args(args, 0);
				if (num < 0) {
					sign = 1;
					unum = (unsigned int) - num;
				} else {
					unum = (unsigned int)num;
				}
				count += print_decimal_num(unum, 10, padc, padn, sign);
				break;
			case 'u':
				unum = (unsigned int)get_unum_va_args(args, 0);
				count += print_decimal_num(unum, 10, padc, padn, 0);
				break;
			case 'x':
				unum = (unsigned int)get_unum_va_args(args, 0);
				count += print_decimal_num(unum, 16, padc, padn, 0);
				break;
			case 's':
				str = va_arg(args, char *);
				tempstr = str;
				tempcount = 0;
				while (*tempstr != '\0') {
					tempstr++;
					tempcount++;
				}
				// left pad
				if (!pad_on_right && (padn - tempcount > 0)) {
					count += pad_char_control(padn - tempcount, padc);
				}
				// print string
				count += print_string(str);
				// right pad
				if (pad_on_right && (padn - tempcount > 0)) {
					count += pad_char_control(padn - tempcount, padc);
				}
				break;
			case 'c':
			case 'C':
				c = va_arg(args, int);
				// left pad
				if (!pad_on_right && (padn > 1)) {
					count += pad_char_control(padn - 1, padc);
				}
				DiagPutChar(c);
				count++;
				//right pad
				if (pad_on_right && (padn > 1)) {
					count += pad_char_control(padn - 1, padc);
				}
				break;
			/*% is followed by a %. */
			case '%':
				if (percent_flag) {
					DiagPutChar('%');
				}
				count++;
				break;
			case '0':
				padc = '0';
				padn = 0;
				fmt++;
				goto pad_count;
			case '1':
			case '2':
			case '3':
			case '4':
			case '5':
			case '6':
			case '7':
			case '8':
			case '9':
				padc = ' ';
				padn = 0;
				goto pad_count;
			/*% is followed by a spaces, so padc = \space. */
			case ' ':
				padc = ' ';
				padn = 0;
				fmt++;
pad_count:
				for (;;) {
					char ch = *fmt;
					if (ch == '\0') { // deal with case '% \0'.
						return count;
					} else if (!is_digit(ch)) {
						goto loop;
					}
					padn = (padn * 10) + (ch - '0');
					fmt++;
				}

			case '\0':
				break;
			default:
				/* Exit on any other format specifier */
				return -1;
			}
			fmt++;
			continue;
		}
		DiagPutChar(*fmt);
		fmt++;
		count++;
	}

	return count;
}
#pragma GCC diagnostic pop

u32 DiagPrintf(const char *fmt, ...)
{
	int ret;
	va_list va;

	va_start(va, fmt);
	ret = DiagVprintf(fmt, va);
	va_end(va);
	return ret;
}

u32 DiagPrintfNano(const char *fmt, ...)
{
	int ret;
	va_list va;

	va_start(va, fmt);
	ret = DiagVprintfNano(fmt, va);
	va_end(va);
	return ret;
}

/* support %s %d %x %c, %08x.*/
int DiagSnPrintf(char *buf, size_t size, const char *fmt, ...)
{
	va_list     ap;
	char *p, *s, *buf_end = NULL;
	char padc = ' ';
	int padn = 0;

	if (buf == NULL) {
		return 0;
	}

	va_start(ap, fmt);
	s = buf;
	buf_end = size ? (buf + size) : (char *)~0;
	for (; *fmt != '\0'; ++fmt) {
		padn = 0;
		padc = ' ';
		if (*fmt != '%') {
			*s++ = *fmt;

			if (s >= buf_end) {
				goto Exit;
			}

			continue;
		}
		if (*++fmt == 's') {
			p = va_arg(ap, char *);
			while (*p != '\0') {
				*s++ = *p++;
				if (s >= buf_end) {
					goto Exit;
				}
			}
		} else {	/* Length of item is bounded */

			char tmp[20], *q = tmp;
			int shift = 0;// = 12;

			int lpforchk = 0;
			if ((*fmt == 'p') || (*fmt == 'P')) {
				lpforchk = (uintptr_t)va_arg(ap, void *);
				if (sizeof(uintptr_t) == 4U) {
					padc = '0';
					padn = 8;
				} else {
					padc = '0';
					padn = 16;
				}
			} else {
				lpforchk = va_arg(ap, int);
			}
			if ((lpforchk) < 0x10) {
				shift = 0;
			} else if (((lpforchk) >= 0x10) && ((lpforchk) < 0x100)) {
				shift = 4;
			} else if (((lpforchk) >= 0x100) && ((lpforchk) < 0x1000)) {
				shift = 8;
			} else if (((lpforchk) >= 0x1000) && ((lpforchk) < 0x10000)) {
				shift = 12;
			} else if (((lpforchk) >= 0x10000) && ((lpforchk) < 0x100000)) {
				shift = 16;
			} else if (((lpforchk) >= 0x100000) && ((lpforchk) < 0x1000000)) {
				shift = 20;
			} else if (((lpforchk) >= 0x1000000) && ((lpforchk) < 0x10000000)) {
				shift = 24;
			} else if ((lpforchk) >= 0x10000000) {
				shift = 28;
			} else {
				shift = 28;
			}

			if (*fmt == '0') {
				fmt++;
				padc = '0';
			}

			if ((*fmt  > '0') && (*fmt  <= '9')) {
				int width;
				unsigned char fch = *fmt;
				for (width = 0; (fch >= '0') && (fch <= '9'); fch = *++fmt) {
					width = width * 10 + fch - '0';
				}
				padn = width;
			}
			/*
			 * Before each format q points to tmp buffer
			 * After each format q points past end of item
			 * Only %p and %x can be filled with digits 0 or spaces.
			 */
			if ((*fmt == 'x') || (*fmt == 'X') || (*fmt == 'p') || (*fmt == 'P')) {
				/* With x86 gcc, sizeof(long) == sizeof(int) */
				const int lp = lpforchk;
				int h = lp;
				int hex_count = 0;
				unsigned int h_back = h;
				int ncase = (*fmt & 0x20);

				if ((*fmt == 'p') || (*fmt == 'P')) {
					*q++ = '0';
					*q++ = 'X' | ncase;
				}

				while (h_back) {
					hex_count += 1;
					h_back  = h_back >> 4;
				}
				while (padn-- > hex_count) {
					*q++ = padc;
				}

				for (; shift >= 0; shift -= 4) {
					*q++ = "0123456789ABCDEF"[(h >> shift) & 0xF] | ncase;
				}
			} else if (*fmt == 'd') {
				int i = lpforchk;
				char *r;
				int digit_space = 0;

				if (i < 0) {
					*q++ = '-';
					i = -i;
					digit_space++;
				}

				p = q;		/* save beginning of digits */

				do {
					*q++ = '0' + (i % 10);
					i /= 10;
					digit_space++;
				} while (i);


				for (; shift >= 0; shift -= 4) {

					if (digit_space-- > 0) {
						; //do nothing
					} else {
						*q++ = '0';
					}
				}

				/* reverse digits, stop in middle */
				r = q;		/* don't alter q */

				while (--r > p) {
					i = *r;
					*r = *p;
					*p++ = i;
				}
			} else if (*fmt == 'c') {
				*q++ = lpforchk;
			} else {
				*q++ = *fmt;
			}

			/* now output the saved string */
			for (p = tmp; p < q; ++p) {
				*s++ = *p;

				if (s >= buf_end) {
					goto Exit;
				}
			}
		}
	}
Exit:

	if (buf) {
		*s = '\0';
	}
	va_end(ap);
	return (s - buf);
}

/***
*  @brief	Print the modules' tag/level set by the rtk_log_level_set()
*
*  @param	rtk_log_tag_array cache array
*
*  @return	success,0; fail,-1
*
***/
int rtk_log_array_print(rtk_log_tag_t *rtk_log_tag_array)
{
	uint32_t index = MIN(rtk_log_entry_count, LOG_TAG_CACHE_ARRAY_SIZE);
	if (rtk_log_tag_array != NULL) {
		for (uint32_t i = 0; i < index; i++) {
			RTK_LOGS(TAG, RTK_LOG_INFO, "[%s] level = %d\n", rtk_log_tag_array[i].tag, rtk_log_tag_array[i].level);
		}
		return RTK_SUCCESS;
	}
	return RTK_FAIL;
}

/***
*  @brief	Add label and level to array
*
*  @param	tag the label to set
*
*  @param	level The level of the label to set
*
*  @return	none
*
***/
static inline void rtk_log_array_add(const char *tag, rtk_log_level_t level)
{
	if (rtk_log_entry_count >= LOG_TAG_CACHE_ARRAY_SIZE) {
		RTK_LOGS(TAG, RTK_LOG_WARN, "Cache array is full, and replace old entry\n");
	}
	/* Replace old entry with taking the remainder. */
	rtk_log_tag_array[rtk_log_entry_count % LOG_TAG_CACHE_ARRAY_SIZE].level = level;
	strncpy(rtk_log_tag_array[rtk_log_entry_count % LOG_TAG_CACHE_ARRAY_SIZE].tag, tag, LOG_TAG_MAX_LEN);
	rtk_log_entry_count++;
}
/***
*  @brief	Clear cache array
*
*  @return	none
*
***/
void rtk_log_array_clear(void)
{
	_memset(rtk_log_tag_array, 0, sizeof(rtk_log_tag_array));
	rtk_log_entry_count = 0;
}

/***
*  @brief	Get the log level from cache array
*
*  @param	tag the label of the module to look for
*
*  @return	If the tag is queried, its level is returned; Otherwise, the default level is returned.
*           1. level of tag
*           2. rtk_log_default_level
*
*  @note	public API
***/
rtk_log_level_t rtk_log_level_get(const char *tag)
{
	uint32_t index = MIN(rtk_log_entry_count, LOG_TAG_CACHE_ARRAY_SIZE);
	if (tag) {
		// Look for the tag in cache first
		for (uint32_t i = 0; i < index; i++) {
			if (_strcmp(rtk_log_tag_array[i].tag, tag) == 0) {
				return (rtk_log_level_t)rtk_log_tag_array[i].level;
			}
		}
	}
	// If not found, return default level
	return rtk_log_default_level;
}

/***
*  @brief	Set the log display level of the module tag
*
*  @param	tag the label to set
*
*  @param	level The level of the label to set
*
*  @return	success,0; fail,-1
*
*  @note
***/
int rtk_log_level_set(const char *tag, rtk_log_level_t level)
{
	uint32_t i = 0;
	uint32_t index = MIN(rtk_log_entry_count, LOG_TAG_CACHE_ARRAY_SIZE);
	if ((tag == NULL) || (level > RTK_LOG_DEBUG)) {
		return RTK_FAIL;
	}
	// for wildcard tag, remove all array items and clear the cache
	if (_strcmp(tag, "*") == 0) {
		rtk_log_default_level = level;
		return RTK_SUCCESS;
	}
	// search in the cache and update the entry it if exists
	for (i = 0; i < index; i++) {
		if (_strcmp(rtk_log_tag_array[i].tag, tag) == 0) {
			rtk_log_tag_array[i].level = level;
			break;
		}
	}
	/*Case:
	    1. Not found, but there is empty array that can be storaged. i in [rtk_log_entry_count, LOG_TAG_CACHE_ARRAY_SIZE -1]
	    2. The array is full and replace old entry.
	    3. Found it. i < rtk_log_entry_count
	*/
	if (i >= index) { //
		rtk_log_array_add(tag, level);
	}
	return RTK_SUCCESS;
}

/***
*  @brief	dump memory in word
*
*  @param	src The starting address of the target memory
*
*  @param   len The length of the target memory
*
*  @return	none
*
*  @note	for exmample: [200447b4] 20015e08 00000000 20000674 000055d9 0c002763 20000749 0c0067f4 00000000
***/
void rtk_log_memory_dump_word(uint32_t *src, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		if (!i) {
			RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "[%08x] ", (u32)src);
		} else if (i % DISPLAY_NUMBER == 0) {
			RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "\r\n[%08x] ", (u32)(src + i));
		}
		RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "%08x ", (u32)src[i]);
	}
	RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "\n");
}
/***
*  @brief	dump memory in byte
*
*  @param	src The starting address of the target memory
*
*  @param   len The length of the target memory
*
*  @return	none
*
*  @note	for exmample: [200447b4] 08 5e 01 20 00 00 00 00
***/
void rtk_log_memory_dump_byte(uint8_t *src, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {
		if (!i) {
			RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "[%08x] ", (u32)src);
		} else if (i % DISPLAY_NUMBER == 0) {
			RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "\r\n[%08x] ", (u32)(src + i));
		}
		RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "%02x ", src[i]);
	}
	RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "\n");
}

/***
*  @brief	dump memory
*
*  @param	src_buff The starting address of the target memory
*
*  @param	buff_len The length of the target memory
*
*  @return	none
*
*  @note	1. format: field[length]
*           ADDR[10]+"   "+DATA_HEX[8*3]+" "+DATA_HEX[8*3]+"  |"+DATA_CHAR[8]+"|"
*           2. example:
*           [0xe005263]   7c 03 f0 7f 03 23 74 cf  e7 07 25 cd e7 a1 69 30  ||....#t...%...i0|
***/
extern int isprint(int c);
void rtk_log_memory_dump2char(const char *src_buff, uint32_t buff_len)
{
	//If src_buff length is eaqul to zero, exit.
	if (buff_len == 0) {
		return;
	}
	char *src_ptr;
	u8 *dst_ptr;
	uint32_t bytes_per_line = 0;

	char dst_buff[10 + 3 + BYTES_PER_LINE * 3 + 3 + BYTES_PER_LINE + 1 + 1];

	do {
		if (buff_len > BYTES_PER_LINE) {
			bytes_per_line = BYTES_PER_LINE;
		} else {
			bytes_per_line = buff_len;
		}

		src_ptr = (char *)src_buff;
		dst_ptr = (u8 *)dst_buff;
		dst_ptr += DiagSPrintf(dst_ptr, "[%p] ", src_buff);
		for (uint32_t i = 0; i < BYTES_PER_LINE; i ++) {
			if ((i & 7) == 0) {
				dst_ptr += DiagSPrintf(dst_ptr, " ");
			}
			if (i < bytes_per_line) {
				dst_ptr += DiagSPrintf(dst_ptr, " %02x", src_ptr[i]);
			} else {
				dst_ptr += DiagSPrintf(dst_ptr, "   ");
			}
		}
		dst_ptr += DiagSPrintf(dst_ptr, "  |");
		for (uint32_t i = 0; i < bytes_per_line; i ++) {
			if (isprint((int)src_ptr[i])) {
				dst_ptr += DiagSPrintf(dst_ptr, "%c", src_ptr[i]);
			} else {
				dst_ptr += DiagSPrintf(dst_ptr, ".");
			}
		}
		dst_ptr += DiagSPrintf(dst_ptr, "|");
		RTK_LOGS(NOTAG, RTK_LOG_ALWAYS, "%s\n", dst_buff);
		src_buff += bytes_per_line;
		buff_len -= bytes_per_line;
	} while (buff_len);
}

/**
 * @brief print log(stack size: 252bytes)
 *
 * @param level  current log lvel
 * @param tag    tag of the current log
 * @param letter the letter corresponding to a specific log level
 * @param fmt    the format string to be output
 * @param ... 	 other parameters
 */
void rtk_log_write(rtk_log_level_t level, const char *tag, const char letter, const char *fmt, ...)
{
	if (tag) {
		rtk_log_level_t level_of_tag = rtk_log_level_get(tag);
		va_list ap;
		if (level_of_tag < level) {
			return;
		}
		if (tag[0] != '#') {
			DiagPrintf("[%s-%c] ", tag, letter);
		}
		va_start(ap, fmt);
		DiagVSprintf(NULL, fmt, ap);
		va_end(ap);
	}
}

/**
 * @brief print log(smaller stack, 136Bytes)
 *
 * @param level  current log lvel
 * @param tag    tag of the current log
 * @param letter the letter corresponding to a specific log level
 * @param fmt    the format string to be output
 * @param ... 	 other parameters
 */
void rtk_log_write_nano(rtk_log_level_t level, const char *tag, const char letter, const char *fmt, ...)
{
	if (tag) {
		rtk_log_level_t level_of_tag = rtk_log_level_get(tag);
		va_list ap;
		if (level_of_tag < level) {
			return;
		}
		if (tag[0] != '#') {
			DiagPrintfNano("[%s-%c] ", tag, letter);
		}
		va_start(ap, fmt);
		DiagVprintfNano(fmt, ap);
		va_end(ap);
	}
}
