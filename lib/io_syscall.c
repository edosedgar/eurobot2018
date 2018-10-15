#include "io_syscall.h"
#include "stm32f4xx_ll_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"

#include <sys/stat.h>
#include <string.h>

int iosysc_setdev(USART_TypeDef *cd)
{
        chardev = cd;
        return 0;
}

static char getc_uart(void)
{
        while (!LL_USART_IsActiveFlag_RXNE(chardev))
                taskYIELD();
        return LL_USART_ReceiveData8(chardev);
}

static void putc_uart(char c)
{
        while (!LL_USART_IsActiveFlag_TC(chardev))
                taskYIELD();
        LL_USART_TransmitData8(chardev, c);
        return;
}

int _read_r(struct _reent *re, int fd, void *data, size_t len)
{
        (void) re;
        (void) fd;
        unsigned i;
        char c;
        int size  = 0;

        memset(data, 0, len);
        printf("req: %d\n", len);
        for (i = 0; i < len; i++) {
                c = getc_uart();
                putc_uart(c);
                if (c == '\r') {
                        putc_uart('\n');
                        ((char *) data)[i] = '\0';
                        size++;
                        break;
                }
                ((char *) data)[i] = c;
                size++;
        }
        printf("size: %d\n", size);
        return size;
}

int _write_r(struct _reent *re, int fd, const void *data, size_t len)
{
        (void) fd;
        (void) re;
        unsigned i;

        for (i = 0; i < len; i++) {
                if (((char *) data)[i] == '\n')
                        putc_uart('\r');
                putc_uart(((uint8_t *) data)[i]);
        }
        return len;
}

int _close_r(struct _reent *re, int fd)
{
        (void) fd;
        (void) re;

        return -1;
}

off_t _lseek_r(struct _reent *re, int fd, off_t pos, int whence)
{
        (void) fd;
        (void) re;
        (void) pos;
        (void) whence;

        return -1;
}

int _fstat_r(struct _reent *re, int fd, struct stat *st)
{
        (void) fd;
        (void) re;

        st->st_mode = S_IFCHR;
        return 0;
}

void *_malloc_r(struct _reent *re, size_t size)
{
        (void) re;

        return pvPortMalloc(size);
}

void _free_r(struct _reent *re, void *ptr)
{
        (void) re;

        vPortFree(ptr);
        return;
}

int _isatty_r(struct _reent *re, int fd)
{
        (void) re;
        (void) fd;

        return 1;
}
