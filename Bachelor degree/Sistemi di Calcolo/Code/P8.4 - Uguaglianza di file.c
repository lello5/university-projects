//Scrivere una funzione int file_eq(char* f1, char* f2) che, dati i percorsi di due file f1 e f2, restituisce zero se i file sono uguali, un valore maggiore di zero se i file sono diversi, e -1 in caso di errore.

#include "e4.h"
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define BUF_SIZE 4096

int file_eq(char* f1, char* f2) {

    int res, equal = 1;
    char* buf1 = NULL;
    char* buf2 = NULL;
    int fd1 = -1;
    int fd2 = -1;

    buf1 = malloc(BUF_SIZE);
    if (buf1 == NULL) goto error;

    buf2 = malloc(BUF_SIZE);
    if (buf2 == NULL) goto error;

    fd1 = open(f1, O_RDONLY);
    if (fd1 == -1) goto error;

    fd2 = open(f2, O_RDONLY);
    if (fd2 == -1) goto error;

    for (;;) {
        int r1 = read(fd1, buf1, BUF_SIZE);
        if (r1 == -1) goto error;

        int r2 = read(fd2, buf2, BUF_SIZE);
        if (r2 == -1) goto error;

        if (r1 != r2 || memcmp(buf1, buf2, r1) != 0) {
            equal = 0;
            break;
        }

        if (r1 == 0) break;
    }

    free(buf1);
    free(buf2);
    res = close(fd1);
    if (res == -1) goto error;
    res = close(fd2);
    if (res == -1) goto error;

    return !equal;
error: ;
    int e = errno;
    if (buf1 != NULL) free(buf1);
    if (buf2 != NULL) free(buf2);
    if (fd1 != -1) close(fd1);
    if (fd2 != -1) close(fd2);
    errno = e;
    return -1;
}
