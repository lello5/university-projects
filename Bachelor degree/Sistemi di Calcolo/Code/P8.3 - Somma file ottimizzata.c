//Ottimizzare la soluzione dell’esercizo P8.1 leggendo a blocchi di 4KB e non a word di 32 bit.

#include <unistd.h> // read, write, close
#include <fcntl.h>  // open
#include <stdlib.h> // exit
#include <stdio.h>  // perror

#include "e3.h"

#define BLOCK_SIZE 1024

int sum(const char* filename, unsigned long *psum) {

    unsigned *block = malloc(BLOCK_SIZE*sizeof(unsigned)), i;
    if (block == NULL) {
        perror("errore in malloc");
        exit(EXIT_FAILURE);
    }

    unsigned long s = 0;

    int fd = open(filename, O_RDONLY);
    if (fd == -1) {
        perror("errore in open");
        exit(EXIT_FAILURE);
    }

    for(;;) {
        ssize_t res = read(fd, block, sizeof(BLOCK_SIZE)*sizeof(unsigned));
        if (res == -1) {
            perror("errore in read");
            exit(EXIT_FAILURE);
        }

        if (res > 0){
            for (i = 0; i < res/4; ++i)
                s += block[i];
        }
        else break; //  termina quando il file si svuota
    }

    fd = close(fd);
    if (fd == -1) {
        perror("errore in close");
        exit(EXIT_FAILURE);
    }

    free(block);

    *psum = s;

    return EXIT_SUCCESS;
}
