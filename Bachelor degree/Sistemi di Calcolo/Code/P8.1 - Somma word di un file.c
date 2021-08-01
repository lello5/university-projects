/*
Si vuole scrivere un programma che calcola la somma dei valori senza segno a 32 bit contenuti in un file, ignorando eventuali byte finali resto della divisione per 4, se la lunghezza del file in byte non è divisibile per 4. La lettura deve avvenire 4 byte alla volta.
*/

#include <unistd.h> // read, write, close
#include <fcntl.h>  // open
#include <stdlib.h> // exit
#include <stdio.h>  // perror

#include "/e1.h"

int sum(const char* filename, unsigned long *psum) {

    unsigned long s = 0;

    int fd = open(filename, O_RDONLY);
    if (fd == -1) {
        perror("errore in open");
        exit(EXIT_FAILURE);
    }

    for(;;) {
        unsigned data;
        ssize_t res = read(fd, &data, sizeof(data));

        if (res == -1) {
            perror("errore in read");
            exit(EXIT_FAILURE);
        }

        if (res == 4) s += data; // prende solo valori a 4 byte

        if (res == 0) break; //  termina quando il file si svuota
    }

    fd = close(fd);
    if (fd == -1) {
        perror("errore in close");
        exit(EXIT_FAILURE);
    }

    *psum = s;

    return EXIT_SUCCESS;
}
