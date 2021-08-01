/*
Scrivere una funzione archiver che crea un file archivio in cui inserisce un numero arbitrario di file in modo simile a quanto avviene per un file tar.

1) archive è il pathname del file archivio di output (es. archive.dat)
2) files è un array di stringhe che rappresentano i pathname dei file di input da archiviare in archive
3) n è il numero di file da archiviare

Ogni file archiviato ha una header formata da: 1) 256 byte che contengono il pathname del file (come stringa C, quindi con terminatore zero alla fine del pathname - non è necessario ovviamente che il pathname effettivo usi tutti i 256 byte disponibili e i byte extra saranno padding) e 2) 8 byte che rappresentano la dimensione del file. Alla header seguono i byte del file stesso.

Se un file con il pathname archive esiste già, il suo contenuto deve essere inizialmente troncato a dimensione zero dalla funzione archiver. Il file archivio creato deve avere privilegi di lettura e scrittura per l’utente proprietario, sola lettura per il gruppo proprietario, e nessun permesso per tutti gli altri.
*/

#include "e2.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

void check_error(long res, char* msg) {
    if (res != -1) return;
    perror(msg);
    exit(EXIT_FAILURE);
}

#define FILENAME_LEN 256
#define BUF_SIZE 4096

void copy_file(int fd_dest, int fd_src) {
    ssize_t res;
    char buf[BUF_SIZE];
    for (;;) {
        res = read(fd_src, buf, BUF_SIZE);
        check_error(res, "read");
        if (res == 0) break;
        res = write(fd_dest, buf, res);
        check_error(res, "write");
    }
}

void archiver(const char* archive, const char** files, int n) {
    char filename[FILENAME_LEN];
    int fd_archive, fd, i;
    long res;

    fd_archive = open(archive, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    check_error(fd_archive, "open");

    for (i=0; i<n; ++i) {
        fd = open(files[i], O_RDONLY);
        check_error(fd, "open");

        long size = lseek(fd, 0, SEEK_END);
        check_error(size, "lseek");

        res = lseek(fd, 0, SEEK_SET);
        check_error(res, "lseek");

        strcpy(filename, files[i]);

        res = write(fd_archive, filename, FILENAME_LEN);
        check_error(res, "write");

        res = write(fd_archive, &size, sizeof(size));
        check_error(res, "write");

        copy_file(fd_archive, fd);

        res = close(fd);
        check_error(res, "close");
    }

    res = close(fd_archive);
    check_error(res, "close");
}
