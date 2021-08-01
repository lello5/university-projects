/*
Si vuole scrivere una funzione che misura il tempo medio in millisecondi per accesso a disco in lettura e scrittura in modo casuale e in modo sequenziale. La funzione ha il seguente prototipo:
void file_access_time(const char* file, unsigned trials, time_rec_t *t)
dove file è il file da accedere (sovrascrivendolo), trials è il numero di accessi casuali da effettuare sia per le letture che per le scritture, e t è una struttura definita come segue in E3-files/e3.h:

typedef struct {
    unsigned file_size;
    double seq_read_t;
    double seq_write_t;
    double rnd_read_t;
    double rnd_write_t;
} time_rec_t;

I campi della struttura vanno interpretati come segue:

    file_size è la dimensione in byte del file
    seq_read_t è il tempo medio in millisecondi per lettura sequenziale di 4 byte
    seq_write_t è il tempo medio in millisecondi per scrittura sequenziale di 4 byte
    rnd_read_t è il tempo medio in millisecondi per lettura casuale di 4 byte
    rnd_write_t è il tempo medio in millisecondi per scrittura casuale di 4 byte

Tutti i campi della struttura devono essere riempiti dalla funzione file_access_time. Le letture e le scritture sequenziali devono scorrere l’intero file. I valori che vengono scritti nel file non sono rilevanti e possono essere arbitrari. L’eseguibile può essere compilato con make ed eseguito con ./e3 file trials dove file e trials sono definiti come sopra. E’ possibile generare un file di prova di file.dat da 390 MB con make file. Per ripulire la directory alla fine dell’esperimento usare make clean.
*/

#include "e3.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/types.h>
#include <unistd.h>

double get_real_time_msec() {
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return tp.tv_sec*1E3 + tp.tv_nsec*1E-6;
}

void check_error(int err, char* msg) {
    if (err != -1) return;
    perror(msg);
    exit(EXIT_FAILURE);
}

void file_access_time(const char* file, unsigned trials, time_rec_t *t) {
    unsigned i, val, cnt;
    double start, elapsed;
    off_t off;
    int res, fd = open(file, O_RDWR);
    check_error(fd, "open");

    // get file size
    t->file_size = lseek(fd, 0, SEEK_END);
    check_error(t->file_size, "lseek");

    printf("File size: %f MB\n", t->file_size/(1024.0*1024.0));

    // random read
    printf("Performing %u random reads...\n", trials);
    start = get_real_time_msec();
    for (i=0; i<trials; ++i) {
        off = rand() % t->file_size;
        off = lseek(fd, off, SEEK_SET);
        check_error(off, "lseek");
        res = read(fd, &val, sizeof(val));
        check_error(res, "read");
    }
    t->rnd_read_t = (get_real_time_msec() - start)/trials;

    // random write
    printf("Performing %u random writes...\n", trials);
    start = get_real_time_msec();
    for (i=0; i<trials; ++i) {
        off = rand() % t->file_size;
        off = lseek(fd, off, SEEK_SET);
        check_error(off, "lseek");
        res = write(fd, &val, sizeof(val));
        check_error(res, "read");
    }
    t->rnd_write_t = (get_real_time_msec() - start)/trials;

    // sequential read
    printf("Reading the file sequentially...\n");
    off = lseek(fd, 0, SEEK_SET);
    check_error(off, "lseek");
    cnt = 0;
    start = get_real_time_msec();
    for (;;) {
        cnt++;
        res = read(fd, &val, sizeof(val));
        check_error(res, "read");
        if (res == 0) break;
    }
    t->seq_read_t = (get_real_time_msec() - start)/cnt;

    // sequential write
    printf("Writing the file sequentially...\n");
    off = lseek(fd, 0, SEEK_SET);
    check_error(off, "lseek");
    start = get_real_time_msec();
    for (i=0; i<cnt; ++i) {
        res = write(fd, &val, sizeof(val));
        check_error(res, "read");
        if (res == 0) break;
    }
    t->seq_write_t = (get_real_time_msec() - start)/cnt;

    res = close(fd);
    check_error(res, "close");
}
