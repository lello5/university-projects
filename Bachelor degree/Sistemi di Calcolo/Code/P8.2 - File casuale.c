/*
Lo scopo dell’esercizio è quello di scrivere file binari con la seguente struttura composti da numeri pseudo-casuali:

 0 |magic number|
 4 |   size     |
 8 |   rnd 1    |
12 |   rnd 2    |
16 |    ...     |
Il magic number deve essere 0xEFBEADDE per tutti i file generati con questo programma. I magic number sono codici associati ai tipi di file, specialmente se binari, per identificarli univocamente al momento dell’apertura, e sono disposti all’inizio del file. Il magic number è considerato a scopo di appredimento. SUbito dopo c’è il numero di valori casuali del file.

Si richiede di scrivere nel file E2-make-rnd-file/e2.c una funzione int make_rnd_file(int size, int seed, int mod, char *filename); con i seguenti parametri:
    size: numero di valori casuali registrati nel file;
    seed: seme del generatore pseudo-casuale;
    mod: limite superiore per i valori casuali, in valore assoluto;
    filename: percorso assoluto o relativo del file da generare.
*/

#include <unistd.h>
#include <stdio.h> 
#include <stdlib.h> 
#include <sys/stat.h> 
#include <fcntl.h> 
#include "e2.h"

int make_rnd_file(unsigned size, unsigned seed, unsigned mod, char *filename) {
    
    int res, data, fd, i, magic = MAGIC_NUMBER;
    
    // set pseudo-random generator seed
    srand(seed);

    // open file
    mode_t mode = S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH;
    fd = open(filename, O_WRONLY | O_TRUNC | O_CREAT, mode);
    check_perror(fd, "open");

    // write file magic code
    res = write(fd, &magic, sizeof(magic));
    check_perror(res, "write");

    // write size of the file in terms of random ints
    res = write(fd, &size, sizeof(size));
    check_perror(res, "write");

    // write random data
    for (i=0; i < size; ++i){
        data = rand() % mod;
        res = write(fd, &data, sizeof(data));
        check_perror(res, "write");
    }
    
    res = close(fd);
    check_perror(res, "write");

    return EXIT_SUCCESS;
}
