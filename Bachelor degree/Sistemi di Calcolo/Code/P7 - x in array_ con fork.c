/*
Scrivere una funzione C int par_find(int* v, unsigned n, int x) che cerca se x appartiene all’array v di dimensione n. La soluzione deve usare almeno due processi distinti che effettuano la ricerca in sottoparti distinte di v in modo indipendente l’uno dall’altro.

Suggerimento: fare in modo che il processo restituisca al genitore come stato di exit 1 se x è presente nella porzione esplorata dal processo, e 0 altrimenti.
*/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "e1.h"

#define PROC 10

int find(int* v, unsigned n, int x) {
    int i;
    for (i=0; i<n; ++i)
        if (v[i]==x) return 1;
    return 0;
}

int par_find(int* v, unsigned n, int x){
    int i, res = 0;
    for (i=0; i<PROC; ++i) {
        pid_t pid = fork();
        if (pid == -1) {
            perror("fork");
            _exit(EXIT_FAILURE);
        }
        if (pid == 0) {
            int res = find(v+i*n/PROC, n/PROC, x);
            _exit(res);
        }
    }
    for (i=0; i<PROC; ++i) {
        int status;
        wait(&status);
        if (WIFEXITED(status)) {
            res = res || WEXITSTATUS(status);
        }
    }
    return res || v[n-1] == x;
}
