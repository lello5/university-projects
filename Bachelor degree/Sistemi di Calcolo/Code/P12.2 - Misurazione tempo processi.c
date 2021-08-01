//Scrivere una funzione proc_gettime che prende come parametri il pathname di un eseguibile file, un puntatore a una funzione callback get_arg fornita dall’utente, un puntatore a dei dati data da fornire alla funzione callback, e un numero di ripetizioni n. La funzione proc_gettime esegue file per n volte, misurando il tempo in secondi (double) richiesto dall’esecuzione e restituisce la media dei tempi richiesti sulle n esecuzioni. Ad ogni esecuzione, la funzione chiama la callback per ottenere gli argomenti argv da passare al processo in modo da rendere possibile comportamenti diversi del processo ad ogni ripetizione. La callback deve ricevere il puntatore data preso come parametro da proc_gettime e il numero della ripetizione i con i compreso tra 0 e n-1.

#include "e2.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>


double get_real_time() {
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return tp.tv_sec + tp.tv_nsec*1E-9;
}

double proc_gettime(const char* file, char** (*get_argv)(int i, void* data), void *data, int n){
    int i;
    double start, elapsed = 0.0;
    for (i=0; i<n; ++i) {
        pid_t pid = fork();
        if (pid == -1) {
            perror("fork");
            exit(EXIT_FAILURE);
        }
        if (pid == 0) {
            execvp(file, get_argv(i,data));
            perror("execvp");
            _exit(EXIT_FAILURE);
        }
        start = get_real_time();
        pid = wait(NULL);
        if (pid == -1) {
            perror("wait");
            exit(EXIT_FAILURE);
        }
        elapsed += get_real_time() - start;
    }
    return elapsed/n;
}
