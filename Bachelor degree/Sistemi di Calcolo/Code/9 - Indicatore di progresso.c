/*
Si vogliono usare i segnali per creare un indicatore di progresso per la funzione do_sort, che implementa un semplice algoritmo di ordinamento a bolle. L’indicatore di progresso è la percentuale di n coperta da i, vale a dire 100.0*i/n.
*/

#include "e2.h"
#include "signal.h"
#include "stdlib.h"
#include "stdio.h"
int i, max;

void handler(int signo){
	float perc = (float) 100.0 * i/max;
	printf("%f%%\n", perc);
	ualarm(500000U, 0);
}

static void do_sort(int *v, int n) {
    int j;
    for (i=0; i<n; ++i)
        for (j=1; j<n; ++j)
            if (v[j-1] > v[j]) {
                int tmp = v[j-1];
                v[j-1] = v[j];
                v[j] = tmp;
            }
}

void sort(int *v, int n) {
    max = n;
    struct sigaction a = {0};
    a.sa_handler = handler;
    int res = sigaction(SIGALRM, &a, NULL);
    if (res == -1){
    	perror("errore sigaction");
    	exit(EXIT_FAILURE);
    }
    ualarm(500000U, 500000U);
    do_sort(v, n);
}

