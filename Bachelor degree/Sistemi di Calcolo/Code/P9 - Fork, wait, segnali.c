//Si vuole scrivere un programma che crea n figli tale che il figlio i-esimo dorme i secondi, con i=1,...,n e termina con successo. Per ogni figlio creato, il programma deve stampare "- creato figlio <pid>", dove pid è il pid del processo generato. Dopo aver creato i figli, il genitore rimane in attesa perenne. La terminazione dei figli è catturata tramite il segnale SIGCHLD. Il gestore del segnale (che gira nel processo genitore) fa una wait; quando l’ultimo processo termina, il gestore del segnale manda in segnale SIGTERM a se stesso per terminare anche il genitore. Per ogni figlio terminato, il gestore del segnale stampa "* terminato figlio <pid>", dove pid è il pid del processo terminato.
//Parametro il numero n di processi da creare.

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/wait.h>

int last;

void handler(int signum) {
    int status;
    pid_t pid = wait(&status);
    if (pid == -1) {
        perror("wait error");
        exit(EXIT_FAILURE);
    }
    printf("* terminato figlio %d\n", pid);
    if (pid == last)
        kill(getpid(), SIGTERM);
}

void do_work(int n){

    int i;

    struct sigaction act = { 0 }; 
    act.sa_handler = handler; 
    int ret = sigaction(SIGCHLD, &act, NULL);
    if (ret == -1) {
        perror("sigaction error");
        exit(EXIT_FAILURE);
    }

    for (i=1; i<=n; ++i) {

        pid_t pid = fork();

        if (pid == -1) {
            perror("fork error");
            exit(EXIT_FAILURE);
        }
        
        if (pid == 0) {
            printf("- creato figlio %d\n", getpid());
            sleep(i);
            _exit(EXIT_SUCCESS);
        }

        if (pid > 0 && i == n) 
            last = pid;
    }

    while(1)
        pause();
}
