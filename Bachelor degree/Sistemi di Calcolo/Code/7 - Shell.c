/*
Si chiede di scrivere una semplice shell sotto forma di una funzione int do_shell(const char* prompt); che prende come parametro la stringa di prompt e si comporta come segue:

    1) Stampa il prompt;
    2) Attende che l’utente inserisca in stdin un comando seguito dai suoi eventuali argomenti (es: ls -l, dove ls è il comando e -l è il suo unico argomento). Per ottenere comando e argomenti da stdin usare il risultato dell’esercizio 2;
    3) Se il comando è vuoto (NULL) tornare al punto 1;
    4) Se il comando è quit terminare con successo la shell;
    5) Creare con fork un nuovo processo che esegua il comando con gli argomenti dati usando execvp;
    6) Se il comando si riferisce a un programma inesistente riportare l’errore unknown command seguito dal nome del comando e tornare al punto 1;
    7) Attendere con wait la terminazione del processo e tornare al punto 1.
*/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "e2.h"

#define MAX_LINE    1024
#define MAX_TOKENS  64

void do_cmd(char* argv[MAX_TOKENS]) {
    int res;
    pid_t pid = fork();

    if (pid == -1) {
        perror("fork error");
        exit(EXIT_FAILURE);
    }

    if (pid == 0) {
        res = execvp(argv[0], argv);
        if (res == -1) {
            printf("unknown command %s\n", argv[0]);
            _exit(EXIT_FAILURE);
        }
    }

    res = wait(NULL);
    if (pid == -1) {
        perror("wait error");
        exit(EXIT_FAILURE);
    }
}

char* dup_string(const char* in) {
    size_t n = strlen(in);
    char* out = malloc(n + 1);
    strcpy(out, in);
    return out;
}

void get_cmd_line(char* argv[MAX_TOKENS]) {
    int argc = 0;
    char line[MAX_LINE];
    fgets(line, MAX_LINE, stdin);
    char* token = strtok(line, " \t\n");
    argc = 0;
    while (argc < MAX_TOKENS && token != NULL) {
        argv[argc++] = dup_string(token);
        token = strtok(NULL, " \t\n");
    }
    argv[argc] = NULL;
}

int do_shell(const char* prompt){
    for (;;) {
        printf("%s", prompt);
        char* argv[MAX_TOKENS];
        get_cmd_line(argv);
        if (argv[0] == NULL) continue;
        if (strcmp(argv[0], "quit") == 0) break;
        do_cmd(argv);
    }
    return EXIT_SUCCESS;
}
