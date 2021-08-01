/*
L’obiettivo dell’esercizio è implementare l’operazione di deallocazione di un semplice allocatore di memoria. L’operazione mymalloc è fornita e si richiede di scrivere l’operazione myfree. L’operazione deve prendere il blocco deallocato e inserirlo in testa alla lista di blocchi liberi puntata dalla variabile globale free_list come descritto nella dispensa del corso. I blocchi liberi dell’allocatore hanno una header di 4 byte che contiene la dimensione del blocco, seguita da un campo next.
*/

#include <unistd.h>
#include <stdlib.h>
#include "e2.h"

header_t* free_list = NULL;

void* mymalloc(size_t m) {
    header_t *p, *q = NULL;
    m = ((m+3)/4)*4;                // arrotonda al più piccolo multiplo di 4 maggiore o uguale a m
    if (m < 8) m = 8;               // garantisce spazio per il puntatore next
    for (p=free_list; p != NULL; q = p, p = p->next)    // cerca blocco libero first-fit
        if (p->size >= m) break;
    if (p == NULL) p = sbrk(4 + m); // nessun blocco libero, si espande l'heap
    else                            // toglie nodo dalla lista dei blocchi liberi
        if (q == NULL) free_list = free_list->next;
        else q->next = q->next->next;
    ((header_t*)p)->size = m;       // size misura il blocco a meno del campo size stesso  
    return (char*)p+4;
}


void myfree(void* p) {
	header_t * q = (header_t*) (p - 4);
	q->next = free_list;
	free_list = q;
}


