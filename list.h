#ifndef __LIST_H_
#define __LIST_H_

#include "defs.h"

void l_init(list_t *list);
void l_add(list_t *list, node_t t);
void l_remove(list_t *list, int index);
int l_in_list(list_t *list, node_t n);
bool l_is_empty(list_t *list);

#endif
