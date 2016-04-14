#include "list.h"

//void l_init(list_t *list) {
//	list->size = 0;
//	for (int i = 0; i < MAX_SIZE; i++) {
//		list->exists[i] = false;
//	}
//}
//
///**
// * Return index, or -1 if doesn't exist
// */
//int l_in_list(list_t *list, node_t n) {
//	for (int i = 0; i < MAX_SIZE; i++) {
//		if (list->items[i].point.x == n.point.x && list->items[i].point.y == n.point.y) {
//			return i;
//		}
//	}
//
//	return -1;
//}
//
//void l_add(list_t *list, node_t t) {
//	int free_index;
//	for (int i = 0; i < MAX_SIZE; i++) {
//		if (!list->exists[i]) {
//			free_index = i;
//			break;
//		}
//	}
//
//	list->items[free_index] = t;
//	list->exists[free_index] = true;
//	list->size++;
//}
//
//void l_remove(list_t *list, int index) {
//	list->exists[index] = false;
//	list->size--;
//}
//
//bool l_is_empty(list_t *list) {
//	return list->size == 0;
//}
