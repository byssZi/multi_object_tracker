#include "multi_object_tracker/IDManager.h"

IDManager::IDManager() : next_id(1) {}

// 分配一个新的ID
int IDManager::allocateID() {
    int id;
    if (!recycled_ids.empty()) {
        id = recycled_ids.front();
        recycled_ids.pop();
    } else {
        id = next_id++;
    }
    active_ids.insert(id);
    return id;
}

// 回收一个ID
void IDManager::recycleID(int id) {
    if (active_ids.erase(id) > 0) {
        recycled_ids.push(id);
    }
}