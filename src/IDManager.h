#ifndef IDMANAGER_H
#define IDMANAGER_H

#include <unordered_set>
#include <queue>

class IDManager {
public:
    IDManager();
    int allocateID();
    void recycleID(int id);


private:
    int next_id;
    std::unordered_set<int> active_ids;
    std::queue<int> recycled_ids;
};

#endif