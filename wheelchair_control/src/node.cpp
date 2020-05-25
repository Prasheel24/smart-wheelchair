#include <numeric>
#include <set>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <algorithm>
#include <string>

#include "node.hpp"

void node::setPosition(float x, float y) {
    position = std::make_pair(x, y);
}

void node::setParentIdx(int idx) {
    parentIdx = idx;
}

void node::setIdx(int i) {
    idx = i;
}

std::pair<float, float> node::getPosition() {
    return position;
}

int node::getParentIdx() {
    return parentIdx;
}

int node::getIdx() {
    return idx;
}
