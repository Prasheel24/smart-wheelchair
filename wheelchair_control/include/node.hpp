#ifndef INCLUDE_NODE_HPP_
#define INCLUDE_NODE_HPP_

#include <set>
#include <string>
#include <vector>
#include <utility>
#include <random>

class node {
 private:
    std::pair<float, float> position;

    int parentIdx = 0;

    int idx = 0;

 public:
    void setPosition(float x, float y);

    void setParentIdx(int idx);

    void setIdx(int i);

    std::pair<float, float> getPosition();

    int getParentIdx();

    int getIdx();
};

#endif  //  INCLUDE_NODE_HPP_

