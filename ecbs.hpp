#ifndef _ECBS_H
#define _ECBS_H

#include <vector>
#include <unordered_set>

using std::hash;
using std::unordered_set;

enum Action {
    Up, Down, Left, Right, Wait
};

struct State {
    int x, y, Time;
    State(int x, int y, int Time) : x(x), y(y), Time(Time) {}
    bool operator==(const State &s) const {
        return x == s.x && y == s.y && Time == s.Time;
    }
};

struct Conflict {
    int agent1, agent2, Time;
};

struct VertexConstraint {
    int x, y, Time;
    VertexConstraint(int x, int y, int Time) : x(x), y(y), Time(Time) {}
    bool operator==(const VertexConstraint &vc) const {
        return x == vc.x && y == vc.y && Time == vc.Time;
    }
};

struct EdgeConstraint {
    int x1, y1, x2, y2, Time;
    EdgeConstraint(int x1, int y1, int x2, int y2, int Time) : x1(x1), y1(y1), x2(x2), y2(y2), Time(Time) {}
    bool operator==(const EdgeConstraint &ec) const {
        return x1 == ec.x1 && y1 == ec.y1 && x2 == ec.x2 && y2 == ec.y2 && Time == ec.Time;
    }
};

struct Constraint {
    unordered_set<VertexConstraint> vertexConstraints;
    unordered_set<EdgeConstraint> edgeConstraints;

    void addVertexConstraint(int x, int y, int Time) {
        vertexConstraints.insert(VertexConstraint(x, y, Time));
    }

    void addEdgeConstraint(int x1, int y1, int x2, int y2, int Time) {
        edgeConstraints.insert(EdgeConstraint(x1, y1, x2, y2, Time));
    }

    bool hasConflict(const State &s) const {
        return vertexConstraints.find(VertexConstraint(s.x, s.y, s.Time)) != vertexConstraints.end();
    }
};

template <class T>
inline void HashCombine(std::size_t& seed, const T& value) {
    hash<T> hasher;
    seed ^= hasher(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

namespace std {
    template <>
    struct hash<State> {
        size_t operator()(const State &s) const {
            size_t seed = 0;
            HashCombine(seed, s.x);
            HashCombine(seed, s.y);
            HashCombine(seed, s.Time);
            return seed;
        }
    };

    template <>
    struct hash<VertexConstraint> {
        size_t operator()(const VertexConstraint &vc) const {
            size_t seed = 0;
            HashCombine(seed, vc.x);
            HashCombine(seed, vc.y);
            HashCombine(seed, vc.Time);
            return seed;
        }
    };

    template <>
    struct hash<EdgeConstraint> {
        size_t operator()(const EdgeConstraint &ec) const {
            size_t seed = 0;
            HashCombine(seed, ec.x1);
            HashCombine(seed, ec.y1);
            HashCombine(seed, ec.x2);
            HashCombine(seed, ec.y2);
            HashCombine(seed, ec.Time);
            return seed;
        }
    };
}

class ECBS {
public:
    unordered_set<State> OriginStates;

    // add robot origin state
    void addOriginState(int x, int y, int Time) {
        OriginStates.insert(State(x, y, Time));
    }
};

#endif