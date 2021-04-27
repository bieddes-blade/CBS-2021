#ifndef CONSTR_H
#define CONSTR_H

struct Constraint {
    // vertex constraint: the agent is not allowed to be in v1 at a certain time
    // edge constraint: the agent is not allowed to be in v1 and try to reach v2 at a certain time
    std::string type;
    int agent;
    int time;
    std::pair<int, int> v1;
    std::pair<int, int> v2;

    Constraint(std::string type, int agent, int time, std::pair<int, int> v1, std::pair<int, int> v2)
        : type(type)
        , agent(agent)
        , time(time)
        , v1(v1)
        , v2(v2)
    { }
};

struct Conflict {
    // vertex conflict: agent[0] and agent[1] collided in v1 at a certain time
    // edge conflict: agent[0] was traveling from v1 to v2 at a certain time, agent2 was traveling from v2 to v1
    // none conflict
    std::string type;
    std::pair<int, int> agents;
    int time1;
    int time2;
    std::pair<int, int> v1;
    std::pair<int, int> v2;

    Conflict(std::string type, std::pair<int, int> agents, int time1, int time2, std::pair<int, int> v1, std::pair<int, int> v2)
        : type(type)
        , agents(agents)
        , time1(time1)
        , time2(time2)
        , v1(v1)
        , v2(v2)
    { }

    Conflict(std::string type)
        : type(type)
        , agents(std::pair<int, int>())
        , time1(0)
        , time2(0)
        , v1(std::pair<int, int>())
        , v2(std::pair<int, int>())
    { }
};

#endif