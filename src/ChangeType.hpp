#ifndef CHANGETYPE_HPP
#define CHANGETYPE_HPP

enum class ChangeType {
    UNDEFINED = 0,
    ADDSTATE = 1,
    REWARD = 2,
    TRANSITION = 3,
    DELSTATE = 4,
    ADDOBSERVATION = 5,
    ADDOBSTACLE = 6
};

#endif /* CHANGETYPE_HPP */
