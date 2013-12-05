#ifndef TAGTEXTSERIALIZER_HPP
#define TAGTEXTSERIALIZER_HPP

#include "TextSerializer.hpp"

class TagTextSerializer: public TextSerializer {
public:
    TagTextSerializer();
    TagTextSerializer(Solver *solver);
    virtual ~TagTextSerializer() = default;
    TagTextSerializer(TagTextSerializer const &) = delete;
    TagTextSerializer(TagTextSerializer &&) = delete;
    TagTextSerializer &operator=(TagTextSerializer const &) = delete;
    TagTextSerializer &operator=(TagTextSerializer &&) = delete;

    void saveState(State &state, std::ostream &os);
    std::unique_ptr<State> loadState(std::istream &is);
};

#endif /* TAGTEXTSERIALIZER_HPP */
