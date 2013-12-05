#ifndef ROCKSAMPLETEXTSERIALIZER_HPP
#define ROCKSAMPLETEXTSERIALIZER_HPP

#include "TextSerializer.hpp"

class RockSampleTextSerializer: public TextSerializer {
public:
    RockSampleTextSerializer();
    RockSampleTextSerializer(Solver *solver);
    virtual ~RockSampleTextSerializer() = default;
    RockSampleTextSerializer(RockSampleTextSerializer const &) = delete;
    RockSampleTextSerializer(RockSampleTextSerializer &&) = delete;
    RockSampleTextSerializer &operator=(RockSampleTextSerializer const &) = delete;
    RockSampleTextSerializer &operator=(RockSampleTextSerializer &&) = delete;

    void save(State &state, std::ostream &os);
    void load(State &state, std::istream &is);
};

#endif /* ROCKSAMPLETEXTSERIALIZER_HPP */
