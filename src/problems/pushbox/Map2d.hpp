/*
 * 2Dmap.hpp
 *
 *  Created on: 18 Sep 2014
 *      Author: kaymes
 */

#pragma once

#include <vector>
#include <string>
#include <sstream>
#include <fstream>


namespace pushbox {

/* A 2-dimensional map. Individual fields can contain any value type.
 * However, for loading from a file, it is required that the value can
 * be constructed from a char.
 */
template<class ValueType>
class Map2D {
	typedef Map2D This;
public:
	typedef ValueType value_type;
private:
	typedef std::vector<value_type> Storage;
public:
	typedef typename Storage::iterator iterator;
	typedef typename Storage::const_iterator const_iterator;

	Map2D(): xSize(0), ySize(0), storage() {}
	Map2D(const size_t theSizeX, const size_t theSizeY, const value_type& defaultValue): xSize(theSizeX), ySize(theSizeY), storage(xSize*ySize, defaultValue) {}

	size_t sizeX() const { return xSize; }
	size_t sizeY() const { return ySize; }

	value_type& operator()(const size_t x, const size_t y) { return storage[x + y*xSize]; }
	const value_type& operator()(const size_t x, const size_t y) const { return storage[x + y*xSize]; }

	iterator begin() { return storage.begin(); }
	const_iterator begin() const { return storage.begin(); }
	iterator end() { return storage.end(); }
	const_iterator end() const { return storage.end(); }


	void clear() { xSize = 0; ySize=0; storage.clear(); }
	void loadFromFile(const std::string& filename);
	void loadFromStream(std::istream& is);


private:
	size_t xSize;
	size_t ySize;
	Storage storage;
};



template<class ValueType>
void Map2D<ValueType>::loadFromFile(const std::string& filename) {
	std::ifstream is;
	is.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	is.open(filename);
	is.exceptions(std::ifstream::goodbit);
	loadFromStream(is);
}

template<class ValueType>
void Map2D<ValueType>::loadFromStream(std::istream& is) {
	std::vector<std::string> lines;

	while(!is.eof()) {
		lines.push_back("");
		while(true) {
			char c = is.get();
			if ( is.eof() || (c == '\n')) break;
			lines.back() += c;
		}
	}

	// delete trailing empty lines.
	while (!lines.empty() && (lines.back() == "")) {
		lines.erase(lines.end()-1);
	}


	if (lines.empty()) {
		clear();
		return;
	}

	xSize = lines[0].size();
	ySize = lines.size();
	storage.resize(xSize*ySize);

	for (size_t i=0; i<lines.size(); i++) {
		if (lines[i].size() != xSize) {
			auto oldXSize = xSize;
			clear();

			std::stringstream ss;

			class MapDataInvalid: public std::exception {
				virtual const char* what() const noexcept {
					return message.c_str();
				}
			public:
				std::string message;
			} e;
			ss << "Map data is invalid. Line " << i << " has length " << lines[i].size() << ". Expected length " << oldXSize << ".";
			e.message = ss.str();
			throw e;

			return;
		}
	}

	for (size_t y=0; y<ySize; y++) {
		for (size_t x=0; x<xSize; x++) {
			// reverse the lines so the origin is the lower left corner and the y-coordinates are positive.
			operator()(x,y)=value_type(lines[ySize - 1 - y][x]);
		}
	}
}




} // namespace contnav
