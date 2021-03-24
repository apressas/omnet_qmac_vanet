#include "veins/modules/mobility/traci/TraCIBuffer.h"
#include "veins/modules/mobility/traci/TraCIConstants.h"
#include "veins/modules/mobility/traci/TraCICoord.h"

#include <iomanip>
#include <sstream>

namespace Veins {

TraCIBuffer::TraCIBuffer() : buf() {
	buf_index = 0;
}

TraCIBuffer::TraCIBuffer(std::string buf) : buf(buf) {
	buf_index = 0;
}

bool TraCIBuffer::eof() const {
	return buf_index == buf.length();
}

void TraCIBuffer::set(std::string buf) {
	this->buf = buf;
	buf_index = 0;
}

void TraCIBuffer::clear() {
	set("");
}

std::string TraCIBuffer::str() const {
	return buf;
}

std::string TraCIBuffer::hexStr() const {
	std::stringstream ss;
	for (std::string::const_iterator i = buf.begin() + buf_index; i != buf.end(); ++i) {
		if (i != buf.begin()) ss << " ";
		ss << std::hex << std::setw(2) << std::setfill('0') << (int)(uint8_t)*i;
	}
	return ss.str();
}

template<> void TraCIBuffer::write(std::string inv) {
	uint32_t length = inv.length();
	write<uint32_t> (length);
	for (size_t i = 0; i < length; ++i) write<char> (inv[i]);
}

template<> std::string TraCIBuffer::read() {
	uint32_t length = read<uint32_t> ();
	if (length == 0) return std::string();
	char obuf[length + 1];

	for (size_t i = 0; i < length; ++i) read<char> (obuf[i]);
	obuf[length] = 0;

	return std::string(obuf, length);
}

template<> void TraCIBuffer::write(TraCICoord inv) {
	write<uint8_t>(POSITION_2D);
	write<double>(inv.x);
	write<double>(inv.y);
}

template<> TraCICoord TraCIBuffer::read() {
	uint8_t posType = read<uint8_t>();
	ASSERT(posType == POSITION_2D);

	TraCICoord p;
	p.x = read<double>();
	p.y = read<double>();

	return p;
}

bool isBigEndian() {
	short a = 0x0102;
	unsigned char *p_a = reinterpret_cast<unsigned char*>(&a);
	return (p_a[0] == 0x01);
}

}
