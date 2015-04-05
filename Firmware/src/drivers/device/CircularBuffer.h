/*
 * CircularBuffer.h
 *
 *  Created on: Apr 3, 2015
 *      Author: Lucifer
 */

#ifndef FIRMWARE_SRC_DRIVERS_DEVICE_CIRCULARBUFFER_H_
#define FIRMWARE_SRC_DRIVERS_DEVICE_CIRCULARBUFFER_H_

class CircularBuffer {
public:
	CircularBuffer(int maxCapacity) {
		_maxCapacity = maxCapacity;
		_head = _tail = 0;
		_size = 0;

		_buf = new char[_maxCapacity];
	}
	virtual ~CircularBuffer() {
		delete[] _buf;
	}
	void put(const char* data, int size) {
		if (size == nullptr)
			return;// nothing to do
		if ((_maxCapacity - _size) < size)
			return;	// overflow
	}
	char* _buf;
	int _head;
	int _tail;
	int _size;
	int _maxCapacity;
};



#endif /* FIRMWARE_SRC_DRIVERS_DEVICE_CIRCULARBUFFER_H_ */
