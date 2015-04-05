/*
 * RingBuffer.h
 *
 *  Created on: Apr 1, 2015
 *      Author: Lucifer
 */

#ifndef FIRMWARE_SRC_DRIVERS_HMC5883L_RINGBUFFER_H_
#define FIRMWARE_SRC_DRIVERS_HMC5883L_RINGBUFFER_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>

class RingBuffer {
public:
	RingBuffer(unsigned num_items, size_t item_size) :
			_item_size(item_size) {
		_num_items = num_items;
		_buf = new char[(_num_items + 1) * item_size];
		_head = _num_items;
		_tail = _num_items;
	}
	virtual ~RingBuffer() {
		if (_buf != nullptr)
			delete[] _buf;
	}

	/**
	 * Put an item into the buffer.
	 *
	 * @param val
	 * 		Item to put
	 * @return true if the item was put, false if the buffer is full
	 */
	bool put(const void* val, size_t val_size = 0) {
		unsigned inext = next(_head);
		if (inext != _tail) {
			if ((val_size == 0) || (val_size > _item_size))
				val_size = _item_size;

			memcpy(&_buf[_head * _item_size], val, val_size);
			_head = inext;
			return true;
		} else {
			return false;
		}
	}

	bool put(int8_t val) {
		return put(&val, sizeof(val));
	}
	bool put(uint8_t val) {
		return put(&val, sizeof(val));
	}
	bool put(int16_t val) {
		return put(&val, sizeof(val));
	}
	bool put(uint16_t val) {
		return put(&val, sizeof(val));
	}
	bool put(int32_t val) {
		return put(&val, sizeof(val));
	}
	bool put(uint32_t val) {
		return put(&val, sizeof(val));
	}
	bool put(int64_t val) {
		return put(&val, sizeof(val));
	}
	bool put(uint64_t val) {
		return put(&val, sizeof(val));
	}
	bool put(float val) {
		return put(&val, sizeof(val));
	}
	bool put(double val) {
		return put(&val, sizeof(val));
	}

	/**
	 * Force an item into the buffer, discarding an older item if there is not space.
	 *
	 * @param val
	 * 		Item to put
	 * @return true if an item was discarded to make space
	 */
	bool force(const void* val, size_t val_size = 0) {
		bool overwrote = false;
		for (;;) {
			if (put(val, val_size))
				break;

			get (NULL);
			overwrote = true;
		}
		return overwrote;
	}

	bool force(int8_t val) {
		return force(&val, sizeof(val));
	}
	bool force(uint8_t val) {
		return force(&val, sizeof(val));
	}
	bool force(int16_t val) {
		return force(&val, sizeof(val));
	}
	bool force(uint16_t val) {
		return force(&val, sizeof(val));
	}
	bool force(int32_t val) {
		return force(&val, sizeof(val));
	}
	bool force(uint32_t val) {
		return force(&val, sizeof(val));
	}
	bool force(int64_t val) {
		return force(&val, sizeof(val));
	}
	bool force(uint64_t val) {
		return force(&val, sizeof(val));
	}
	bool force(float val) {
		return force(&val, sizeof(val));
	}
	bool force(double val) {
		return force(&val, sizeof(val));
	}

	/**
	 * Get an item from the buffer.
	 *
	 * @param val
	 * 		Item that was gotten
	 * @return true if an item was got, false if the buffer was empty.
	 */
	bool get(void* val, size_t val_size = 0) {
		if (_tail != _head) {
			unsigned candidate;
			unsigned inext;
			if ((val_size == 0) || (val_size > _item_size))
				val_size = _item_size;

			do {
				/* decide which element we think we're going to read */
				candidate = _tail;
				/* and what the corresponding next index will be */
				inext = next(candidate);
				/* go ahead and read from this index */
				if (val != NULL)
					memcpy(val, &_buf[candidate * _item_size], val_size);
				/* if the tail pointer didn't change, we got our item */
			} while (!__sync_bool_compare_and_swap(&_tail, candidate, inext));
			return true;
		} else {
			return false;
		}
	}

	bool get(int8_t& val) {
		return get(&val, sizeof(val));
	}
	bool get(uint8_t& val) {
		return get(&val, sizeof(val));
	}
	bool get(int16_t& val) {
		return get(&val, sizeof(val));
	}
	bool get(uint16_t& val) {
		return get(&val, sizeof(val));
	}
	bool get(int32_t& val) {
		return get(&val, sizeof(val));
	}
	bool get(uint32_t& val) {
		return get(&val, sizeof(val));
	}
	bool get(int64_t& val) {
		return get(&val, sizeof(val));
	}
	bool get(uint64_t& val) {
		return get(&val, sizeof(val));
	}
	bool get(float& val) {
		return get(&val, sizeof(val));
	}
	bool get(double& val) {
		return get(&val, sizeof(val));
	}

	/**
	 * Get the number of slots free in the buffer.
	 *
	 * @return The number of items that can be put into the buffer before
	 *		it becomes full.
	 */
	unsigned space() {
		unsigned tail, head;
		/*
		 * Make a copy of the head/tail pointers in a fashion that
		 * may err on the side of under-estimating the free space
		 * in the buffer in the case that the buffer is being updated
		 * asynchronously with our check.
		 * If the head pointer changes (reducing space) while copying,
		 * re-try the copy.
		 */
		do {
			head = _head;
			tail = _tail;
		} while (head != _head);
		return (tail >= head) ? (_num_items - (tail - head)) : (head - tail - 1);
	}

	/**
	 * Get the number of items in the buffer.
	 *
	 * @return	The number of items that can be got from the buffer before
	 *		it becomes empty.
	 */
	unsigned count() {
		/*
		 * Note that due to the conservative nature of space(), this may
		 * over-estimate the number of items in the buffer.
		 */
		return _num_items - space();
	}

	/**
	 * Returns true if the buffer is empty.
	 */
	bool empty() {
		return _tail == _head;
	}

	/**
	 * Returns true if the buffer is full.
	 */
	bool full() {
		return next(_head) == _tail;
	}

	/**
	 * Returns the capacity of the buffer, or zero if the buffer could
	 * not be allocated.
	 */
	unsigned size() {
		return (_buf != nullptr) ? _num_items : 0;
	}

	/**
	 * Empties the buffer.
	 */
	void flush() {
		while (!empty())
			get (NULL);
	}

	/**
	 * resize the buffer. This is unsafe to be called while
	 * a producer or consuming is running. Caller is responsible
	 * for any locking needed
	 *
	 * @param new_size	new size for buffer
	 * @return true if the resize succeeds, false if
	 * 		not (allocation error)
	 */
	bool resize(unsigned new_size) {
		char* old_buffer;
		char* new_buffer = new char[(new_size + 1) * _item_size];
		if (new_buffer == nullptr) {
			return false;
		}
		old_buffer = _buf;
		_buf = new_buffer;
		_num_items = new_size;
		_head = new_size;
		_tail = new_size;
		delete[] old_buffer;
		return true;
	}

	/**
	 * printf() some info on the buffer
	 */
	void print_info(const char* name) {
		printf("%s	%u/%u (%u/%u @ %p)\n", name, _num_items,
				_num_items * _item_size, _head, _tail, _buf);
	}
	unsigned next(unsigned index) {
		return (0 == index) ? _num_items : (index - 1);
	}

private:
	unsigned _num_items;
	const size_t _item_size;
	char *_buf;
	volatile unsigned _head; // insertion point in _item_size units
	volatile unsigned _tail; // removal point in _item_size units
};

#endif /* FIRMWARE_SRC_DRIVERS_HMC5883L_RINGBUFFER_H_ */
