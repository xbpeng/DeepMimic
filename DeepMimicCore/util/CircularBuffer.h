#pragma once
#include <vector>

template<typename tVal, typename tAlloc = std::allocator<tVal>>
class cCircularBuffer
{
public:
	cCircularBuffer();
	cCircularBuffer(size_t capacity);
	virtual ~cCircularBuffer();

	virtual void Reserve(size_t capacity);
	virtual void Clear();
	virtual size_t GetSize() const;
	virtual size_t GetCapacity() const;

	virtual void Add(const tVal& val);
	virtual const tVal& operator[](size_t i) const;
	virtual tVal& operator[](size_t i);

protected:
	size_t mHead;
	size_t mSize;
	std::vector<tVal, tAlloc> mData;

	virtual size_t CalcIdx(size_t i) const;
};

template<typename tVal, typename tAlloc>
cCircularBuffer<tVal, tAlloc>::cCircularBuffer() :
		cCircularBuffer(0)
{
}

template<typename tVal, typename tAlloc>
cCircularBuffer<tVal, tAlloc>::cCircularBuffer(size_t capacity)
{
	mHead = 0;
	mSize = 0;
	Reserve(capacity);
}

template<typename tVal, typename tAlloc>
cCircularBuffer<tVal, tAlloc>::~cCircularBuffer()
{
}

template<typename tVal, typename tAlloc>
void cCircularBuffer<tVal, tAlloc>::Reserve(size_t capacity)
{
	size_t prev_size = mData.size();
	mData.resize(capacity);

	if (prev_size > 0 && capacity > prev_size)
	{
		for (size_t i = mHead; i < prev_size; ++i)
		{
			size_t old_idx = prev_size - 1 - (i - mHead);
			size_t new_idx = capacity - 1 - (i - mHead);
			mData[new_idx] = mData[i];
		}
	}

	mSize = std::min(mSize, capacity);
	if (mHead > capacity)
	{
		mHead = 0;
	}
}

template<typename tVal, typename tAlloc>
void cCircularBuffer<tVal, tAlloc>::Clear()
{
	mHead = 0;
	mSize = 0;
}

template<typename tVal, typename tAlloc>
size_t cCircularBuffer<tVal, tAlloc>::GetSize() const
{
	return mSize;
}

template<typename tVal, typename tAlloc>
size_t cCircularBuffer<tVal, tAlloc>::GetCapacity() const
{
	return mData.size();
}

template<typename tVal, typename tAlloc>
void cCircularBuffer<tVal, tAlloc>::Add(const tVal& val)
{
	mData[mHead] = val;
	size_t capacity = GetCapacity();
	mHead = (mHead + 1) % capacity;
	mSize = std::min((mSize + 1), capacity);
}

template<typename tVal, typename tAlloc>
const tVal& cCircularBuffer<tVal, tAlloc>::operator[](size_t i) const
{
	size_t idx = CalcIdx(i);
	return mData[idx];
}

template<typename tVal, typename tAlloc>
tVal& cCircularBuffer<tVal, tAlloc>::operator[](size_t i)
{
	size_t idx = CalcIdx(i);
	return mData[idx];
}

template<typename tVal, typename tAlloc>
size_t cCircularBuffer<tVal, tAlloc>::CalcIdx(size_t i) const
{
	size_t idx = (mHead + i) % GetSize();
	return idx;
}