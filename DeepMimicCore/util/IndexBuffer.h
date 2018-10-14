#pragma once
#include <vector>
#include <assert.h>

template<typename tEntry, typename tAlloc = std::allocator<tEntry>>
class cIndexBuffer
{
public:
	cIndexBuffer();
	virtual ~cIndexBuffer();

	virtual size_t GetSize() const;
	virtual size_t GetCapacity() const;
	virtual void Clear();

	virtual size_t Add(const tEntry& entry);
	virtual void Free(size_t idx);
	virtual size_t GetNumFree() const;
	virtual bool IsFree(size_t idx) const;

	virtual const tEntry& operator[](size_t idx) const;
	virtual tEntry& operator[](size_t idx);

protected:
	std::vector<tEntry, tAlloc> mEntries;
	std::vector<size_t> mFreeIndices;
};

template<typename tEntry, typename tAlloc>
cIndexBuffer<tEntry, tAlloc>::cIndexBuffer()
{
	Clear();
}

template<typename tEntry, typename tAlloc>
cIndexBuffer<tEntry, tAlloc>::~cIndexBuffer()
{
	Clear();
}

template<typename tEntry, typename tAlloc>
size_t cIndexBuffer<tEntry, tAlloc>::GetSize() const
{
	assert(GetCapacity() >= GetNumFree());
	return GetCapacity() - GetNumFree();
}

template<typename tEntry, typename tAlloc>
size_t cIndexBuffer<tEntry, tAlloc>::GetCapacity() const
{
	return mEntries.size();
}

template<typename tEntry, typename tAlloc>
void cIndexBuffer<tEntry, tAlloc>::Clear()
{
	mEntries.clear();
	mFreeIndices.clear();
}

template<typename tEntry, typename tAlloc>
size_t cIndexBuffer<tEntry, tAlloc>::Add(const tEntry& entry)
{
	size_t idx = 0;
	if (GetNumFree() > 0)
	{
		idx = mFreeIndices[GetNumFree() - 1];
		mFreeIndices.pop_back();
		mEntries[idx] = entry;
	}
	else
	{
		idx = GetCapacity();
		mEntries.push_back(entry);
	}
	return idx;
}

template<typename tEntry, typename tAlloc>
void cIndexBuffer<tEntry, tAlloc>::Free(size_t idx)
{
	assert(idx >= 0 && idx < GetCapacity());
	if (!IsFree(idx))
	{
		bool is_last = idx == GetCapacity() - 1;
		if (is_last)
		{
			mEntries.pop_back();

			size_t curr_idx = idx - 1;
			auto it = find(mFreeIndices.begin(), mFreeIndices.end(), curr_idx);
			while (it != mFreeIndices.end())
			{
				mEntries.pop_back();
				*it = mFreeIndices[GetNumFree() - 1];
				mFreeIndices.pop_back();

				if (curr_idx == 0)
				{
					break;
				}
				--curr_idx;
				it = find(mFreeIndices.begin(), mFreeIndices.end(), curr_idx);
			}
		}
		else
		{
			mFreeIndices.push_back(idx);
		}
	}
	else
	{
		assert(false); // entry already freed
	}
}

template<typename tEntry, typename tAlloc>
size_t cIndexBuffer<tEntry, tAlloc>::GetNumFree() const
{
	return mFreeIndices.size();
}

template<typename tEntry, typename tAlloc>
const tEntry& cIndexBuffer<tEntry, tAlloc>::operator[](size_t idx) const
{
	return mEntries[idx];
}

template<typename tEntry, typename tAlloc>
tEntry& cIndexBuffer<tEntry, tAlloc>::operator[](size_t idx)
{
	return mEntries[idx];
}

template<typename tEntry, typename tAlloc>
bool cIndexBuffer<tEntry, tAlloc>::IsFree(size_t idx) const
{
	auto it = std::find(mFreeIndices.begin(), mFreeIndices.end(), idx);
	return it != mFreeIndices.end();
}